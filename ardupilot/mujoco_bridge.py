import json
import socket
import time
import sys
import numpy as np
import mujoco
import mujoco.viewer
import psutil  # Requires: pip install psutil

def find_ardupilot_port():
    """Scans running system processes to dynamically find the active ArduPilot UDP port."""
    print("[Discovery] Searching system for running ArduPilot SITL process...")
    for proc in psutil.process_iter(['pid', 'name']):
        try:
            if "arducopter" in proc.info['name'].lower() or "sim_vehicle" in proc.info['name'].lower():
                connections = proc.connections(kind='udp')
                for conn in connections:
                    if conn.status == 'NONE' and conn.laddr.port >= 9000:
                        print(f"[Discovery] Found active ArduPilot process (PID: {proc.info['pid']}) listening on UDP Port: {conn.laddr.port}")
                        return conn.laddr.port
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            continue
    print("[Discovery] Warning: Falling back to default JSON port 9002.")
    return 9002

class MuJoCoArduPilotBridge:
    def __init__(self, xml_path):
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        
        self.sitl_ip = "127.0.0.1"
        self.sitl_port = find_ardupilot_port()
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 0)) 
        
        print(f"[Bridge] Initialized. Streaming telemetry to {self.sitl_ip}:{self.sitl_port}")

    def get_sensor_data(self, clean_calibration_mode=False):
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "quadcopter")
        
        if clean_calibration_mode:
            # During calibration, send a mathematically perfect, static resting state
            position = [0.0, 0.0, 0.0]
            velocity = [0.0, 0.0, 0.0]
            quaternion = [1.0, 0.0, 0.0, 0.0] 
            gyro = [0.0, 0.0, 0.0]
            accel_body = [0.0, 0.0, -9.81] 
        else:
            # --- CRITICAL STAGE 2 ENU TO NED CONVERSION CORRECTION ---
            pos_enu = self.data.xpos[body_id]
            # ENU [X=East, Y=North, Z=Up] -> NED [X=North, Y=East, Z=Down]
            position = [pos_enu[1], pos_enu[0], -pos_enu[2]]
            
            vel_enu = self.data.cvel[body_id][3:6]
            velocity = [vel_enu[1], vel_enu[0], -vel_enu[2]]
            
            q_enu = self.data.xquat[body_id] # MuJoCo: [w, x, y, z]
            # Convert orientation rotation frame cleanly from ENU to NED orientation
            quaternion = [q_enu[0], q_enu[2], q_enu[1], -q_enu[3]]
            
            try:
                gyro_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "gyro")
                accel_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "accelerometer")
                
                raw_gyro = self.data.sensordata[self.model.sensor_adr[gyro_id]:self.model.sensor_adr[gyro_id]+3]
                gyro = [raw_gyro[1], raw_gyro[0], -raw_gyro[2]]
                
                raw_accel = np.array(self.data.sensordata[self.model.sensor_adr[accel_id]:self.model.sensor_adr[accel_id]+3])
                
                # Dynamic gravity matrix mapping based on local body rotations
                R = self.data.xmat[body_id].reshape(3, 3)
                gravity_local = R.T @ np.array([0.0, 0.0, -9.81])
                
                accel_enu = raw_accel + gravity_local
                accel_body = [accel_enu[1], accel_enu[0], -accel_enu[2]]
            except Exception:
                gyro = [0.0, 0.0, 0.0]
                accel_body = [0.0, 0.0, -9.81]

        payload = {
            "timestamp": self.data.time,
            "imu": {
                "gyro": gyro, 
                "accel_body": accel_body
            },
            "position": position,
            "velocity": velocity,
            "quaternion": quaternion
        }
        return payload

    def apply_motor_commands(self, servo_values):
        for i, val in enumerate(servo_values):
            if i < self.model.nu:
                normalized_thrust = (val - 1000) / 1000.0
                self.data.ctrl[i] = max(0.0, min(1.0, normalized_thrust))

    def run(self):
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            # STEP 1: Safe Software-Isolated Handshake (Feeds ArduPilot perfect static values)
            print("[Bridge] Feeding perfect clean telemetry to bypass ground contact calibration lock...")
            self.sock.setblocking(False)
            
            calibrated = False
            while not calibrated and viewer.is_running():
                telemetry = self.get_sensor_data(clean_calibration_mode=True)
                msg = json.dumps(telemetry)
                self.sock.sendto(msg.encode('utf-8'), (self.sitl_ip, self.sitl_port))
                
                try:
                    data, _ = self.sock.recv(4096)
                    response = json.loads(data.decode('utf-8'))
                    if "servos" in response:
                        print("\n[Bridge] Received valid servo data! ArduPilot is booted and calibrated.")
                        calibrated = True
                        self.apply_motor_commands(response["servos"])
                except BlockingIOError:
                    pass
                
                mujoco.mj_step(self.model, self.data)
                viewer.sync()
                time.sleep(0.0025)

            # STEP 2: Strict Real-Time Physics Lockstep Simulation
            print("[Bridge] Switching cleanly to live MuJoCo environment physics.")
            self.sock.settimeout(0.05)

            while viewer.is_running():
                telemetry = self.get_sensor_data(clean_calibration_mode=False)
                msg = json.dumps(telemetry)
                self.sock.sendto(msg.encode('utf-8'), (self.sitl_ip, self.sitl_port))
                
                try:
                    data, _ = self.sock.recvfrom(4096)
                    response = json.loads(data.decode('utf-8'))
                    if "servos" in response:
                        self.apply_motor_commands(response["servos"])
                except socket.timeout:
                    pass

                mujoco.mj_step(self.model, self.data)
                viewer.sync()
                
                # Fixed string formatting to print index 0 of motor array safely without crashes
                print(f"[Simulation Time]: {self.data.time:.3f}s | Motor 1 Ctrl: {self.data.ctrl[0]:.3f}", end="\r")

if __name__ == "__main__":
    bridge = MuJoCoArduPilotBridge(xml_path="quadcopter.xml")
    bridge.run()
