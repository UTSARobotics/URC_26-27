import socket
import json
import time
import struct
import mujoco
import mujoco.viewer

TARGET_IP = "127.0.0.1"
ARDUPILOT_PORT = 52471  # Target SITL port

def main():
    # Load MuJoCo Model
    model = mujoco.MjModel.from_xml_path('quadcopter.xml')
    data = mujoco.MjData(model)

    # Configure UDP client socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((TARGET_IP, 0)) # Dynamic client port
    sock.setblocking(False)

    sitl_address = (TARGET_IP, ARDUPILOT_PORT)
    print(f"MuJoCo Bridge active. Targeting ArduPilot JSON Server at UDP:{ARDUPILOT_PORT}")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        last_sync = time.time()
        
        while viewer.is_running():
            step_start = time.time()

            # Receive PWM Commands from ArduPilot SITL
            try:
                packet, addr = sock.recvfrom(2048)
                if len(packet) >= 2:
                    magic = struct.unpack('<H', packet[0:2])[0] # Safely get int from tuple
                    if (magic == 18458 or magic == 29569) and len(packet) >= 40:
                        pwms = struct.unpack('<16H', packet[8:40])
                        for i in range(min(4, len(pwms))):
                            data.ctrl[i] = max(0.0, (pwms[i] - 1000) / 1000.0)
                    else:
                        try:
                            ap_data = json.loads(packet.decode('utf-8'))
                            if 'servos' in ap_data:
                                pwms = ap_data['servos']
                                for i in range(min(4, len(pwms))):
                                    data.ctrl[i] = max(0.0, (pwms[i] - 1000) / 1000.0)
                        except UnicodeDecodeError:
                            pass
            except BlockingIOError:
                pass

            mujoco.mj_step(model, data)

            # Sensor data for ArduPilot SITL
            q = data.qpos[3:7].tolist()       # [w, x, y, z]
            ang_vel = data.qvel[3:6].tolist() # Rad/s
            lin_acc = data.qacc[0:3].tolist() # m/s^2
            pos = data.qpos[0:3].tolist()     # Position
            vel = data.qvel[0:3].tolist()     # Velocity

            sensor_packet = {
                "timestamp": float(data.time),
                "imu": {
                    "gyro":  {"x": float(ang_vel[0]), "y": float(ang_vel[1]), "z": float(ang_vel[2])},
                    "accel": {"x": float(lin_acc[0]), "y": float(lin_acc[1]), "z": float(lin_acc[2] + 9.81)}
                },
                "spatial": {
                    "q": {"w": float(q[0]), "x": float(q[1]), "y": float(q[2]), "z": float(q[3])},
                    "pos": {"x": float(pos[0]), "y": float(pos[1]), "z": float(pos[2])},
                    "velocities": {"x": float(vel[0]), "y": float(vel[1]), "z": float(vel[2])}
                }
            }

            tx_data = json.dumps(sensor_packet).encode('utf-8')
            sock.sendto(tx_data, sitl_address)

            now = time.time()
            if now - last_sync > 1.0/60.0:
                viewer.sync()
                last_sync = now

            elapsed = time.time() - step_start
            if elapsed < model.opt.timestep:
                time.sleep(model.opt.timestep - elapsed)

if __name__ == "__main__":
    main()
