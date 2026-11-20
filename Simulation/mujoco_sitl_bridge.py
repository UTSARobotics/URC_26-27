import socket
import json
import time
import struct

import mujoco
import mujoco.viewer
import numpy as np
from scipy.spatial.transform import Rotation


LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 9002



def apply_pwm_packet(packet, data, model):
    """Decode an ArduPilot PWM packet and apply its first four channels."""
    if len(packet) < 8:
        return

    magic = struct.unpack_from("<H", packet, 0)[0]

    if magic == 18458 and len(packet) >= 40:
        channel_count = 16
    elif magic == 29569 and len(packet) >= 72:
        channel_count = 32
    else:
        return

    pwms = struct.unpack_from(f"<{channel_count}H", packet, 8)

    for i in range(min(4, model.nu, len(pwms))):
        thrust = (pwms[i] - 1000) / 1000.0
        data.ctrl[i] = max(0.0, min(1.0, thrust))


def main():
    model = mujoco.MjModel.from_xml_path("quadcopter.xml")
    data = mujoco.MjData(model)

    # These names must match the body and sensor names in quadcopter.xml.
    body_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "quadcopter"
    )
    gyro_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_SENSOR, "gyro"
    )
    accel_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_SENSOR, "accelerometer"
    )

    for name, obj_id in (
        ("body 'quadcopter'", body_id),
        ("sensor 'gyro'", gyro_id),
        ("sensor 'accelerometer'", accel_id),
    ):
        if obj_id < 0:
            raise ValueError(f"Could not find {name} in quadcopter.xml")

    # Assumptions: MuJoCo world is ENU (X east, Y north, Z up),
    # and the vehicle body is FLU (X forward, Y left, Z up).
    # Convert world ENU -> ArduPilot world NED.
    ENU_TO_NED = np.array([
        [0.0, 1.0,  0.0],
        [1.0, 0.0,  0.0],
        [0.0, 0.0, -1.0],
    ])

    # Convert body FLU -> ArduPilot body FRD.
    FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])

    # ArduPilot sends its output packets to port 9002.
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((LISTEN_IP, LISTEN_PORT))
    sock.setblocking(False)

    print(f"MuJoCo bridge listening on UDP {LISTEN_PORT}")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        last_sync = time.time()
        last_report = 0.0 #why do I do this?

        while viewer.is_running():
            step_start = time.time()
            reply_addr = None

            # Receive PWM from ArduPilot. Remember its address so we can
            # return the sensor JSON to that exact address.
            try:
                packet, reply_addr = sock.recvfrom(2048)
                apply_pwm_packet(packet, data, model)
            except BlockingIOError:
                pass

            mujoco.mj_step(model, data)
            now = time.monotonic() #this is new
            if now - last_report >= 1.0:
                print(
                    f"ctrl={data.ctrl[:4].copy()} "
                    f"pos={data.qpos[:3].copy()} "
                    f"vel={data.qvel[:3].copy()}"
                )
                last_report = now #this is where new ends


            # Position: MuJoCo world ENU -> ArduPilot world NED.
            pos_ned = ENU_TO_NED @ data.xpos[body_id]

            # For a free joint, the first three qvel values are
            # translational velocity in the world frame.
            vel_ned = ENU_TO_NED @ data.qvel[:3]

            # Read the model's body-frame IMU sensors, then convert
            # body FLU -> ArduPilot body FRD.
            gyro_start = model.sensor_adr[gyro_id]
            gyro_flu = data.sensordata[gyro_start:gyro_start + 3]
            gyro_frd = FLU_TO_FRD @ gyro_flu

            accel_start = model.sensor_adr[accel_id]
            accel_flu = data.sensordata[accel_start:accel_start + 3]
            accel_frd = FLU_TO_FRD @ accel_flu

            # Convert orientation to FRD-body -> NED-world.
            R_world_body = data.xmat[body_id].reshape(3, 3)
            R_ned_frd = ENU_TO_NED @ R_world_body @ FLU_TO_FRD
            q_xyzw = Rotation.from_matrix(R_ned_frd).as_quat()
            q_wxyz = [q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]]

            sensor_packet = {
                "timestamp": float(data.time),
                "imu": {
                    "gyro": [float(v) for v in gyro_frd],
                    "accel_body": [float(v) for v in accel_frd],
                },
                "position": [float(v) for v in pos_ned],
                "velocity": [float(v) for v in vel_ned],
                "quaternion": [float(v) for v in q_wxyz],
            }

            # Only reply after receiving a packet from SITL.
            if reply_addr is not None:
                message = json.dumps(sensor_packet).encode("utf-8") + b"\n"
                sock.sendto(message, reply_addr)

            now = time.time()
            if now - last_sync >= 1.0 / 60.0:
                viewer.sync()
                last_sync = now

            elapsed = time.time() - step_start
            if elapsed < model.opt.timestep:
                time.sleep(model.opt.timestep - elapsed)


if __name__ == "__main__":
    main()

