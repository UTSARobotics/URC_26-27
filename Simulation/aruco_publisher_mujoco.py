from pathlib import Path

import cv2
import mujoco
import ros3 as rose

TOPIC = "aruco_detections"
CAMERA_NAME = "robot_camera"
SCENE_PATH = Path(__file__).resolve().parent / "go2_scene" / "scene.xml"
WIDTH, HEIGHT = 640, 480

# False: robot stays in its starting pose (static view).
# True: physics advances every frame (robot may collapse if nothing controls it).
STEP_PHYSICS = False

if not hasattr(cv2, "aruco"):
    raise RuntimeError(
        "OpenCV ArUco is unavailable. Install opencv-contrib-python "
        "in the active virtual environment."
    )

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

# MuJoCo setup (replaces cv2.VideoCapture)
model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
renderer = mujoco.Renderer(model, height=HEIGHT, width=WIDTH)

node = rose.Node("aruco_publisher")
publisher = node.publisher(TOPIC, message_size=1024, rate=30)

print(f"Publishing detections on '{TOPIC}'. Press q in the camera window to quit.")

try:
    while node.ok():
        if STEP_PHYSICS:
            mujoco.mj_step(model, data)
        else:
            mujoco.mj_forward(model, data)

        # Render the MuJoCo camera (replaces camera.read())
        renderer.update_scene(data, camera=CAMERA_NAME)
        rgb_frame = renderer.render()
        frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)

        corners, ids, _ = detector.detectMarkers(frame)
        detections = []

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            for marker_corners, marker_id in zip(corners, ids.flatten()):
                points = marker_corners.reshape(4, 2)
                center_x = int(points[:, 0].mean())
                center_y = int(points[:, 1].mean())

                detections.append([int(marker_id), center_x, center_y])
                cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)

        # Each detection is [marker_id, center_x, center_y].
        # An empty list means no markers were found in this frame.
        publisher.publish(detections)

        cv2.imshow("ArUco detection (MuJoCo)", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
finally:
    renderer.close()
    cv2.destroyAllWindows()




