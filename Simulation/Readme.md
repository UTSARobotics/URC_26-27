# Aruco Detector in Mujoco

This guide shows how to detect an aruco marker in mujoco. The `mujoco_aruco` guide under "guides for ros3" should be followed before this.

## Folder Setup
Potential folder layout in `~/ros3_ws/src/mujoco_projects`:
- `aruco_0.png`
- `aruco_publisher_mujoco.py` --this replaces test_camera.py
-  `Unitree.py` --from aruco_mujoco guide
- `go2_scene` --from aruco_mujoco guide
- `mujoco_menagerie` --from aruco_mujoco guide

## Add publisher script

Copy and paste this into `aruco_publisher_mujoco.py`:
```python
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
```

## Printing Detections

The same `aruco_sub.py` from ros3 installation can be used to print detections:
```python
import ros3 as rose

TOPIC = "aruco_detections"

node = rose.Node("aruco_subscriber")
subscriber = node.subscriber(TOPIC)

print(f"Listening on '{TOPIC}'...")

while node.ok():
    for message in subscriber:
        print("Detections:", message)
```

## Running everything

Open a terminal and activate your virtual environment, then inside of your `mujoco_projects` folder, run:
```python
python aruco_publisher_mujoco.py
```

A window should pop up showing the detected aruco marker in mujoco from the go2 head POV.

Open another terminal and go to where the aruco subscriper file is, and run:
```python
python aruco_sub.py
```
Detections should now be printing in that terminal.


## Potential Errors

If you get the "no module named 'ros3'" error, then ros3 can't be found inside of your virtual environment. What fixed this for me is changing the `include-system-site-packages = false` inside of the `my_venv/pyvenv.cfg` to equal true.

