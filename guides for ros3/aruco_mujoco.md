# MuJoCo ArUco Marker Setup Guide

This guide recaps how to add an ArUco marker and a simulated camera to your Go2 scene, then view the image that camera renders.

## Your folders and files

Your project is in `~/mujoco_code`. The relevant files are:

- `Unitree.py` — your Python script.
- `aruco_0.png` — the generated marker image.
- `mujoco_menagerie/` — your local Menagerie files.
- `go2_scene/scene.xml` — your **working copy** of the Go2 scene, where you added the marker and camera.

The installed `mujoco_menagerie` package originally loaded its scene from the venv cache. You copied it to `go2_scene` so you can edit it without changing the cached package files.

## 1. Add the marker texture and geometry

The marker image must be from the same ArUco dictionary as the detector—in this setup, `DICT_4X4_50`. The image `aruco_0.png` is in the same folder as `scene.xml`, so the XML can refer to it by filename.

In `go2_scene/scene.xml`, the marker’s texture and material belong inside `<asset>`:

```xml
<texture name="aruco_texture" type="2d" file="aruco_0.png"/>
<material name="aruco_material" texture="aruco_texture" texrepeat="1 1"/>
```

The marker geometry belongs inside `<worldbody>`:

```xml
<body name="aruco_target" pos="1 0 0.5" euler="0 -90 0">
  <geom name="aruco_marker"
        type="plane"
        size="0.15 0.15 0.01"
        material="aruco_material"
        contype="0"
        conaffinity="0"/>
</body>
```

This places the marker near the Go2. The orientation `euler="0 -90 0"` turns its visible side toward the dog. If you move the marker or camera later, you may need to adjust its position or orientation.

To edit the scene:

```bash
cd ~/mujoco_code
nano go2_scene/scene.xml
```

In Nano, save with **Ctrl+O**, press **Enter**, and exit with **Ctrl+X**.

## 2. Load your edited scene in `Unitree.py`

The original `mm.load("unitree_go2")` call loads the package’s cached scene. To use your edited copy, load its XML directly. `Unitree.py` should look like this:

```python
from pathlib import Path

import mujoco
import mujoco.viewer

scene_path = Path(__file__).resolve().parent / "go2_scene" / "scene.xml"

model = mujoco.MjModel.from_xml_path(str(scene_path))
data = mujoco.MjData(model)

print(model.nq)

mujoco.viewer.launch(model, data)
```

Run it from your project folder with your venv active:

```bash
cd ~/mujoco_code
python Unitree.py
```

The `19` printed in the terminal is `model.nq`, MuJoCo’s count of position coordinates in the model. It is **not** a marker ID or detection result. The viewer window should show the Go2 and marker.

## 3. Add a simulated camera

The viewer’s mouse-controlled camera is just for looking around the scene. It is not the camera your ArUco detector will use.

Add this camera inside `<worldbody>` in `go2_scene/scene.xml`, alongside the light, floor, and marker:

```xml
<camera name="aruco_cam" pos="0 0 0.5" euler="0 -90 0" fovy="60"/>
```

This is a fixed camera near the origin, pointing toward the marker at approximately `x=1`. It is stationary in the world, not mounted on the Go2.

## 4. Test the camera’s rendered image

Create a separate test script:

```bash
cd ~/mujoco_code
nano test_camera.py
```

Paste in:

```python
from pathlib import Path

import cv2
import mujoco

scene_path = Path(__file__).resolve().parent / "go2_scene" / "scene.xml"
model = mujoco.MjModel.from_xml_path(str(scene_path))
data = mujoco.MjData(model)

renderer = mujoco.Renderer(model, height=480, width=640)
renderer.update_scene(data, camera="aruco_cam")

rgb_frame = renderer.render()
bgr_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)

cv2.imshow("MuJoCo camera: aruco_cam", bgr_frame)
cv2.waitKey(0)

cv2.destroyAllWindows()
renderer.close()
```

Save it, then run:

```bash
python test_camera.py
```

A window should show the image rendered by `aruco_cam`. The RGB-to-BGR conversion is needed because OpenCV expects BGR frames. If the marker is visible in this window, that rendered image is ready to use as input to the ArUco detector.

## Current status

You have added the marker and camera, and pointed `Unitree.py` at your edited scene. The remaining step is to connect the rendered camera frame to the guide’s existing ArUco detection and ROS3 publishing code.
