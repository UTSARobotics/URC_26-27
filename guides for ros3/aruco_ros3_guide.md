Here’s a first end-to-end test. The webcam publisher will detect ArUco markers and publish each detection as `[marker_id, center_x, center_y]`. The subscriber will print what it receives.

## 1. Activate your environment and install OpenCV

In a terminal:

```sh
source ~/ros3-demo/setup-ros3.sh
python -m pip install opencv-contrib-python
```

Check that OpenCV and its ArUco module are available:

```sh
python -c "import cv2; print(cv2.__version__); print('ArUco available:', hasattr(cv2, 'aruco'))"
```

You should see `ArUco available: True`.

## 2. Create the webcam publisher

Save this as `~/ros3-demo/aruco_pub.py`:

```python
import cv2
import ros3 as rose

TOPIC = "aruco_detections"
CAMERA_INDEX = 0

if not hasattr(cv2, "aruco"):
    raise RuntimeError(
        "OpenCV ArUco is unavailable. Install opencv-contrib-python "
        "in the active virtual environment."
    )

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

camera = cv2.VideoCapture(CAMERA_INDEX)
if not camera.isOpened():
    raise RuntimeError(
        f"Could not open camera {CAMERA_INDEX}. Try CAMERA_INDEX = 1, "
        "or check that the camera is connected and accessible."
    )

node = rose.Node("aruco_publisher")
publisher = node.publisher(TOPIC, message_size=1024, rate=30)

print(f"Publishing detections on '{TOPIC}'. Press q in the camera window to quit.")

try:
    while node.ok():
        ok, frame = camera.read()
        if not ok:
            print("Could not read a frame from the camera.")
            break

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

        cv2.imshow("ArUco detection", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
finally:
    camera.release()
    cv2.destroyAllWindows()
```

The detector uses the `DICT_4X4_50` dictionary. The printed marker must use the same dictionary to be detected. The center coordinates are in pixels, not real-world units.

## 3. Create the subscriber

Save this as `~/ros3-demo/aruco_sub.py`:

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

## 4. Run the test

Open two terminals. In **both**, run:

```sh
source ~/ros3-demo/setup-ros3.sh
cd ~/ros3-demo
```

Start the publisher **first** in terminal 1:

```sh
python aruco_pub.py
```

A camera window should appear. Hold a printed `DICT_4X4_50` marker in view.

Then start the subscriber in terminal 2:

```sh
python aruco_sub.py
```

The subscriber should print messages like:

```text
Detections: [[7, 318, 241]]
```

The exact ID and coordinates will depend on the marker and where it appears in the camera image. With no marker in view, it should print an empty list. The publisher sends messages but doesn’t print every detection; watch the subscriber terminal for those.

Press **q** while the camera window is active to stop the publisher, and press **Ctrl+C** to stop the subscriber.

## If something goes wrong

- **Topic does not exist:** Start the publisher before the subscriber. Check that both scripts use the same topic name, `aruco_detections`.
- **Camera won’t open:** Try setting `CAMERA_INDEX = 1`. Also check that another app isn’t using the webcam.
- **No marker is detected:** Make sure the marker uses `DICT_4X4_50`, is clearly visible, and has good lighting.
- **OpenCV has no `aruco`:** Confirm the virtual environment is active, then install `opencv-contrib-python` there.
- **Messages are too large:** These detections contain only IDs and centers, so they should be small. Don’t publish the full camera image for this first test.

When you test it, note which step fails and paste the full error or terminal output.



## Choosing the camera

The detection and ROS3 code works the same with a laptop’s built-in camera or an external webcam. The camera index may differ when both are connected.

In `aruco_pub.py`, find:

```python
CAMERA_INDEX = 0
```

Try `0`, then `1` if the wrong camera opens. Run the publisher after each change and check the camera window to identify the feed. Keep the index that selects your intended camera.

If the correct camera opens but marker detection is less reliable than with the external webcam, the cameras may differ in resolution, focus, or exposure. Test the camera at its defaults first; you can tune those settings if needed.
