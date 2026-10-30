import numpy as np
import cv2
import pyrealsense2 as rs
import ros3 as rose
import time

TOPIC = "aruco_detections"
MARKER_SIZE = 0.2  # meters

# RealSense Camera Setup

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(
    rs.stream.color,
    1920,
    1080,
    rs.format.bgr8,
    30
)

profile = pipeline.start(config)

# Camera Intrinsics
color_profile = profile.get_stream(
    rs.stream.color
).as_video_stream_profile()

intrinsics = color_profile.get_intrinsics()

camera_matrix = np.array([
    [intrinsics.fx, 0, intrinsics.ppx],
    [0, intrinsics.fy, intrinsics.ppy],
    [0, 0, 1]
], dtype=np.float64)

dist_coeffs = np.array(intrinsics.coeffs,dtype=np.float64)

print("Camera Matrix:")
print(camera_matrix)

# ArUco Setup
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()

parameters.adaptiveThreshWinSizeMin = 3
parameters.adaptiveThreshWinSizeMax = 53
parameters.adaptiveThreshWinSizeStep = 5
parameters.minMarkerPerimeterRate = 0.01

parameters.cornerRefinementMethod = (cv2.aruco.CORNER_REFINE_SUBPIX)

detector = cv2.aruco.ArucoDetector(dictionary,parameters)

# Marker 3D Coordinates
half = MARKER_SIZE / 2

object_points = np.array([
    [-half,  half, 0],
    [ half,  half, 0],
    [ half, -half, 0],
    [-half, -half, 0]
], dtype=np.float32)

# ROS3 Setup
node = rose.Node("aruco_publisher")

publisher = node.publisher(
    TOPIC,
    message_size=1024,
    rate=30
)

# CLAHE Setup
clahe_modes = {
    2: cv2.createCLAHE(
        clipLimit=1.0,
        tileGridSize=(8, 8)
    ),
    3: cv2.createCLAHE(
        clipLimit=2.0,
        tileGridSize=(8, 8)
    ),
    4: cv2.createCLAHE(
        clipLimit=3.0,
        tileGridSize=(8, 8)
    ),
    5: cv2.createCLAHE(
        clipLimit=4.0,
        tileGridSize=(8, 8)
    )
}

mode = 0

print("CV-01 RealSense ArUco Detection")
print("0 = Original")
print("1 = Grayscale")
print("2-5 = CLAHE Contrast Increase")
print("q = Quit")

# Main Loop
try:
    while node.ok():

        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        frame = np.asanyarray(
            color_frame.get_data()
        )

        gray = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2GRAY
        )

        # Image processing modes
        if mode == 0:
            processed = frame
        elif mode == 1:
            processed = gray
        else:
            processed = clahe_modes[mode].apply(gray)

        # Detection Timer
        start_time = time.perf_counter()

        corners, ids, _ = detector.detectMarkers(
            processed
        )

        detection_ms = (
            time.perf_counter() - start_time
        ) * 1000

        detections = []

        # Marker Detection
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(
                frame,
                corners,
                ids
            )

            for marker_corners, marker_id in zip(corners,ids.flatten()):
                points = marker_corners.reshape(4, 2)

                success, rvec, tvec = cv2.solvePnP(
                    object_points,
                    points.astype(np.float32),
                    camera_matrix,
                    dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )

                if success:
                    x, y, z = tvec.flatten()
                    detections.append([
                        int(marker_id),
                        float(x),
                        float(y),
                        float(z)
                    ])

                    center_x = int(points[:, 0].mean())
                    center_y = int(points[:, 1].mean())
                    cv2.circle(
                        frame,
                        (center_x, center_y),
                        5,
                        (0, 255, 0),
                        -1
                    )

                    cv2.putText(
                        frame,
                        f"ID {marker_id}: {z:.2f}m",
                        (center_x, center_y - 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0),
                        2
                    )

        # ROS3 Publishing
        publisher.publish(detections)

        # Display Information
        cv2.putText(
            frame,
            f"Mode: {mode} | Markers: {len(detections)}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )

        cv2.putText(
            frame,
            f"Detection: {detection_ms:.2f} ms",
            (20, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )

        cv2.imshow(
            "CV-01 RealSense D435",
            frame
        )

        cv2.imshow(
            "Processed Image",
            processed
        )

        # Keyboard Controls
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

        elif key in [
            ord("0"),
            ord("1"),
            ord("2"),
            ord("3"),
            ord("4"),
            ord("5")
        ]:
            mode = int(chr(key))

            print(
                f"Switched to mode {mode}"
            )

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
