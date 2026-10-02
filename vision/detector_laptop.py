"""
Laptop camera detector: OpenCV color+shape detection + ArUco.
ArUco ID = quantity of the nearest detected shape.

Usage:
    python detector_laptop.py
    python detector_laptop.py --camera 1
    python detector_laptop.py --stream-url http://192.168.1.50:81/stream
"""

import warnings
warnings.filterwarnings("ignore")

import os
os.environ["OPENCV_LOG_LEVEL"] = "SILENT"
import argparse
import cv2
import numpy as np
import sys

# HSV color ranges
COLOR_RANGES = {
    "red":    [
        (np.array([0, 100, 80]),   np.array([10, 255, 255])),
        (np.array([160, 100, 80]), np.array([180, 255, 255])),
    ],
    "yellow": [(np.array([18, 100, 100]), np.array([38, 255, 255]))],
    "green":  [(np.array([35, 80, 80]),   np.array([85, 255, 255]))],
    "blue":   [(np.array([90, 100, 80]),  np.array([130, 255, 255]))],
    "black":  [(np.array([0, 0, 0]),      np.array([180, 60, 50]))],
}

COLOR_BGR = {
    "red": (0, 0, 255), "yellow": (0, 255, 255),
    "green": (0, 255, 0), "blue": (255, 0, 0), "black": (128, 128, 128),
}

MIN_AREA = 800


def detect_shapes(frame):
    """Detect colored cubes and cylinders using HSV color masking + contour analysis."""
    blurred = cv2.GaussianBlur(frame, (7, 7), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    detections = []
    frame_area = frame.shape[0] * frame.shape[1]
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    for color_name, ranges in COLOR_RANGES.items():
        mask = None
        for lower, upper in ranges:
            m = cv2.inRange(hsv, lower, upper)
            mask = m if mask is None else mask | m

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_AREA or area > frame_area * 0.4:
                continue

            peri = cv2.arcLength(cnt, True)
            if peri == 0:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            circularity = 4 * np.pi * area / (peri * peri)
            extent = area / (w * h)
            aspect = float(w) / h if h > 0 else 1
            vertices = len(cv2.approxPolyDP(cnt, 0.03 * peri, True))

            # Classify shape
            # Cylinders appear round/elliptical from the side
            # Cubes appear rectangular
            if vertices == 4 and extent > 0.5 and 0.3 < aspect < 3.0:
                shape = "cube"
            elif circularity > 0.65 and extent > 0.65:
                shape = "cylinder"
            else:
                continue

            cx, cy = x + w // 2, y + h // 2
            bgr = COLOR_BGR.get(color_name, (0, 255, 0))

            # Draw
            cv2.rectangle(frame, (x, y), (x + w, y + h), bgr, 2)
            label = f"{color_name} {shape}"
            cv2.putText(frame, label, (x, y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr, 2)
            cv2.circle(frame, (cx, cy), 4, bgr, -1)

            detections.append({
                "color": color_name, "shape": shape,
                "x": cx, "y": cy,
                "x1": x, "y1": y, "x2": x + w, "y2": y + h,
                "area": area,
            })

    return frame, detections


def detect_aruco(frame, seen_codes, detections=None):
    """Detect ArUco markers and pair with nearest shape."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_results = []

    aruco_dicts = [
        ("4X4_50", cv2.aruco.DICT_4X4_50),
        ("5X5_100", cv2.aruco.DICT_5X5_100),
        ("ORIGINAL", cv2.aruco.DICT_ARUCO_ORIGINAL),
    ]

    for dict_name, dict_id in aruco_dicts:
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                pts = corners[i][0].astype(int)
                cx = int(np.mean(pts[:, 0]))
                cy = int(np.mean(pts[:, 1]))

                # Pair with nearest shape
                paired_shape = "unknown"
                paired_color = "unknown"
                if detections:
                    best_dist = float("inf")
                    for d in detections:
                        dist = ((d["x"] - cx) ** 2 + (d["y"] - cy) ** 2) ** 0.5
                        if dist < best_dist:
                            best_dist = dist
                            paired_shape = d.get("shape", "unknown")
                            paired_color = d.get("color", "unknown")

                # Draw
                for j in range(4):
                    cv2.line(frame, tuple(pts[j]), tuple(pts[(j + 1) % 4]), (0, 255, 255), 2)

                label = f"{marker_id}x {paired_color} {paired_shape}"
                cv2.putText(frame, label, (pts[0][0], pts[0][1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                cv2.circle(frame, (cx, cy), 4, (0, 255, 255), -1)

                key = f"aruco:{marker_id}"
                if key not in seen_codes:
                    seen_codes.add(key)
                    print(f"[ArUco] ID={marker_id} => {marker_id}x {paired_color} {paired_shape}")
                    sys.stdout.flush()

                aruco_results.append({
                    "id": int(marker_id), "shape": paired_shape,
                    "color": paired_color, "x": cx, "y": cy
                })

    return frame, aruco_results


def main():
    parser = argparse.ArgumentParser(description="Laptop camera detector")
    source_group = parser.add_mutually_exclusive_group()
    source_group.add_argument("--camera", type=int, default=0,
                              help="Local camera index (default: 0)")
    source_group.add_argument("--stream-url",
                              help="HTTP video stream URL, for example an ESP32-CAM MJPEG endpoint")
    args = parser.parse_args()

    source = args.stream_url if args.stream_url else args.camera
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        print(f"ERROR: Cannot open video source {source}")
        return

    if args.stream_url is None:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    print(f"Video source {source} opened")
    print("Keys: 'q' quit")
    sys.stdout.flush()

    seen_codes = set()
    last_aruco_results = []

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        # Detect colored shapes
        frame, detections = detect_shapes(frame)

        # Detect ArUco and pair with shapes
        frame, new_aruco = detect_aruco(frame, seen_codes, detections)
        if new_aruco:
            last_aruco_results = new_aruco

        # Top bar
        total = sum(a["id"] for a in last_aruco_results)
        parts = [f"{a['id']}x {a['color']} {a['shape']}" for a in last_aruco_results]
        summary = " | ".join(parts) if parts else "No ArUco"
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 30), (0, 0, 0), -1)
        cv2.putText(frame, f"Total: {total}  [{summary}]", (5, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

        cv2.imshow("Detector", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

    if seen_codes:
        print(f"\nAll codes detected ({len(seen_codes)}):")
        for code in sorted(seen_codes):
            print(f"  {code}")


if __name__ == "__main__":
    main()
