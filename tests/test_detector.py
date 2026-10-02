import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "vision"))
from detector_laptop import detect_aruco, detect_shapes  # noqa: E402


class DetectorTests(unittest.TestCase):
    def test_colored_square_and_circle(self):
        frame = np.full((480, 640, 3), 255, dtype=np.uint8)
        cv2.rectangle(frame, (60, 100), (180, 220), (255, 0, 0), -1)
        cv2.circle(frame, (400, 160), 60, (0, 255, 255), -1)

        _, detections = detect_shapes(frame)
        labels = {(item["color"], item["shape"]) for item in detections}
        self.assertIn(("blue", "cube"), labels)
        self.assertIn(("yellow", "cylinder"), labels)

    def test_aruco_marker_id(self):
        frame = np.full((400, 400, 3), 255, dtype=np.uint8)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        marker = cv2.aruco.generateImageMarker(dictionary, 3, 150)
        frame[100:250, 100:250] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)

        seen = set()
        _, results = detect_aruco(frame, seen)
        self.assertTrue(any(item["id"] == 3 for item in results))
        self.assertIn("aruco:3", seen)


if __name__ == "__main__":
    unittest.main()
