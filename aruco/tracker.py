"""Camera access and ArUco marker detection."""
import math
from dataclasses import dataclass

import cv2
import numpy as np

import config as C


@dataclass
class Detection:
    found: bool = False
    center: tuple = None    # (x, y) px
    heading: float = None   # radians, direction of the marker's top edge
    side_px: float = None   # mean side length in px
    corners: np.ndarray = None  # 4x2 float


class MarkerTracker:
    def __init__(self):
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, C.ARUCO_DICT))
        self.detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
        self.cap = None

    def open(self):
        """Open the camera. Returns True on success."""
        cap = cv2.VideoCapture(C.CAMERA_INDEX, cv2.CAP_DSHOW)
        if not cap.isOpened():
            cap.release()
            return False
        if C.CAMERA_FOURCC:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*C.CAMERA_FOURCC))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, C.FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, C.FRAME_HEIGHT)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap = cap
        return True

    def read(self):
        """Return the next frame, or None if the grab failed."""
        if self.cap is None:
            return None
        ok, frame = self.cap.read()
        return frame if ok else None

    def release(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def detect(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return Detection()
        for c, i in zip(corners, ids.flatten()):
            if C.MARKER_ID is None or int(i) == C.MARKER_ID:
                pts = c.reshape(4, 2).astype(np.float64)
                sides = [np.linalg.norm(pts[k] - pts[(k + 1) % 4]) for k in range(4)]
                top = pts[1] - pts[0]
                return Detection(True, tuple(pts.mean(axis=0)), math.atan2(top[1], top[0]),
                                 float(np.mean(sides)), pts)
        return Detection()


def draw_detection(frame, det):
    if not det.found:
        return
    pts = det.corners.astype(np.int32).reshape(-1, 1, 2)
    cv2.polylines(frame, [pts], True, (0, 255, 255), 2, cv2.LINE_AA)
    cx, cy = det.center
    cv2.circle(frame, (int(cx), int(cy)), 4, (0, 0, 255), -1, cv2.LINE_AA)
    r = det.side_px * 0.8
    cv2.arrowedLine(frame, (int(cx), int(cy)),
                    (int(cx + r * math.cos(det.heading)), int(cy + r * math.sin(det.heading))),
                    (0, 255, 0), 2, cv2.LINE_AA, tipLength=0.3)
