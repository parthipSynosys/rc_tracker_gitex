"""Track polyline: load/save, projection of a position onto it, and drawing helpers."""
import json
import math
import os

import cv2
import numpy as np

import config as C


class Track:
    def __init__(self, points=None, closed=False, frame_size=None):
        self.points = [tuple(map(float, p)) for p in (points or [])]
        self.closed = bool(closed)
        self.frame_size = tuple(frame_size) if frame_size else None

    # ------------------------------------------------------------ file I/O
    @classmethod
    def load(cls, path=None):
        path = path or C.TRACK_FILE
        if not os.path.exists(path):
            return None
        with open(path, "r", encoding="utf-8") as f:
            d = json.load(f)
        return cls(d.get("points", []), d.get("closed", False), d.get("frame_size"))

    def save(self, path=None):
        path = path or C.TRACK_FILE
        data = {
            "points": [[round(x, 1), round(y, 1)] for x, y in self.points],
            "closed": self.closed,
            "frame_size": list(self.frame_size) if self.frame_size else None,
        }
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)

    # ------------------------------------------------------------ editing
    def add(self, x, y):
        self.points.append((float(x), float(y)))

    def undo(self):
        if self.points:
            self.points.pop()

    def clear(self):
        self.points.clear()

    # ------------------------------------------------------------ geometry
    def segments(self):
        pts = self.points
        segs = list(zip(pts[:-1], pts[1:]))
        if self.closed and len(pts) >= 3:
            segs.append((pts[-1], pts[0]))
        return segs

    @property
    def length_px(self):
        return sum(math.dist(p, q) for p, q in self.segments())

    def project(self, pt, cm_per_px):
        """Project pt (px) onto the track.

        Returns dict(nearest_px, dist_cm, progress_pct, on_track) or None if the
        track has no points.
        """
        if not self.points:
            return None
        px, py = pt
        segs = self.segments()
        if not segs:  # single point
            nx, ny = self.points[0]
            d = math.hypot(px - nx, py - ny)
            best = ((nx, ny), d, 0.0)
        else:
            total = self.length_px
            best = None
            cum = 0.0
            for (ax, ay), (bx, by) in segs:
                dx, dy = bx - ax, by - ay
                seg_len2 = dx * dx + dy * dy
                u = 0.0 if seg_len2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg_len2))
                cx, cy = ax + u * dx, ay + u * dy
                d = math.hypot(px - cx, py - cy)
                seg_len = math.sqrt(seg_len2)
                if best is None or d < best[1]:
                    best = ((cx, cy), d, cum + u * seg_len)
                cum += seg_len
            best = (best[0], best[1], 0.0 if total == 0 else best[2] / total * 100.0)
        dist_cm = best[1] * cm_per_px
        return {
            "nearest_px": best[0],
            "dist_cm": dist_cm,
            "progress_pct": best[2],
            "on_track": dist_cm <= C.OFF_TRACK_CM,
        }


# ---------------------------------------------------------------- drawing
def draw_track(img, track, corridor_px=None, show_ends=True):
    """Draw the track on a camera frame: optional corridor, line, arrows, start/finish."""
    if track is None or not track.points:
        return
    pts = np.array(track.points, dtype=np.int32).reshape(-1, 1, 2)
    closed = track.closed and len(track.points) >= 3

    if corridor_px and corridor_px >= 1:
        overlay = img.copy()
        r = int(round(corridor_px))
        cv2.polylines(overlay, [pts], closed, C.CORRIDOR_COLOR, 2 * r, cv2.LINE_AA)
        for p in pts.reshape(-1, 2):
            cv2.circle(overlay, tuple(int(v) for v in p), r, C.CORRIDOR_COLOR, -1, cv2.LINE_AA)
        img[:] = cv2.addWeighted(overlay, 0.25, img, 0.75, 0)

    if len(track.points) >= 2:
        cv2.polylines(img, [pts], closed, C.TRACK_COLOR, 2, cv2.LINE_AA)
    for (ax, ay), (bx, by) in track.segments():
        seg = math.hypot(bx - ax, by - ay)
        if seg < 40:
            continue
        ux, uy = (bx - ax) / seg, (by - ay) / seg
        mx, my = (ax + bx) / 2, (ay + by) / 2
        cv2.arrowedLine(img, (int(mx - ux * 10), int(my - uy * 10)), (int(mx + ux * 10), int(my + uy * 10)),
                        C.TRACK_COLOR, 2, cv2.LINE_AA, tipLength=0.7)
    for p in track.points:
        cv2.circle(img, (int(p[0]), int(p[1])), 3, C.TRACK_COLOR, -1, cv2.LINE_AA)
    if show_ends:
        sx, sy = track.points[0]
        cv2.circle(img, (int(sx), int(sy)), 9, C.START_COLOR, -1, cv2.LINE_AA)
        cv2.putText(img, "START", (int(sx) + 12, int(sy) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, C.START_COLOR, 1, cv2.LINE_AA)
        if not track.closed and len(track.points) >= 2:
            fx, fy = track.points[-1]
            cv2.circle(img, (int(fx), int(fy)), 9, C.FINISH_COLOR, -1, cv2.LINE_AA)
            cv2.putText(img, "FINISH", (int(fx) + 12, int(fy) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, C.FINISH_COLOR, 1, cv2.LINE_AA)
