"""Render one dashboard image per city theme into theme_previews/ (no camera needed).

    python preview_themes.py
"""
import os
from collections import deque

import cv2
import numpy as np

import config as C
import city_map
import dashboard
from kinematics import Kinematics
from track import Track

OUT = "theme_previews"


def sample_state():
    """A short simulated drive: accelerate along the route, then turn and brake."""
    track = Track.load() or Track([(200, 150), (1000, 150), (1000, 550), (200, 550)], closed=True)
    kin = Kinematics()
    trail = deque(maxlen=C.TRAIL_MAX)
    pts = track.points
    path = []
    for a, b in zip(pts[:2], pts[1:3]):  # drive the first two legs of the route
        n = 40
        path += [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n) for i in range(n)]
    info = None
    for i, (x, y) in enumerate(path):
        kin.update(i / 30, x, y, 80.0)
        trail.append((x, y))
        info = track.project((x, y), kin.scale)
    return kin, track, info, trail


def main():
    os.makedirs(OUT, exist_ok=True)
    kin, track, info, trail = sample_state()
    tiles = []
    for name in city_map.THEMES:
        C.MAP_THEME = name
        img = dashboard.render(kin, track, info, trail, (1280, 720), False, (1920, 1080))
        path = os.path.join(OUT, f"theme_{name}.png")
        cv2.imwrite(path, img)
        print("wrote", path)
        t = cv2.resize(img, (960, 540), interpolation=cv2.INTER_AREA)
        cv2.rectangle(t, (0, 470), (360, 540), (0, 0, 0), -1)
        cv2.putText(t, f"{len(tiles) + 1}. {name.upper()}", (16, 518), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
                    (255, 255, 255), 2, cv2.LINE_AA)
        tiles.append(t)
    while len(tiles) % 2:
        tiles.append(np.zeros_like(tiles[0]))
    sheet = np.vstack([np.hstack(tiles[i:i + 2]) for i in range(0, len(tiles), 2)])
    cv2.imwrite(os.path.join(OUT, "all_themes.png"), sheet)
    print("wrote", os.path.join(OUT, "all_themes.png"))


if __name__ == "__main__":
    main()
