"""Write a printable ArUco marker PNG. Usage: python generate_marker.py [id]
Print it so the black square is MARKER_SIZE_CM wide (see config.py)."""
import sys

import cv2
import numpy as np

import config as C

marker_id = int(sys.argv[1]) if len(sys.argv) > 1 else (C.MARKER_ID or 0)
d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, C.ARUCO_DICT))
img = cv2.aruco.generateImageMarker(d, marker_id, 600)
img = np.pad(img, 100, constant_values=255)  # white quiet zone
name = f"marker_{marker_id}.png"
cv2.imwrite(name, img)
print(f"wrote {name} - print with the black square exactly {C.MARKER_SIZE_CM} cm wide")
