"""Dubai base map for the dashboard.

On first use the map tiles around config.MAP_CENTER are downloaded (Esri satellite /
OpenStreetMap / Esri dark, see STYLES), stitched into one image and cached in config.MAP_CACHE_DIR, so later
runs work offline. Delete the cache folder to download again (e.g. after changing the area).
"""
import math
import os
import urllib.request
from concurrent.futures import ThreadPoolExecutor

import cv2
import numpy as np

import config as C

TILE = 256
USER_AGENT = "aruco-tracker-dashboard/1.0"
_ESRI = "https://server.arcgisonline.com/ArcGIS/rest/services/{}/MapServer/tile/{{z}}/{{y}}/{{x}}"
STYLES = {  # config.MAP_STYLE -> (tile URL, attribution)
    "satellite": (_ESRI.format("World_Imagery"), "Imagery (c) Esri, Maxar, Earthstar Geographics"),
    "streets": ("https://tile.openstreetmap.org/{z}/{x}/{y}.png", "(c) OpenStreetMap contributors"),
    "dark": (_ESRI.format("Canvas/World_Dark_Gray_Base"), "(c) Esri, HERE, Garmin, (c) OpenStreetMap contributors"),
}


def _style():
    return STYLES.get(C.MAP_STYLE, STYLES["satellite"])


def attribution():
    return _style()[1]

_image = None


def _tile_xy(lat, lon, z):
    """WGS84 -> fractional slippy-map tile coordinates."""
    n = 2 ** z
    x = (lon + 180.0) / 360.0 * n
    y = (1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n
    return x, y


def cache_path():
    lat, lon = C.MAP_CENTER
    w, h = C.MAP_IMAGE_SIZE
    style = C.MAP_STYLE.replace("/", "-")
    return os.path.join(C.MAP_CACHE_DIR, f"dubai_{style}_z{C.MAP_ZOOM}_{lat:.4f}_{lon:.4f}_{w}x{h}.png")


def _fetch(args):
    tx, ty = args
    url = _style()[0].format(z=C.MAP_ZOOM, x=tx, y=ty)
    req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
    with urllib.request.urlopen(req, timeout=15) as r:
        data = np.frombuffer(r.read(), np.uint8)
    tile = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if tile is None:
        raise ValueError(f"bad tile {url}")
    return tx, ty, tile


def download():
    """Download and stitch the map. Returns a BGR image of size MAP_IMAGE_SIZE."""
    lat, lon = C.MAP_CENTER
    w, h = C.MAP_IMAGE_SIZE
    cx, cy = _tile_xy(lat, lon, C.MAP_ZOOM)
    left, top = cx * TILE - w / 2, cy * TILE - h / 2
    tx0, ty0 = math.floor(left / TILE), math.floor(top / TILE)
    tx1, ty1 = math.floor((left + w - 1) / TILE), math.floor((top + h - 1) / TILE)
    jobs = [(tx, ty) for ty in range(ty0, ty1 + 1) for tx in range(tx0, tx1 + 1)]
    print(f"Downloading Dubai map ({len(jobs)} tiles)...")
    canvas = np.zeros(((ty1 - ty0 + 1) * TILE, (tx1 - tx0 + 1) * TILE, 3), np.uint8)
    with ThreadPoolExecutor(4) as pool:
        for tx, ty, tile in pool.map(_fetch, jobs):
            y, x = (ty - ty0) * TILE, (tx - tx0) * TILE
            canvas[y:y + TILE, x:x + TILE] = cv2.resize(tile, (TILE, TILE)) if tile.shape[:2] != (TILE, TILE) else tile
    ox, oy = int(round(left - tx0 * TILE)), int(round(top - ty0 * TILE))
    return canvas[oy:oy + h, ox:ox + w].copy()


def _placeholder():
    w, h = C.MAP_IMAGE_SIZE
    img = np.full((h, w, 3), (34, 26, 20), np.uint8)
    for x in range(0, w, 64):
        cv2.line(img, (x, 0), (x, h), (48, 40, 32), 1)
    for y in range(0, h, 64):
        cv2.line(img, (0, y), (w, y), (48, 40, 32), 1)
    for i, line in enumerate(("Dubai map could not be downloaded.",
                              "Check the internet connection and restart (it is cached after the first run).")):
        (tw, _), _ = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
        cv2.putText(img, line, ((w - tw) // 2, h // 2 - 60 + i * 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (120, 120, 120), 2, cv2.LINE_AA)
    return img


def load():
    """The cached map image (downloads it the first time; placeholder if offline)."""
    global _image
    if _image is not None:
        return _image
    path = cache_path()
    if os.path.exists(path):
        _image = cv2.imread(path)
    if _image is None:
        try:
            _image = download()
            os.makedirs(C.MAP_CACHE_DIR, exist_ok=True)
            cv2.imwrite(path, _image)
            print(f"Map cached to {path}")
        except Exception as e:  # offline, blocked, ...
            print(f"WARNING: could not download the Dubai map ({e}). Using a blank map.")
            _image = _placeholder()
    return _image


if __name__ == "__main__":
    img = load()
    print("map size:", img.shape[1], "x", img.shape[0], "->", cache_path())
