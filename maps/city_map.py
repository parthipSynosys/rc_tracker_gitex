"""Animated, procedurally generated 'Dubai-style' city used as the dashboard background.

Not a real map: the city is built around the route (the track becomes a highway and
buildings keep clear of it), and the car's trail is paved as a road while it drives.
Static parts are drawn once per (screen size, route, theme) and cached; animate() adds the
moving bits each frame (water shimmer, twinkling windows, ambient traffic, tower beacon).
The look comes from THEMES, picked with config.MAP_THEME.
"""
import math

import cv2
import numpy as np

import config as C

AA = cv2.LINE_AA

# Colour themes (BGR). config.MAP_THEME picks one.
THEMES = {
    "night": dict(  # dark city at night, lit windows
        ground=(40, 34, 30), noise=True, street=(58, 54, 52), avenue=(72, 68, 64),
        sea=(92, 58, 22), sea_light=(150, 110, 50), beach=(70, 120, 150), park=(45, 85, 45), tree=(40, 110, 50),
        asphalt=(26, 26, 28), edge=(190, 190, 190), lane=(60, 200, 240), trail=(40, 190, 255),
        buildings=[(78, 62, 58), (92, 74, 66), (70, 60, 72), (96, 84, 80), (84, 70, 90), (64, 58, 54)],
        outline=None, shadow=(20, 18, 16), windows=[(120, 210, 255), (200, 230, 255), (255, 220, 160)],
        burj=[(150, 150, 160), (185, 185, 195), (215, 215, 225), (245, 245, 250)], lake=(110, 80, 40),
        plaza=(60, 90, 60), label=(220, 220, 230), glow=1.0, traffic=True),
    "day": dict(  # sunny desert city, white towers, turquoise sea
        ground=(170, 200, 222), noise=True, street=(225, 232, 238), avenue=(245, 248, 250),
        sea=(200, 175, 40), sea_light=(250, 235, 170), beach=(160, 220, 245), park=(110, 180, 110), tree=(60, 140, 70),
        asphalt=(80, 80, 86), edge=(250, 250, 250), lane=(40, 200, 250), trail=(30, 120, 255),
        buildings=[(236, 236, 240), (218, 222, 230), (240, 232, 222), (205, 212, 222), (226, 214, 204), (250, 246, 240)],
        outline=None, shadow=(120, 145, 165), windows=None,
        burj=[(170, 170, 180), (205, 205, 215), (230, 230, 240), (255, 255, 255)], lake=(220, 190, 70),
        plaza=(150, 200, 160), label=(40, 40, 50), glow=0.0, traffic=True),
    "neon": dict(  # cyberpunk: black city with glowing neon outlines
        ground=(14, 8, 16), noise=False, street=(45, 18, 45), avenue=(90, 30, 100),
        sea=(40, 14, 8), sea_light=(255, 210, 0), beach=(200, 60, 255), park=(20, 55, 20), tree=(60, 255, 120),
        asphalt=(16, 10, 20), edge=(255, 0, 255), lane=(255, 255, 0), trail=(255, 255, 0),
        buildings=[(26, 16, 30), (30, 18, 26), (22, 16, 34)],
        outline=[(255, 0, 255), (255, 255, 0), (0, 160, 255), (200, 0, 255)], shadow=None,
        windows=[(255, 0, 255), (255, 255, 0), (0, 200, 255)],
        burj=[(120, 0, 160), (200, 0, 255), (255, 120, 255), (255, 255, 255)], lake=(80, 30, 10),
        plaza=(40, 10, 40), label=(255, 255, 0), glow=1.6, traffic=True),
    "blueprint": dict(  # technical drawing: white lines on blueprint blue
        ground=(105, 55, 20), noise=False, street=(150, 95, 50), avenue=(185, 135, 85),
        sea=(80, 38, 12), sea_light=(230, 190, 130), beach=(235, 210, 170), park=(120, 72, 32), tree=(220, 190, 150),
        asphalt=(70, 34, 10), edge=(250, 240, 225), lane=(250, 240, 225), trail=(80, 220, 255),
        buildings=[(105, 55, 20)], outline=[(240, 225, 205)], shadow=None, windows=None,
        burj=[(140, 90, 40), (190, 150, 100), (230, 210, 180), (255, 255, 255)], lake=(80, 38, 12),
        plaza=(120, 70, 30), label=(250, 240, 225), glow=0.0, traffic=False, grid=(118, 66, 28)),
    "minimal": dict(  # clean light navigation-app look, blue route
        ground=(240, 241, 243), noise=False, street=(255, 255, 255), avenue=(140, 222, 252),
        sea=(250, 212, 160), sea_light=(255, 236, 205), beach=(224, 238, 246), park=(196, 234, 204), tree=(150, 210, 165),
        asphalt=(235, 140, 50), edge=(200, 100, 30), lane=None, trail=(235, 140, 50),
        buildings=[(226, 226, 230), (220, 222, 228), (230, 228, 226)], outline=[(205, 205, 210)],
        shadow=None, windows=None,
        burj=[(180, 180, 190), (200, 200, 210), (220, 220, 230), (240, 240, 248)], lake=(250, 212, 160),
        plaza=(196, 234, 204), label=(90, 90, 100), glow=0.0, traffic=False),
}

_cache = {}


def theme():
    return THEMES.get(getattr(C, "MAP_THEME", "night"), THEMES["night"])


class City:
    pass


def _dashed(img, pts, closed, color, thick, dash, gap):
    """Dashed polyline; the dash pattern continues across corners."""
    seq = list(pts) + ([pts[0]] if closed else [])
    period = max(2.0, dash + gap)
    walked = 0.0  # distance along the whole polyline
    for a, b in zip(seq[:-1], seq[1:]):
        d = math.dist(a, b)
        if d < 1e-6:
            continue
        ux, uy = (b[0] - a[0]) / d, (b[1] - a[1]) / d
        k = math.floor(walked / period)  # first dash that may touch this segment
        while True:
            s0, s1 = k * period - walked, k * period + dash - walked  # dash k in segment coords
            if s0 >= d:
                break
            s0, s1 = max(0.0, s0), min(d, s1)
            if s1 > s0:
                cv2.line(img, (int(a[0] + ux * s0), int(a[1] + uy * s0)),
                         (int(a[0] + ux * s1), int(a[1] + uy * s1)), color, thick, AA)
            k += 1
        walked += d


def draw_road(img, pts, closed, width, center=True):
    """Highway look: edges, asphalt, dashed centre line (colours from the theme)."""
    T = theme()
    a = np.array(pts, np.int32).reshape(-1, 1, 2)
    cv2.polylines(img, [a], closed, T["edge"], width + 6, AA)
    cv2.polylines(img, [a], closed, T["asphalt"], width, AA)
    for p in a.reshape(-1, 2):  # round joints
        cv2.circle(img, tuple(int(v) for v in p), width // 2, T["asphalt"], -1, AA)
    if center and T["lane"] is not None and len(pts) >= 2:
        _dashed(img, pts, closed, T["lane"], max(1, width // 12), width * 0.6, width * 0.5)


def _sea_and_creek(W, H, ui):
    water = np.zeros((H, W), np.uint8)
    # coastline across the bottom-left corner
    n = 40
    coast = []
    for i in range(n + 1):
        f = i / n
        x, y = f * W * 0.42, H * 0.66 + f * H * 0.34
        off = (math.sin(f * 9) * 14 + math.sin(f * 23) * 6) * ui
        coast.append((x + off, y - off))
    sea_poly = np.array([(0, H * 0.66)] + coast + [(0, H)], np.int32)
    cv2.fillPoly(water, [sea_poly], 255)
    # creek winding from the top edge to the right edge
    creek = [(W * 0.66 + math.sin(f * 6) * 30 * ui + f * W * 0.36, f * f * H * 0.55 + math.sin(f * 9) * 18 * ui)
             for f in (i / 59 for i in range(60))]
    cv2.polylines(water, [np.array(creek, np.int32)], False, 255, int(46 * ui), AA)
    # palm island in the sea: trunk from the shore, fronds on both sides, crescent around it
    palm = np.zeros_like(water)
    R = 95 * ui
    c = np.array([W * 0.10, H * 0.90])
    d = np.array([-1.0, 1.0]) / math.sqrt(2)  # from the shore out to sea
    base = c - d * R * 0.95
    tip = c + d * R * 0.25
    cv2.line(palm, tuple(int(v) for v in base), tuple(int(v) for v in tip), 255, int(11 * ui), AA)
    for k in range(6):
        p0 = base + d * R * (0.35 + k * 0.13)
        for side in (-1, 1):
            a = math.radians(62 * side)
            dr = np.array([d[0] * math.cos(a) - d[1] * math.sin(a), d[0] * math.sin(a) + d[1] * math.cos(a)])
            p1 = p0 + dr * R * 0.55 * (1 - k * 0.07)
            cv2.line(palm, tuple(int(v) for v in p0), tuple(int(v) for v in p1), 255, int(6 * ui), AA)
    a0 = math.degrees(math.atan2(-d[1], -d[0]))  # direction back to the shore = crescent opening
    cv2.ellipse(palm, tuple(int(v) for v in c), (int(R * 1.05), int(R * 1.05)), a0, 45, 315, 255, int(9 * ui), AA)
    return water, palm


def _free(mask, x0, y0, x1, y1):
    H, W = mask.shape
    x0, y0, x1, y1 = max(0, int(x0)), max(0, int(y0)), min(W, int(x1)), min(H, int(y1))
    return x1 > x0 and y1 > y0 and not mask[y0:y1, x0:x1].any()


def _burj(img, glow, c, ui):
    """Top-down Y-shaped super-tall tower with stepped wings."""
    for step, col in zip((1.0, 0.72, 0.45, 0.2), theme()["burj"]):
        for k in range(3):
            a = -math.pi / 2 + k * 2 * math.pi / 3
            L, w = 46 * ui * step, 10 * ui * max(step, 0.4)
            tip = (c[0] + math.cos(a) * L, c[1] + math.sin(a) * L)
            nx, ny = -math.sin(a) * w, math.cos(a) * w
            poly = np.array([(c[0] + nx, c[1] + ny), (tip[0] + nx * 0.6, tip[1] + ny * 0.6),
                             (tip[0] - nx * 0.6, tip[1] - ny * 0.6), (c[0] - nx, c[1] - ny)], np.int32)
            cv2.fillPoly(img, [poly], col, AA)
    cv2.circle(glow, (int(c[0]), int(c[1])), int(80 * ui), (200, 160, 90), -1, AA)


def _sail(img, c, ui):
    """Sail-shaped hotel on its own little island."""
    cv2.circle(img, (int(c[0]), int(c[1])), int(16 * ui), theme()["beach"], -1, AA)
    poly = np.array([(c[0] - 8 * ui, c[1] + 8 * ui), (c[0] + 10 * ui, c[1] - 14 * ui), (c[0] + 4 * ui, c[1] + 10 * ui)], np.int32)
    cv2.fillPoly(img, [poly], (235, 235, 245), AA)


def build(size, route=None, closed=False, ui=1.0, seed=None):
    """Static city image + data for animation. route = screen-space points of the track."""
    T = theme()
    W, H = size
    rng = np.random.default_rng(C.MAP_SEED if seed is None else seed)
    img = np.full((H, W, 3), T["ground"], np.uint8)
    noise = cv2.resize(rng.integers(0, 10, (H // 6 + 1, W // 6 + 1)).astype(np.uint8), (W, H))
    if T["noise"]:
        img = cv2.add(img, cv2.merge([noise // 2, noise // 2, noise]))
    if T.get("grid"):  # fine drafting grid
        for x in range(0, W, max(8, int(24 * ui))):
            cv2.line(img, (x, 0), (x, H), T["grid"], 1)
        for y in range(0, H, max(8, int(24 * ui))):
            cv2.line(img, (0, y), (W, y), T["grid"], 1)
    glow = np.zeros_like(img)

    water, palm = _sea_and_creek(W, H, ui)
    beach = cv2.dilate(water, np.ones((int(9 * ui) | 1, int(9 * ui) | 1), np.uint8))
    img[beach > 0] = T["beach"]
    img[water > 0] = T["sea"]
    img[palm > 0] = T["beach"]
    keep = cv2.bitwise_or(water, cv2.dilate(palm, np.ones((5, 5), np.uint8)))
    keep = cv2.dilate(keep, np.ones((int(7 * ui) | 1, int(7 * ui) | 1), np.uint8))
    route_mask = np.zeros_like(keep)
    road_w = int(30 * ui)
    has_route = route is not None and len(route) >= 2
    if has_route:
        cv2.polylines(route_mask, [np.array(route, np.int32).reshape(-1, 1, 2)], closed, 255, road_w + int(26 * ui))
    blocked = cv2.bitwise_or(keep, route_mask)

    # street grid (streets are not drawn over water)
    g = int(118 * ui)
    ox, oy = int(rng.integers(0, g)), int(rng.integers(0, g))
    streets = np.zeros_like(img)
    smask = np.zeros((H, W), np.uint8)
    lanes = []
    for i, x in enumerate(range(ox - g, W + g, g)):
        major = i % 3 == 0
        th = int((12 if major else 6) * ui)
        cv2.line(streets, (x, 0), (x, H), T["avenue"] if major else T["street"], th)
        cv2.line(smask, (x, 0), (x, H), 255, th)
        if major:
            lanes.append(((x, 0), (x, H)))
    for j, y in enumerate(range(oy - g, H + g, g)):
        major = j % 3 == 0
        th = int((12 if major else 6) * ui)
        cv2.line(streets, (0, y), (W, y), T["avenue"] if major else T["street"], th)
        cv2.line(smask, (0, y), (W, y), 255, th)
        if major:
            lanes.append(((0, y), (W, y)))
    sm = (smask > 0) & (water == 0)
    img[sm] = streets[sm]

    # blocks: parks or buildings (with lit windows in themes that have them)
    lights = []
    inset = int(10 * ui)
    landmark_spots = []
    for bx in range(ox - g, W + g, g):
        for by in range(oy - g, H + g, g):
            x0, y0, x1, y1 = bx + inset, by + inset, bx + g - inset, by + g - inset
            if x1 < 0 or y1 < 0 or x0 > W or y0 > H:
                continue
            if _free(blocked, x0, y0, x1, y1) and 0 < x0 and x1 < W and 0 < y0 and y1 < H:
                landmark_spots.append(((x0 + x1) / 2, (y0 + y1) / 2))
            if rng.random() < 0.07:
                if _free(blocked, x0, y0, x1, y1):
                    cv2.rectangle(img, (x0, y0), (x1, y1), T["park"], -1)
                    for _ in range(6):
                        cv2.circle(img, (int(rng.uniform(x0, x1)), int(rng.uniform(y0, y1))), int(5 * ui), T["tree"], -1, AA)
                continue
            # split block into 1-4 lots
            nx, ny = int(rng.integers(1, 3)), int(rng.integers(1, 3))
            lw, lh = (x1 - x0) / nx, (y1 - y0) / ny
            for i in range(nx):
                for j in range(ny):
                    m = 4 * ui
                    rx0, ry0 = x0 + i * lw + m, y0 + j * lh + m
                    rx1, ry1 = rx0 + lw * rng.uniform(0.55, 0.95) - m, ry0 + lh * rng.uniform(0.55, 0.95) - m
                    if rx1 - rx0 < 8 or ry1 - ry0 < 8 or not _free(blocked, rx0 - 2, ry0 - 2, rx1 + 8 * ui, ry1 + 8 * ui):
                        continue
                    tall = rng.random()
                    sh = int((3 + tall * 9) * ui)
                    p0, p1 = (int(rx0), int(ry0)), (int(rx1), int(ry1))
                    if T["shadow"] is not None:
                        cv2.rectangle(img, (p0[0] + sh, p0[1] + sh), (p1[0] + sh, p1[1] + sh), T["shadow"], -1)
                    col = T["buildings"][int(rng.integers(len(T["buildings"])))]
                    if T["outline"] is None:
                        col = tuple(int(min(255, v * (0.85 + tall * 0.3))) for v in col)
                    cv2.rectangle(img, p0, p1, col, -1)
                    if T["outline"] is None:
                        edge = tuple(min(255, v + 25) for v in col) if sum(col) < 450 else tuple(max(0, v - 30) for v in col)
                        cv2.rectangle(img, p0, p1, edge, 1)
                    else:
                        oc = T["outline"][int(rng.integers(len(T["outline"])))]
                        cv2.rectangle(img, p0, p1, oc, 1)
                        if T["glow"]:
                            cv2.rectangle(glow, p0, p1, oc, max(1, int(3 * ui)))
                    if tall > 0.85 and rx1 - rx0 > 26 * ui:  # helipad on the tallest ones
                        c = (int((rx0 + rx1) / 2), int((ry0 + ry1) / 2))
                        cv2.circle(img, c, int(8 * ui), (120, 120, 120), 1, AA)
                    if T["windows"] is None:
                        continue
                    step = max(4, int(7 * ui))
                    for wx in range(int(rx0 + 3), int(rx1 - 2), step):
                        for wy in range(int(ry0 + 3), int(ry1 - 2), step):
                            if rng.random() < 0.28:
                                wc = T["windows"][int(rng.integers(len(T["windows"])))]
                                dim = tuple(int(v * 0.45) for v in wc)
                                img[wy:wy + 2, wx:wx + 2] = dim
                                lights.append((wx, wy, wc, rng.uniform(0, 2 * math.pi), rng.uniform(0.3, 1.2)))

    # landmarks: the tallest tower near the centre, a sail hotel off the coast
    city = City()
    city.beacon = None
    if landmark_spots:
        spot = min(landmark_spots, key=lambda p: math.dist(p, (W * 0.5, H * 0.45)))
        x0, y0 = spot[0] - g / 2 + inset, spot[1] - g / 2 + inset
        cv2.rectangle(img, (int(x0), int(y0)), (int(x0 + g - 2 * inset), int(y0 + g - 2 * inset)), T["plaza"], -1)
        cv2.circle(img, (int(spot[0]), int(spot[1])), int(52 * ui), T["lake"], -1, AA)  # fountain lake
        _burj(img, glow, spot, ui)
        city.beacon = spot
        cv2.putText(img, "BURJ", (int(spot[0] - 18 * ui), int(spot[1] + 68 * ui)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45 * ui, T["label"], 1, AA)
    _sail(img, (W * 0.30, H * 0.93), ui)
    cv2.circle(glow, (int(W * 0.30), int(H * 0.93)), int(40 * ui), (180, 140, 255), -1, AA)

    # the route becomes a highway
    if has_route:
        draw_road(img, route, closed, road_w)

    if T["glow"]:
        if T["outline"] is not None:  # neon: sharp glow around outlines and the route
            if has_route:
                cv2.polylines(glow, [np.array(route, np.int32).reshape(-1, 1, 2)], closed, T["edge"], road_w)
            img = cv2.add(img, cv2.convertScaleAbs(cv2.GaussianBlur(glow, (0, 0), 6 * ui), alpha=0.5 * T["glow"]))
        else:
            img = cv2.add(img, cv2.GaussianBlur(glow, (0, 0), 30 * ui) // 3)

    # water shimmer sample points
    ys, xs = np.nonzero(water)
    idx = rng.choice(len(xs), size=min(260, len(xs)), replace=False) if len(xs) else []
    city.waves = [(int(xs[i]), int(ys[i]), rng.uniform(0, 2 * math.pi)) for i in idx]
    li = rng.permutation(len(lights))[:400] if lights else []
    city.lights = [lights[i] for i in li]
    city.theme = T
    city.traffic = [] if not (T["traffic"] and lanes) else [
        (lanes[int(rng.integers(len(lanes)))], rng.uniform(40, 120) * ui * rng.choice([-1, 1]), rng.uniform(0, 5000))
        for _ in range(26)]
    city.water = water
    city.base = img
    city.road_w = road_w
    return city


def get(size, route, closed, ui):
    key = (size, tuple(map(tuple, route)) if route is not None else None, closed, round(ui, 3), C.MAP_SEED,
           getattr(C, "MAP_THEME", "night"))
    if key not in _cache:
        _cache.clear()
        _cache[key] = build(size, route, closed, ui)
    return _cache[key]


def animate(img, city, t, ui):
    """Draw the moving parts onto a copy of city.base."""
    H, W = img.shape[:2]
    sea, sea_l = city.theme["sea"], city.theme["sea_light"]
    for x, y, ph in city.waves:  # water shimmer
        b = math.sin(t * 1.6 + ph)
        if b > 0.3:
            d = int((4 + 4 * b) * ui)
            dx = int(math.sin(t * 0.7 + ph) * 6 * ui)
            col = tuple(int(sea[i] + (sea_l[i] - sea[i]) * b) for i in range(3))
            cv2.line(img, (x - d + dx, y), (x + d + dx, y), col, 1, AA)
    for x, y, col, ph, sp in city.lights:  # twinkling windows
        if math.sin(t * sp + ph) > 0.2:
            img[y:y + 2, x:x + 2] = col
    for ((x0, y0), (x1, y1)), v, off in city.traffic:  # ambient traffic on the avenues
        L = math.hypot(x1 - x0, y1 - y0)
        s = (off + v * t) % L
        ux, uy = (x1 - x0) / L, (y1 - y0) / L
        px, py = x0 + ux * s, y0 + uy * s
        if not (0 <= px < W and 0 <= py < H) or city.water[int(py), int(px)]:
            continue
        sgn = 1 if v > 0 else -1
        lat = 3 * ui * sgn  # drive on the right
        px, py = px - uy * lat, py + ux * lat
        hx, hy = px + ux * sgn * 4 * ui, py + uy * sgn * 4 * ui
        tx, ty = px - ux * sgn * 4 * ui, py - uy * sgn * 4 * ui
        cv2.circle(img, (int(hx), int(hy)), max(1, int(1.6 * ui)), (220, 250, 255), -1, AA)
        cv2.circle(img, (int(tx), int(ty)), max(1, int(1.4 * ui)), (40, 40, 230), -1, AA)
    if city.beacon is not None and int(t * 1.5) % 2 == 0:  # red aircraft beacon on the tower
        cv2.circle(img, (int(city.beacon[0]), int(city.beacon[1])), max(2, int(4 * ui)), (40, 40, 255), -1, AA)
