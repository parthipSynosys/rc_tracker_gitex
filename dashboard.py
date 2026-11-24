"""Full-screen Dubai map dashboard.

The camera frame is mapped onto a Dubai map area (see dubai_map.py / config MAP_*): the
tracked marker is drawn as a car that drives and turns on the map, and a small telemetry
panel sits in the bottom-right corner. render() and render_map() are pure functions
(state in -> image out), so they work without a camera.
"""
import math
import time

import cv2
import numpy as np

import config as C
import city_map
import dubai_map
from kinematics import ACCELERATING, BRAKING, NO_SIGNAL

FONT = cv2.FONT_HERSHEY_SIMPLEX
AA = cv2.LINE_AA
BG = (18, 18, 18)
PANEL_BG = (24, 22, 20)
TEXT = (235, 235, 235)
DIM = (150, 150, 150)
ACCENT = (255, 190, 60)        # BGR light blue
ROUTE = (255, 210, 90)

PW, PH = 360, 314              # telemetry panel size at 1080p (scaled with the window)
RESET_BTN = (0, 0, 0, 0)       # x, y, w, h in screen px - updated by render(), clicked in monitor.py

_base_cache = {}
_last_heading = -math.pi / 2   # car points up until it has moved


def put(img, text, org, scale=0.6, color=TEXT, thick=1, anchor="l"):
    (w, h), _ = cv2.getTextSize(text, FONT, scale, thick)
    x, y = org
    if anchor == "c":
        x -= w // 2
    elif anchor == "r":
        x -= w
    cv2.putText(img, text, (int(x), int(y)), FONT, scale, color, thick, AA)


def _poly(img, pts, color, outline=None):
    a = np.round(np.asarray(pts, np.float64) * 16).astype(np.int32).reshape(-1, 1, 2)
    cv2.fillPoly(img, [a], color, AA, shift=4)
    if outline is not None:
        cv2.polylines(img, [a], True, outline, 1, AA, shift=4)


# ------------------------------------------------------------------ map + coordinates
def _base(size):
    """Map image scaled to cover `size` (cropped to fill, no black bars). Cached per size."""
    if size not in _base_cache:
        img = cv2.convertScaleAbs(dubai_map.load(), alpha=C.MAP_BRIGHTNESS)
        mh, mw = img.shape[:2]
        w, h = size
        s = max(w / mw, h / mh)
        nw, nh = max(w, math.ceil(mw * s)), max(h, math.ceil(mh * s))
        r = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_AREA if s < 1 else cv2.INTER_CUBIC)
        ox, oy = (nw - w) // 2, (nh - h) // 2
        _base_cache.clear()
        _base_cache[size] = (r[oy:oy + h, ox:ox + w].copy(), s, ox, oy, mw, mh)
    return _base_cache[size]


def _city_mode():
    return C.MAP_STYLE == "city"


def _projector(frame_size, size):
    """camera px -> screen px. The camera frame is fitted into the map image, which covers the screen
    (city mode: the generated city is the screen itself)."""
    if _city_mode():
        s, ox, oy, (mw, mh) = 1.0, 0, 0, size
    else:
        _, s, ox, oy, mw, mh = _base(size)
    fw, fh = frame_size
    k = min(mw / fw, mh / fh)
    mx0, my0 = (mw - fw * k) / 2, (mh - fh * k) / 2
    f = k * s  # screen px per camera px

    def to_screen(p):
        return ((mx0 + p[0] * k) * s - ox, (my0 + p[1] * k) * s - oy)

    return to_screen, f


def _ipt(p):
    return int(round(p[0])), int(round(p[1]))


# ------------------------------------------------------------------ car
def _draw_car(img, c, heading, color, braking, ui, layer="body"):
    """layer: "glow" (halo/beam/brake glow, drawn on the translucent overlay), "body" or "ghost"."""
    ghost = layer == "ghost"
    L, Wd = 46 * ui, 23 * ui
    ca, sa = math.cos(heading), math.sin(heading)

    def P(fx, fy):  # car coords (forward, right) -> screen
        return (c[0] + fx * ca - fy * sa, c[1] + fx * sa + fy * ca)

    h2, w2, ch = L / 2, Wd / 2, 3 * ui
    if layer == "glow":
        pulse = 1 + 0.12 * math.sin(time.perf_counter() * 5)
        cv2.circle(img, _ipt(c), int(30 * ui * pulse), color, -1, AA)
        _poly(img, [P(h2, -Wd / 3), P(h2 + 110 * ui, -46 * ui),  # headlight beam
                    P(h2 + 110 * ui, 46 * ui), P(h2, Wd / 3)], (170, 235, 255))
        if braking:
            for side in (-1, 1):
                cv2.circle(img, _ipt(P(-h2 - 4 * ui, side * (w2 - 4 * ui))), int(9 * ui), (40, 40, 255), -1, AA)
        return

    body = [(h2, -w2 + ch), (h2 - ch, -w2), (-h2 + ch * 0.7, -w2), (-h2, -w2 + ch * 0.7),
            (-h2, w2 - ch * 0.7), (-h2 + ch * 0.7, w2), (h2 - ch, w2), (h2, w2 - ch)]
    sh = 3 * ui
    _poly(img, [(P(fx, fy)[0] + sh, P(fx, fy)[1] + sh) for fx, fy in body], (8, 8, 8))
    body_col = (110, 110, 110) if ghost else color
    _poly(img, [P(*q) for q in body], body_col, (255, 255, 255))
    roof = tuple(int(v * 0.7) for v in body_col)
    _poly(img, [P(L * 0.08, -w2 + 2.5 * ui), P(-L * 0.24, -w2 + 2.5 * ui),
                P(-L * 0.24, w2 - 2.5 * ui), P(L * 0.08, w2 - 2.5 * ui)], roof)
    glass = (40, 30, 25)
    _poly(img, [P(L * 0.26, -w2 + 3 * ui), P(L * 0.08, -w2 + 2.5 * ui),
                P(L * 0.08, w2 - 2.5 * ui), P(L * 0.26, w2 - 3 * ui)], glass)
    _poly(img, [P(-L * 0.24, -w2 + 2.5 * ui), P(-L * 0.36, -w2 + 3.2 * ui),
                P(-L * 0.36, w2 - 3.2 * ui), P(-L * 0.24, w2 - 2.5 * ui)], glass)
    if not ghost:
        for side in (-1, 1):
            cv2.circle(img, _ipt(P(h2 - 1.5 * ui, side * (w2 - 3.5 * ui))), max(1, int(2.2 * ui)), (210, 255, 255), -1, AA)
            tail = (40, 40, 255) if braking else (30, 30, 140)
            _poly(img, [P(-h2 + 0.5 * ui, side * (w2 - 1.5 * ui)), P(-h2 + 2.5 * ui, side * (w2 - 1.5 * ui)),
                        P(-h2 + 2.5 * ui, side * (w2 - 6 * ui)), P(-h2 + 0.5 * ui, side * (w2 - 6 * ui))], tail)


# ------------------------------------------------------------------ full-screen map
def render_map(kin, track=None, info=None, trail_px=(), frame_size=(1280, 720), size=None):
    """Full-screen Dubai map with route, trail and the moving car (no telemetry panel)."""
    global _last_heading
    size = tuple(size or C.DASHBOARD_SIZE)
    W, H = size
    ui = max(0.6, min(2.0, H / 1080))
    to_screen, f = _projector(frame_size, size)
    city = _city_mode()
    has_route = track is not None and bool(track.points)
    if has_route:
        pts_f = [to_screen(p) for p in track.points]
        pts = np.array([_ipt(p) for p in pts_f], np.int32).reshape(-1, 1, 2)
        closed = track.closed and len(track.points) >= 3

    if city:  # generated city: the route is already a highway in the cached base
        cm = city_map.get(size, pts.reshape(-1, 2).tolist() if has_route else None, has_route and closed, ui)
        img = cm.base.copy()
        city_map.animate(img, cm, time.perf_counter(), ui)
    else:
        img = _base(size)[0].copy()
    overlay = img.copy()

    # route: wide glow + corridor (overlay), bright line on top
    if has_route and not city:
        if kin.scale:  # off-track corridor: very faint band
            r = max(2, int(C.OFF_TRACK_CM / kin.scale * f))
            band = img.copy()
            cv2.polylines(band, [pts], closed, C.CORRIDOR_COLOR, 2 * r, AA)
            img = cv2.addWeighted(band, 0.12, img, 0.88, 0)
            overlay = img.copy()
        cv2.polylines(overlay, [pts], closed, ROUTE, int(14 * ui), AA)
    trail = [_ipt(to_screen(p)) for p in trail_px]
    if city and len(trail) >= 2:  # the driven path is paved as a road
        a = np.array(trail, np.int32).reshape(-1, 1, 2)
        w = int(18 * ui)
        T = city_map.theme()
        cv2.polylines(overlay, [a], False, T["trail"], w + int(14 * ui), AA)  # glow
        cv2.polylines(img, [a], False, T["edge"], w + 4, AA)
        cv2.polylines(img, [a], False, T["asphalt"], w, AA)

    # car position / heading
    color = C.STATE_COLORS.get(kin.state, DIM)
    car = ghost = None
    if kin.pos_px is not None:
        car = to_screen(kin.pos_px)
        if kin.heading is not None and kin.speed > C.STOP_SPEED:
            _last_heading = kin.heading
    elif kin.last_px is not None:
        ghost = to_screen(kin.last_px)

    if car is not None:  # translucent parts of the car (halo, beam) go into the overlay
        _draw_car(overlay, car, _last_heading, color, kin.state == BRAKING, ui, "glow")
    img = cv2.addWeighted(overlay, 0.28, img, 0.72, 0)

    if has_route:
        if len(track.points) >= 2 and not city:
            cv2.polylines(img, [pts], closed, ROUTE, max(2, int(3 * ui)), AA)
        for p, col, lab in ((pts[0, 0], C.START_COLOR, "START"),
                            (pts[-1, 0], C.FINISH_COLOR, "FINISH")):
            if lab == "FINISH" and (track.closed or len(track.points) < 2):
                continue
            cv2.circle(img, tuple(int(v) for v in p), int(9 * ui), col, -1, AA)
            cv2.circle(img, tuple(int(v) for v in p), int(9 * ui), (255, 255, 255), 2, AA)
            org = (p[0] + 14 * ui, p[1] + 5 * ui)
            put(img, lab, org, 0.5 * ui, (0, 0, 0), max(3, int(5 * ui)))
            put(img, lab, org, 0.5 * ui, (255, 255, 255), max(1, int(2 * ui)))

    n = len(trail)
    for i in range(1, n):  # older = dimmer
        k = 0.2 + 0.8 * i / n
        tc = city_map.theme()["trail"] if city else (40, 190, 255)
        cv2.line(img, trail[i - 1], trail[i], tuple(int(v * k) for v in tc),
                 max(2, int((2 if city else 4) * ui)), AA)

    if car is not None:
        if info is not None and not info["on_track"]:
            cv2.line(img, _ipt(car), _ipt(to_screen(info["nearest_px"])), (60, 60, 255), max(2, int(2 * ui)), AA)
        _draw_car(img, car, _last_heading, color, kin.state == BRAKING, ui)
    elif ghost is not None:
        _draw_car(img, ghost, _last_heading, DIM, False, ui, "ghost")
        put(img, f"signal lost {kin.lost_for:.1f}s", (ghost[0] + 28 * ui, ghost[1] + 5 * ui), 0.55 * ui, (80, 170, 255),
            max(1, int(2 * ui)))

    _title(img, kin, ui)
    if not city:
        put(img, dubai_map.attribution(), (10 * ui, H - 10 * ui), 0.38 * ui, (170, 170, 170), 1)
    return img


def _title(img, kin, ui):
    x, y, w, h = int(20 * ui), int(20 * ui), int(300 * ui), int(70 * ui)
    roi = img[y:y + h, x:x + w]
    roi[:] = cv2.addWeighted(np.full_like(roi, PANEL_BG), 0.8, roi, 0.2, 0)
    put(img, "DUBAI", (x + 16 * ui, y + 32 * ui), 0.9 * ui, TEXT, max(1, int(2 * ui)))
    put(img, C.MAP_LABEL, (x + 16 * ui, y + 56 * ui), 0.45 * ui, DIM, 1)
    live = kin.state != NO_SIGNAL
    blink = live and int(time.perf_counter() * 2) % 2 == 0
    cv2.circle(img, (int(x + w - 70 * ui), int(y + 26 * ui)), int(6 * ui),
               (60, 60, 255) if blink else ((40, 40, 140) if live else (90, 90, 90)), -1, AA)
    put(img, "LIVE" if live else "NO SIGNAL", (x + w - 58 * ui, y + 31 * ui), 0.45 * ui, TEXT if live else DIM, 1)


# ------------------------------------------------------------------ telemetry panel
def _fmt_time(s):
    return f"{int(s) // 60:02d}:{int(s) % 60:02d}"


def _panel(kin, info, pressed):
    """Telemetry panel at its 1080p size (PW x PH); scaled by render()."""
    p = np.full((PH, PW, 3), PANEL_BG, np.uint8)
    col = C.STATE_COLORS.get(kin.state, DIM)
    cv2.rectangle(p, (0, 0), (PW, 4), col, -1)

    cv2.circle(p, (22, 29), 7, col, -1, AA)
    put(p, kin.state, (36, 35), 0.6, TEXT, 2)
    if info is not None and kin.state != NO_SIGNAL:
        ok = info["on_track"]
        put(p, "ON TRACK" if ok else "OFF TRACK", (PW - 16, 35), 0.5, (90, 210, 90) if ok else (70, 70, 245), 2, "r")
    else:
        put(p, "NO TRACK" if info is None else "", (PW - 16, 35), 0.5, DIM, 1, "r")

    spd = f"{kin.speed:.1f}"
    put(p, spd, (16, 96), 1.8, TEXT, 3)
    (tw, _), _ = cv2.getTextSize(spd, FONT, 1.8, 3)
    put(p, "cm/s", (24 + tw, 96), 0.6, DIM, 1)
    for i, (lab, on, con, coff) in enumerate((("THROTTLE", kin.state == ACCELERATING, (0, 210, 0), (0, 60, 0)),
                                              ("BRAKE", kin.state == BRAKING, (40, 40, 245), (0, 0, 70)))):
        cy = 64 + i * 26
        cv2.circle(p, (PW - 104, cy), 8, con if on else coff, -1, AA)
        cv2.circle(p, (PW - 104, cy), 8, DIM, 1, AA)
        put(p, lab, (PW - 88, cy + 5), 0.42, TEXT if on else DIM, 1)

    put(p, f"accel {kin.accel:+.1f} cm/s2", (16, 120), 0.45, DIM)
    bx, by, bw, bh = 16, 127, PW - 32, 10
    cv2.rectangle(p, (bx, by), (bx + bw, by + bh), (45, 45, 45), -1)
    frac = max(-1.0, min(1.0, kin.accel / C.ACCEL_BAR_MAX))
    mid = bx + bw // 2
    if frac >= 0:
        cv2.rectangle(p, (mid, by), (mid + int(frac * bw / 2), by + bh), (0, 200, 0), -1)
    else:
        cv2.rectangle(p, (mid + int(frac * bw / 2), by), (mid, by + bh), (40, 40, 230), -1)
    cv2.line(p, (mid, by - 3), (mid, by + bh + 3), TEXT, 1)

    # speed sparkline (last GRAPH_SECONDS)
    gx0, gy0, gx1, gy1 = 16, 150, PW - 16, 190
    cv2.rectangle(p, (gx0, gy0), (gx1, gy1), (34, 32, 30), -1)
    hist = list(kin.history)
    if hist:
        t_end = kin.t_last if kin.t_last is not None else hist[-1][0]
        t_start = t_end - C.GRAPH_SECONDS
        hi = max([r[1] for r in hist] + [20.0]) * 1.15
        pts = [(int(gx0 + (r[0] - t_start) / C.GRAPH_SECONDS * (gx1 - gx0)),
                int(gy1 - r[1] / hi * (gy1 - gy0))) for r in hist if r[0] >= t_start]
        if len(pts) >= 2:
            fill = np.array(pts + [(pts[-1][0], gy1), (pts[0][0], gy1)], np.int32)
            cv2.fillPoly(p, [fill], (90, 64, 22), AA)
            cv2.polylines(p, [np.array(pts, np.int32)], False, ACCENT, 2, AA)
    put(p, f"speed {C.GRAPH_SECONDS:.0f}s", (gx0 + 5, gy0 + 13), 0.35, DIM)

    on = info is not None and kin.state != NO_SIGNAL
    stats = (
        ("DISTANCE", f"{kin.distance / 100:.2f} m"),
        ("MAX SPEED", f"{kin.max_speed:.1f}"),
        ("TIME", _fmt_time(kin.elapsed)),
        ("BRAKES", f"{kin.brake_events}"),
        ("DEVIATION", f"{info['dist_cm']:.1f} cm" if on else "--"),
        ("PROGRESS", f"{info['progress_pct']:.0f}%" if on else "--"),
    )
    for i, (k, v) in enumerate(stats):
        x = 16 + (i % 3) * 112
        y = 212 + (i // 3) * 36
        put(p, k, (x, y), 0.35, DIM)
        put(p, v, (x, y + 18), 0.55, TEXT, 1)

    bx, by, bw, bh = 16, 280, PW - 32, 24
    cv2.rectangle(p, (bx, by), (bx + bw, by + bh), (60, 60, 230) if pressed else (60, 58, 56), -1)
    put(p, "RESET STATS  (r)", (bx + bw // 2, by + 17), 0.45, (255, 255, 255), 1, "c")
    return p, (bx, by, bw, bh)


def render(kin, track=None, info=None, trail_px=(), frame_size=(1280, 720), reset_pressed=False, size=None):
    """Full-screen map + small telemetry panel bottom-right. Sets RESET_BTN (screen px)."""
    global RESET_BTN
    size = tuple(size or C.DASHBOARD_SIZE)
    img = render_map(kin, track, info, trail_px, frame_size, size)
    W, H = size
    ui = max(0.6, min(2.0, H / 1080))
    panel, (bx, by, bw, bh) = _panel(kin, info, reset_pressed)
    pw, ph = int(PW * ui), int(PH * ui)
    if (pw, ph) != (PW, PH):
        panel = cv2.resize(panel, (pw, ph), interpolation=cv2.INTER_AREA if ui < 1 else cv2.INTER_LINEAR)
    m = int(20 * ui)
    x0, y0 = W - pw - m, H - ph - m
    if x0 >= 0 and y0 >= 0:
        roi = img[y0:y0 + ph, x0:x0 + pw]
        roi[:] = cv2.addWeighted(panel, 0.9, roi, 0.1, 0)
        cv2.rectangle(img, (x0, y0), (x0 + pw, y0 + ph), (80, 80, 80), 1)
        RESET_BTN = (x0 + int(bx * ui), y0 + int(by * ui), int(bw * ui), int(bh * ui))
    return img


if __name__ == "__main__":
    # Render one synthetic frame to dashboard_preview.png (no camera needed).
    from collections import deque
    from kinematics import Kinematics
    from track import Track

    C.MARKER_SIZE_CM = 8.0
    track = Track([(200, 150), (1000, 150), (1000, 550), (200, 550)], closed=True, frame_size=(1280, 720))
    kin = Kinematics()
    trail = deque(maxlen=C.TRAIL_MAX)
    t, x, v = 0.0, 200.0, 0.0
    side = 80.0  # 8 cm -> 0.1 cm/px
    info = None
    while t < 2.3:
        a = 400 if t < 1.0 else (0 if t < 1.9 else -400)  # px/s^2: accelerate, cruise, brake
        v = max(0.0, v + a * 1 / 30)
        x = min(x + v / 30, 1000)
        y = 150 + (130 if t > 1.5 else 0)  # drift ~13 cm off the line
        kin.update(t, x, y, side)
        trail.append((x, y))
        info = track.project((x, y), kin.scale)
        t += 1 / 30
    cv2.imwrite("dashboard_preview.png", render(kin, track, info, trail))
    cv2.imwrite("map_preview.png", render_map(kin, track, info, trail))
    print("wrote dashboard_preview.png, map_preview.png; final state:", kin.state)
