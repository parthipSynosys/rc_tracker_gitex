"""Main loop: python monitor.py

Keys: r = reset stats | f = full screen <-> windows | v = switch view in full screen
(camera -> dashboard -> map) | m = map (mini window, or map view in full screen) | q / Esc = quit.
The RESET STATS button on the dashboard can also be clicked.
"""
import csv
import os
import time
from collections import deque
from datetime import datetime

import cv2
import numpy as np

import config as C
import dashboard
from kinematics import Kinematics, NO_SIGNAL
from track import Track, draw_track
from tracker import MarkerTracker, draw_detection

CSV_HEADER = ["time_s", "x_px", "y_px", "x_cm", "y_cm", "speed_cms", "accel_cms2",
              "state", "on_track", "deviation_cm", "progress_pct"]
CAM_WIN, DASH_WIN, MAP_WIN, FULL_WIN = "Camera", "Dashboard", "Map", "Tracker"
VIEWS = ("camera", "dashboard", "map")


class Monitor:
    def __init__(self):
        self.tracker = MarkerTracker()
        self.kin = Kinematics()
        self.track = Track.load()
        if self.track is None:
            print(f"No {C.TRACK_FILE} found - run `python track_editor.py` to draw a route. "
                  "Continuing without track features.")
        self.trail = deque(maxlen=C.TRAIL_MAX)
        self.show_map = False
        self.frame_size = (C.FRAME_WIDTH, C.FRAME_HEIGHT)
        self._csv_file = self._csv = None
        self._rows = 0
        self._last_state = None
        self._last_print = 0.0
        self._disp = (1.0, 0, 0)       # (scale, x offset, y offset) of the dashboard on screen
        self._flash_until = 0.0        # reset button highlight
        self.fullscreen = C.FULLSCREEN
        self.view = C.START_VIEW if C.START_VIEW in VIEWS else "camera"

    # ------------------------------------------------------------ window helpers
    def reset(self):
        self.kin.reset()
        self.trail.clear()
        self._flash_until = time.perf_counter() + 0.3
        print("Stats reset.")

    def _on_mouse(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if self.fullscreen and self.view != "dashboard":
            return  # the reset button only exists on the dashboard
        s, ox, oy = self._disp
        px, py = (x - ox) / s, (y - oy) / s  # screen -> dashboard coordinates
        bx, by, bw, bh = dashboard.RESET_BTN
        if bx <= px <= bx + bw and by <= py <= by + bh:
            self.reset()

    def _setup_windows(self):
        """(Re)create the windows for the current mode."""
        cv2.destroyAllWindows()
        self.show_map = False
        if self.fullscreen:
            cv2.namedWindow(FULL_WIN, cv2.WINDOW_NORMAL)
            cv2.setWindowProperty(FULL_WIN, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            cv2.setMouseCallback(FULL_WIN, self._on_mouse)
        else:
            try:  # screen size (Windows) so the two windows can sit side by side
                import ctypes
                sw = ctypes.windll.user32.GetSystemMetrics(0)
                sh = ctypes.windll.user32.GetSystemMetrics(1)
            except Exception:
                sw, sh = 1920, 1080
            half = sw // 2
            cv2.namedWindow(CAM_WIN, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(CAM_WIN, half, min(sh - 80, half * 9 // 16))
            cv2.moveWindow(CAM_WIN, 0, 0)
            if C.SHOW_DASHBOARD:
                cv2.namedWindow(DASH_WIN, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(DASH_WIN, half, min(sh - 80, half * 9 // 16))
                cv2.moveWindow(DASH_WIN, half, 0)
                cv2.setMouseCallback(DASH_WIN, self._on_mouse)

    @staticmethod
    def _fit(win, img):
        """Scale img to fit window `win` (aspect ratio kept, letterboxed).
        Returns (image to show, (scale, x offset, y offset))."""
        h, w = img.shape[:2]
        try:
            _, _, ww, wh = cv2.getWindowImageRect(win)
        except cv2.error:
            ww, wh = w, h
        if ww <= 0 or wh <= 0:
            ww, wh = w, h
        s = min(ww / w, wh / h)
        nw, nh = max(1, int(w * s)), max(1, int(h * s))
        interp = cv2.INTER_LINEAR if s > 1 else cv2.INTER_AREA
        out = np.full((wh, ww, 3), dashboard.BG, np.uint8)
        ox, oy = (ww - nw) // 2, (wh - nh) // 2
        out[oy:oy + nh, ox:ox + nw] = cv2.resize(img, (nw, nh), interpolation=interp)
        return out, (s, ox, oy)

    @staticmethod
    def _win_size(win):
        """Current drawable size of a window (falls back to DASHBOARD_SIZE)."""
        try:
            _, _, ww, wh = cv2.getWindowImageRect(win)
        except cv2.error:
            ww = wh = 0
        if ww < 320 or wh < 180:
            return tuple(C.DASHBOARD_SIZE)
        return ww, wh

    def _display(self, frame, info):
        """The dashboard / map are drawn at the window's own size, so the map fills the screen."""
        pressed = time.perf_counter() < self._flash_until
        dash = lambda win: dashboard.render(self.kin, self.track, info, self.trail, self.frame_size, pressed,
                                            self._win_size(win))
        if self.fullscreen:
            if self.view == "camera":
                cv2.putText(frame, "v: next view   f: windows   r: reset   q: quit", (10, frame.shape[0] - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
                out = self._fit(FULL_WIN, frame)[0]
            elif self.view == "dashboard":
                out = dash(FULL_WIN)
                self._disp = (1.0, 0, 0)
            else:
                out = dashboard.render_map(self.kin, self.track, info, self.trail, self.frame_size,
                                           self._win_size(FULL_WIN))
            cv2.imshow(FULL_WIN, out)
        else:
            cv2.imshow(CAM_WIN, self._fit(CAM_WIN, frame)[0])
            if C.SHOW_DASHBOARD:
                cv2.imshow(DASH_WIN, dash(DASH_WIN))
                self._disp = (1.0, 0, 0)
            if self.show_map:
                cv2.imshow(MAP_WIN, dashboard.render_map(self.kin, self.track, info, self.trail, self.frame_size,
                                                         (960, 540)))

    # ------------------------------------------------------------ per-frame logic (no camera needed)
    def process(self, frame, t):
        """Detect, update kinematics, project on track, draw overlays. Returns (det, info)."""
        h, w = frame.shape[:2]
        self.frame_size = (w, h)
        det = self.tracker.detect(frame)
        info = None
        if det.found:
            self.kin.update(t, det.center[0], det.center[1], det.side_px)
            self.trail.append(det.center)
            if self.track is not None and self.kin.scale:
                info = self.track.project(det.center, self.kin.scale)
        else:
            self.kin.update_lost(t)
        self._draw_camera(frame, det, info)
        self._log_csv(t, det, info)
        self._print(t, info)
        return det, info

    def _draw_camera(self, frame, det, info):
        if self.track is not None:
            corridor = C.OFF_TRACK_CM / self.kin.scale if self.kin.scale else None
            draw_track(frame, self.track, corridor)
        draw_detection(frame, det)
        k = self.kin
        txt = f"{k.state}  {k.speed:.1f} cm/s  {k.accel:+.0f} cm/s2"
        if info is not None:
            txt += f"  {'ON' if info['on_track'] else 'OFF'} TRACK {info['dist_cm']:.1f}cm"
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 30), (0, 0, 0), -1)
        cv2.putText(frame, txt, (10, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    C.STATE_COLORS.get(k.state, (255, 255, 255)), 2, cv2.LINE_AA)

    # ------------------------------------------------------------ csv / terminal
    def _open_csv(self):
        os.makedirs(C.LOG_DIR, exist_ok=True)
        path = os.path.join(C.LOG_DIR, f"run_{datetime.now():%Y%m%d_%H%M%S}.csv")
        self._csv_file = open(path, "w", newline="", encoding="utf-8")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(CSV_HEADER)
        print(f"Logging to {path}")

    def _log_csv(self, t, det, info):
        if self._csv is None:
            return
        k = self.kin
        if det.found and k.scale:
            xp, yp = f"{det.center[0]:.1f}", f"{det.center[1]:.1f}"
            xc, yc = f"{det.center[0] * k.scale:.2f}", f"{det.center[1] * k.scale:.2f}"
        else:
            xp = yp = xc = yc = ""
        on, dev, prog = ("", "", "") if info is None else (int(info["on_track"]), f"{info['dist_cm']:.2f}", f"{info['progress_pct']:.1f}")
        t0 = k.t0 if k.t0 is not None else t
        self._csv.writerow([f"{t - t0:.3f}", xp, yp, xc, yc, f"{k.speed:.2f}", f"{k.accel:.1f}", k.state, on, dev, prog])
        self._rows += 1
        if self._rows % C.CSV_FLUSH_EVERY == 0:
            self._csv_file.flush()

    def _print(self, t, info):
        k = self.kin
        changed = k.state != self._last_state
        if not changed and t - self._last_print < C.PRINT_INTERVAL_S:
            return
        self._last_state, self._last_print = k.state, t
        line = f"[{k.elapsed:7.1f}s] {k.state:<12} {k.speed:6.1f} cm/s  {k.accel:+7.1f} cm/s2"
        if info is not None and k.state != NO_SIGNAL:
            line += f" | {'ON ' if info['on_track'] else 'OFF'} TRACK {info['dist_cm']:5.1f} cm, {info['progress_pct']:3.0f}%"
        print(line + ("" if changed else "   (periodic)"))

    # ------------------------------------------------------------ main loop
    def run(self):
        if not self.tracker.open():
            print(f"ERROR: could not open camera {C.CAMERA_INDEX}. Close other apps using it "
                  "and check CAMERA_INDEX in config.py.")
            return 1
        self._open_csv()
        t_start = time.perf_counter()
        self._setup_windows()
        try:
            while True:
                frame = self.tracker.read()
                if frame is None:
                    if cv2.waitKey(10) & 0xFF in (ord("q"), 27):
                        break
                    continue
                t = time.perf_counter() - t_start
                det, info = self.process(frame, t)
                self._display(frame, info)
                k = cv2.waitKey(1) & 0xFF
                if k in (ord("q"), 27):
                    break
                elif k == ord("r"):
                    self.reset()
                elif k == ord("f"):
                    self.fullscreen = not self.fullscreen
                    self._setup_windows()
                elif k in (ord("v"), 9) and self.fullscreen:  # v or Tab
                    self.view = VIEWS[(VIEWS.index(self.view) + 1) % len(VIEWS)]
                elif k == ord("m"):
                    if self.fullscreen:
                        self.view = "map"
                    else:
                        self.show_map = not self.show_map
                        if not self.show_map:
                            cv2.destroyWindow(MAP_WIN)
        except KeyboardInterrupt:
            pass
        finally:
            self.tracker.release()
            if self._csv_file:
                self._csv_file.close()
            cv2.destroyAllWindows()
        return 0


if __name__ == "__main__":
    raise SystemExit(Monitor().run())
