"""Speed / acceleration / motion-state estimation. Pure Python: no OpenCV, no camera."""
import math
from collections import deque

import config as C

ACCELERATING = "ACCELERATING"
CRUISING = "CRUISING"
BRAKING = "BRAKING"
STOPPED = "STOPPED"
NO_SIGNAL = "NO SIGNAL"


def lsq_slope(ts, vs):
    """Least-squares slope of vs against ts (0.0 if undefined)."""
    n = len(ts)
    if n < 2:
        return 0.0
    tm = sum(ts) / n
    vm = sum(vs) / n
    den = sum((t - tm) ** 2 for t in ts)
    if den < 1e-12:
        return 0.0
    return sum((t - tm) * (v - vm) for t, v in zip(ts, vs)) / den


class Kinematics:
    def __init__(self):
        self.scale = None  # cm per pixel, EMA-smoothed; survives reset()
        self.reset()

    def reset(self):
        """Clear all stats and buffers (the pixel scale is kept)."""
        self._pos = deque()    # (t, x_cm, y_cm)
        self._spd = deque()    # (t, speed)
        self.history = deque()  # (t, speed, accel, state) for the graphs
        self.speed = 0.0
        self.accel = 0.0
        self.vx = self.vy = 0.0
        self.heading = None    # radians, image coordinates (y down)
        self.state = NO_SIGNAL
        self.pos_px = None     # current position, None while lost
        self.last_px = None    # last position the marker was seen at
        self.t0 = None
        self.t_last = None
        self.lost_since = None
        self.distance = 0.0
        self.max_speed = 0.0
        self.max_accel = 0.0
        self.max_brake = 0.0
        self.brake_events = 0
        self.brake_time = 0.0

    # ------------------------------------------------------------ properties
    @property
    def elapsed(self):
        if self.t0 is None or self.t_last is None:
            return 0.0
        return self.t_last - self.t0

    @property
    def lost_for(self):
        """Seconds since the marker was last seen (0 if currently visible)."""
        if self.lost_since is None or self.t_last is None:
            return 0.0
        return self.t_last - self.lost_since

    @property
    def pos_cm(self):
        if self.pos_px is None or self.scale is None:
            return None
        return (self.pos_px[0] * self.scale, self.pos_px[1] * self.scale)

    # ------------------------------------------------------------ updates
    def update(self, t, x_px, y_px, side_px):
        """Feed one marker detection. t in seconds (monotonic)."""
        if side_px and side_px > 1:
            raw = C.MARKER_SIZE_CM / side_px
            a = C.SCALE_EMA_ALPHA
            self.scale = raw if self.scale is None else a * raw + (1 - a) * self.scale
        if self.scale is None:
            return
        if self.t0 is None:
            self.t0 = t
        dt = min(t - self.t_last, 0.2) if self.t_last is not None else 0.0
        self.t_last = t
        self.lost_since = None
        self.pos_px = self.last_px = (x_px, y_px)

        self._pos.append((t, x_px * self.scale, y_px * self.scale))
        while self._pos and t - self._pos[0][0] > C.SPEED_WINDOW_S:
            self._pos.popleft()

        speed_valid = len(self._pos) >= C.MIN_SPEED_SAMPLES
        if speed_valid:
            ts = [p[0] for p in self._pos]
            self.vx = lsq_slope(ts, [p[1] for p in self._pos])
            self.vy = lsq_slope(ts, [p[2] for p in self._pos])
            self.speed = math.hypot(self.vx, self.vy)
            self._spd.append((t, self.speed))
        else:
            self.speed = 0.0
        while self._spd and t - self._spd[0][0] > C.ACCEL_WINDOW_S:
            self._spd.popleft()

        self.accel = 0.0
        accel_valid = False
        if len(self._spd) >= 4 and self._spd[-1][0] - self._spd[0][0] >= 0.5 * C.ACCEL_WINDOW_S:
            self.accel = lsq_slope([s[0] for s in self._spd], [s[1] for s in self._spd])
            accel_valid = True

        if speed_valid and self.speed > C.STOP_SPEED:
            self.heading = math.atan2(self.vy, self.vx)

        prev = self.state
        self.state = self._next_state()
        if self.state == BRAKING and prev != BRAKING:
            self.brake_events += 1

        # stats
        if self.state != STOPPED:
            self.distance += self.speed * dt
        self.max_speed = max(self.max_speed, self.speed)
        if accel_valid:
            self.max_accel = max(self.max_accel, self.accel)
            self.max_brake = max(self.max_brake, -self.accel)
        if self.state == BRAKING:
            self.brake_time += dt
        self._log(t)

    def update_lost(self, t):
        """Call when the marker was not found in this frame."""
        if self.t0 is None:
            self.state = NO_SIGNAL
            return
        self.t_last = t
        if self.lost_since is None:
            self.lost_since = t
        self._pos.clear()  # avoid a fake spike when the marker reappears
        self._spd.clear()
        self.speed = self.accel = self.vx = self.vy = 0.0
        self.pos_px = None
        self.state = NO_SIGNAL
        self._log(t)

    # ------------------------------------------------------------ internals
    def _next_state(self):
        s, sp, a = self.state, self.speed, self.accel
        if s == STOPPED:
            if sp < C.STOP_SPEED + C.STOP_HYST:
                return STOPPED
        elif sp < C.STOP_SPEED:
            return STOPPED
        if s == BRAKING and a < -C.BRAKE_THRESHOLD * C.HYST_FRACTION:
            return BRAKING
        if s == ACCELERATING and a > C.ACCEL_THRESHOLD * C.HYST_FRACTION:
            return ACCELERATING
        if a < -C.BRAKE_THRESHOLD:
            return BRAKING
        if a > C.ACCEL_THRESHOLD:
            return ACCELERATING
        return CRUISING

    def _log(self, t):
        self.history.append((t, self.speed, self.accel, self.state))
        while self.history and t - self.history[0][0] > C.GRAPH_SECONDS:
            self.history.popleft()
