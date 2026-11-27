"""Speed / acceleration / motion-state estimation. Pure Python: no OpenCV, no camera."""
import math
from collections import deque

import config as C

ACCELERATING = "ACCELERATING"
CRUISING = "CRUISING"
BRAKING = "BRAKING"
STOPPED = "STOPPED"
NO_SIGNAL = "NO SIGNAL"
NO_SENSOR = "NO SENSOR"   # MOTION_SOURCE = "sensor" but no readings are arriving

LEFT = "LEFT"
RIGHT = "RIGHT"
STRAIGHT = "STRAIGHT"

CAMERA = "CAMERA"   # where the accelerating / braking / turning state comes from
IMU = "IMU"


def next_motion(prev, a, accel_thr, brake_thr):
    """ACCELERATING / BRAKING / CRUISING from acceleration a, with hysteresis
    (a state is kept until a falls inside HYST_FRACTION of its threshold)."""
    if prev == BRAKING and a < -brake_thr * C.HYST_FRACTION:
        return BRAKING
    if prev == ACCELERATING and a > accel_thr * C.HYST_FRACTION:
        return ACCELERATING
    if a < -brake_thr:
        return BRAKING
    if a > accel_thr:
        return ACCELERATING
    return CRUISING


def next_turn(prev, yaw_dps, thr):
    """LEFT / RIGHT / STRAIGHT from the turn rate (deg/s, left +), with hysteresis."""
    if prev == LEFT and yaw_dps > thr * C.TURN_HYST_FRACTION:
        return LEFT
    if prev == RIGHT and yaw_dps < -thr * C.TURN_HYST_FRACTION:
        return RIGHT
    if yaw_dps > thr:
        return LEFT
    if yaw_dps < -thr:
        return RIGHT
    return STRAIGHT


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


def trim_window(buf, t, window_s, min_samples, max_age_s=1.0):
    """Drop samples older than window_s, but keep at least min_samples so a low
    frame rate (e.g. 5 fps) still gives a value. Never keep anything older than max_age_s."""
    while buf and t - buf[0][0] > max_age_s:
        buf.popleft()
    while len(buf) > min_samples and t - buf[0][0] > window_s:
        buf.popleft()


class Kinematics:
    def __init__(self):
        self.scale = None  # cm per pixel, EMA-smoothed; survives reset()
        self.imu = None    # set by monitor.py to an ImuState while the sensor is connected
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
        self.turn = STRAIGHT     # LEFT / RIGHT / STRAIGHT
        self.yaw_rate = 0.0      # deg/s, + = turning left
        self.source = IMU if C.MOTION_SOURCE == "sensor" else CAMERA  # CAMERA or IMU: what the state / accel / turn come from (MOTION_SOURCE)
        self.sensor_ok = False   # True while the sensor is sending (only matters when source is IMU)
        self._hd = deque()       # (t, unwrapped heading) for the camera turn rate
        self._hd_raw = None

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
        dt = min(t - self.t_last, 0.5) if self.t_last is not None else 0.0
        self.t_last = t
        self.lost_since = None
        self.pos_px = self.last_px = (x_px, y_px)

        self._pos.append((t, x_px * self.scale, y_px * self.scale))
        trim_window(self._pos, t, C.SPEED_WINDOW_S, C.MIN_SPEED_SAMPLES)

        speed_valid = len(self._pos) >= C.MIN_SPEED_SAMPLES
        if speed_valid:
            ts = [p[0] for p in self._pos]
            self.vx = lsq_slope(ts, [p[1] for p in self._pos])
            self.vy = lsq_slope(ts, [p[2] for p in self._pos])
            self.speed = math.hypot(self.vx, self.vy)
            self._spd.append((t, self.speed))
        else:
            self.speed = 0.0
        trim_window(self._spd, t, C.ACCEL_WINDOW_S, 4)

        self.accel = 0.0
        accel_valid = False
        if len(self._spd) >= 4 and self._spd[-1][0] - self._spd[0][0] >= 0.5 * C.ACCEL_WINDOW_S:
            self.accel = lsq_slope([s[0] for s in self._spd], [s[1] for s in self._spd])
            accel_valid = True

        if speed_valid and self.speed > C.STOP_SPEED:
            self.heading = math.atan2(self.vy, self.vx)
        self._camera_turn(t, speed_valid)

        sensor_motion = None
        if self._use_sensor():  # sensor only: accelerating / braking / turning never come from the camera
            imu = self.imu if self.sensor_ok else None
            self.accel = imu.accel if imu else 0.0
            accel_valid = imu is not None
            self.turn, self.yaw_rate = (imu.turn, imu.yaw_rate) if imu else (STRAIGHT, 0.0)
            sensor_motion = imu.motion if imu else NO_SENSOR

        prev = self.state
        self.state = self._next_state(sensor_motion)
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
        self._hd.clear()
        self._hd_raw = None
        self.turn, self.yaw_rate = STRAIGHT, 0.0
        if self._use_sensor() and self.sensor_ok:
            self.turn, self.yaw_rate = self.imu.turn, self.imu.yaw_rate
        self._log(t)

    # ------------------------------------------------------------ internals
    def _use_sensor(self):
        """Read MOTION_SOURCE and whether the sensor is sending right now."""
        self.source = IMU if C.MOTION_SOURCE == "sensor" else CAMERA
        self.sensor_ok = self.imu is not None and self.imu.ready
        return self.source == IMU

    def _next_state(self, imu_motion=None):
        """STOPPED always comes from the camera speed; otherwise the sensor's state if given."""
        s, sp = self.state, self.speed
        if s == STOPPED:
            if sp < C.STOP_SPEED + C.STOP_HYST:
                return STOPPED
        elif sp < C.STOP_SPEED:
            return STOPPED
        if imu_motion is not None:
            return imu_motion
        return next_motion(s, self.accel, C.ACCEL_THRESHOLD, C.BRAKE_THRESHOLD)

    def _camera_turn(self, t, speed_valid):
        """Turn rate from how fast the direction of travel changes (used when there is no sensor).
        Image y points down, so a growing heading angle is a right turn seen from above."""
        if not speed_valid or self.speed <= C.STOP_SPEED + C.STOP_HYST:
            self._hd.clear()
            self._hd_raw = None
            self.turn, self.yaw_rate = STRAIGHT, 0.0
            return
        h = math.atan2(self.vy, self.vx)
        if self._hd_raw is None:
            unwrapped = h
        else:
            d = (h - self._hd_raw + math.pi) % (2 * math.pi) - math.pi
            unwrapped = self._hd[-1][1] + d
        self._hd_raw = h
        self._hd.append((t, unwrapped))
        trim_window(self._hd, t, C.TURN_WINDOW_S, 4)
        if len(self._hd) >= 4 and self._hd[-1][0] - self._hd[0][0] >= 0.5 * C.TURN_WINDOW_S:
            rate = lsq_slope([p[0] for p in self._hd], [p[1] for p in self._hd])
            self.yaw_rate = -math.degrees(rate)
        else:
            self.yaw_rate = 0.0
        self.turn = next_turn(self.turn, self.yaw_rate, C.TURN_THRESHOLD_DPS)

    def _log(self, t):
        self.history.append((t, self.speed, self.accel, self.state))
        while self.history and t - self.history[0][0] > C.GRAPH_SECONDS:
            self.history.popleft()
