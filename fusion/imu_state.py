"""Turns MPU-6050 readings into motion information. Pure Python: no network, no camera.
Fed one reading at a time by monitor.py.

Reliable:
- forward acceleration (accelerometer, axis IMU_FORWARD_AXIS) -> ACCELERATING / BRAKING / CRUISING
- turn rate (gyroscope, axis IMU_YAW_AXIS)                    -> turning LEFT / RIGHT / STRAIGHT
- moving or standing still (vibration + rotation)             -> moving
Estimates (they drift, so they are labelled "est." on the dashboard):
- turning angle: turn rate added up over time
- speed: forward acceleration added up over time (reset to 0 whenever the car stands still)
- distance: speed added up over time
Exact X,Y position and GPS location cannot be measured by this sensor.
"""
import math
from collections import deque

import config as C
from kinematics import CRUISING, STRAIGHT, next_motion, next_turn

_AXES = {"x": (1, 4), "y": (2, 5), "z": (3, 6)}  # index into Sample for (accel, gyro)


class ImuState:
    def __init__(self):
        self.reset()

    def reset(self):
        self.t = None          # device time of the last reading (s)
        self.accel = 0.0       # smoothed forward acceleration, cm/s^2 (+ = speeding up)
        self.yaw_rate = 0.0    # smoothed turn rate, deg/s (+ = turning left)
        self.motion = CRUISING
        self.turn = STRAIGHT
        self.samples = 0
        self.moving = False
        self._win = deque()    # (t, forward cm/s^2) for the vibration check
        self._still_since = None
        self.reset_estimates()

    def reset_estimates(self):
        """Zero the turning angle, speed and distance estimates (RESET STATS)."""
        self.heading = 0.0     # deg turned since start / reset, + = left, wrapped to -180..180
        self.speed = 0.0       # cm/s, estimate
        self.distance = 0.0    # cm, estimate

    @property
    def ready(self):
        """Enough readings for the smoothed values to be meaningful."""
        return self.samples >= C.IMU_MIN_SAMPLES

    def feed(self, s):
        """Feed one imu_reader.Sample (applies the mounting settings from config.py)."""
        fwd = s[_AXES[C.IMU_FORWARD_AXIS][0]] * C.IMU_FORWARD_SIGN
        yaw = s[_AXES[C.IMU_YAW_AXIS][1]] * C.IMU_YAW_SIGN
        self.update(s.t_dev, fwd, yaw)

    def update(self, t, forward_ms2, yaw_dps):
        """t in seconds (device clock), forward acceleration in m/s^2, turn rate in deg/s (left +)."""
        a = forward_ms2 * 100.0  # m/s^2 -> cm/s^2, same unit as the camera values
        dt = None if self.t is None else t - self.t
        if dt is None or not 0 < dt < 0.5:  # first reading, or the ESP32 restarted / data gap:
            dt = 0.01                       # count it as one normal 100 Hz step (no jump from one bump)
        k = 1 - math.exp(-dt / C.IMU_FILTER_S)
        self.accel += k * (a - self.accel)
        self.yaw_rate += k * (yaw_dps - self.yaw_rate)
        self.t = t
        self.samples += 1
        self.motion = next_motion(self.motion, self.accel, C.IMU_ACCEL_THRESHOLD, C.IMU_BRAKE_THRESHOLD)
        self.turn = next_turn(self.turn, self.yaw_rate, C.TURN_THRESHOLD_DPS)
        self._update_moving(t, a)
        self._update_estimates(dt, a, yaw_dps)

    def _update_moving(self, t, a):
        """Moving = motor vibration, a clear push or a rotation; still = calm for IMU_STILL_S."""
        self._win.append((t, a))
        while self._win and t - self._win[0][0] > C.IMU_MOVING_WINDOW_S:
            self._win.popleft()
        n = len(self._win)
        std = 0.0
        if n >= 5:
            m = sum(v for _, v in self._win) / n
            std = math.sqrt(sum((v - m) ** 2 for _, v in self._win) / n)
        active = (std > C.IMU_MOVING_STD or abs(self.accel) > C.IMU_ACCEL_THRESHOLD
                  or abs(self.yaw_rate) > C.IMU_MOVING_GYRO_DPS)
        if active:
            self.moving, self._still_since = True, None
        else:
            if self._still_since is None:
                self._still_since = t
            if t - self._still_since >= C.IMU_STILL_S:
                self.moving = False

    def _update_estimates(self, dt, a, yaw_dps):
        self.heading = (self.heading + yaw_dps * dt + 180.0) % 360.0 - 180.0
        if self.moving:  # forward only: a car does not reverse here, and this limits the drift
            self.speed = max(0.0, self.speed + a * dt)
        else:            # standing still: the true speed is 0, so throw away the drift
            self.speed = 0.0
        self.distance += self.speed * dt
