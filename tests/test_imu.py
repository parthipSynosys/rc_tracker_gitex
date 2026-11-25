"""Run: python tests/test_imu.py  (also works with pytest). No sensor or camera needed:
the MPU-6050 readings are simulated."""
import os
import socket
import sys
import time

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _sub in ("", "aruco", "fusion", "maps", "ui", "sensor"):
    sys.path.insert(0, os.path.join(_ROOT, _sub))

import config as C
from imu_reader import ImuReader, Sample, parse
from imu_state import ImuState
from kinematics import (ACCELERATING, BRAKING, CAMERA, CRUISING, IMU, LEFT, NO_SENSOR, RIGHT, STOPPED,
                        STRAIGHT, Kinematics)

C.MOTION_SOURCE = "sensor"

HZ = 100
FPS = 30
SIDE = 80.0  # px; with MARKER_SIZE_CM=8 -> 0.1 cm/px


def feed(imu, t0, secs, forward_ms2, yaw_dps=0.0):
    """Simulate `secs` seconds of sensor readings. Returns the end time."""
    t = t0
    for _ in range(int(secs * HZ)):
        imu.update(t, forward_ms2(t - t0) if callable(forward_ms2) else forward_ms2, yaw_dps)
        t += 1 / HZ
    return t


def test_parse():
    s = parse("IMU,12,3456,0.512,-0.020,9.810,0.10,-0.20,45.00\n", 7.0)
    assert s == Sample(3.456, 0.512, -0.02, 9.81, 0.1, -0.2, 45.0, 7.0), s
    assert parse("hello") is None
    assert parse("IMU,1,2,3") is None
    assert parse("IMU,1,2,a,b,c,d,e,f") is None


def test_accelerating():
    imu = ImuState()
    feed(imu, 0, 0.5, 0.6)  # 0.6 m/s^2 = 60 cm/s^2 forward
    assert imu.motion == ACCELERATING, imu.motion
    assert abs(imu.accel - 60) < 3, imu.accel


def test_braking():
    imu = ImuState()
    feed(imu, 0, 0.5, -0.6)
    assert imu.motion == BRAKING, imu.motion


def test_cruising_and_no_flicker_in_vibration():
    imu = ImuState()
    states = set()
    t = 0.0
    for i in range(300):  # 3 s of +-0.4 m/s^2 vibration at 50 Hz around zero
        imu.update(t, 0.4 if i % 2 else -0.4, 0.0)
        states.add(imu.motion)
        t += 1 / HZ
    assert states == {CRUISING}, states


def test_turning():
    imu = ImuState()
    feed(imu, 0, 0.5, 0.0, 60.0)
    assert imu.turn == LEFT, imu.turn
    feed(imu, 0.5, 0.5, 0.0, -60.0)
    assert imu.turn == RIGHT, imu.turn
    feed(imu, 1.0, 0.5, 0.0, 2.0)
    assert imu.turn == STRAIGHT, imu.turn


def test_mounting_flip():
    imu = ImuState()
    old = C.IMU_FORWARD_SIGN
    try:
        C.IMU_FORWARD_SIGN = -1  # board mounted backwards
        for i in range(50):
            imu.feed(Sample(i / HZ, 0.6, 0, 9.8, 0, 0, 0, 0))
    finally:
        C.IMU_FORWARD_SIGN = old
    assert imu.motion == BRAKING, imu.motion


def _cruise(k, t0, secs, x0=100.0, y=300.0, speed=40.0):
    t, x = t0, x0
    for _ in range(int(secs * FPS)):
        x += speed / 0.1 / FPS
        k.update(t, x, y, SIDE)
        t += 1 / FPS
    return t, x


def test_sensor_decides_state():
    k = Kinematics()
    t, x = _cruise(k, 0, 2)  # no sensor yet
    assert k.state == NO_SENSOR and k.source == IMU and not k.sensor_ok
    imu = ImuState()
    feed(imu, 0, 0.5, -0.8, -60.0)  # sensor feels braking while turning right
    k.imu = imu
    _cruise(k, t, 0.2, x0=x)
    assert k.source == IMU
    assert k.state == BRAKING, k.state
    assert k.turn == RIGHT, k.turn
    assert k.brake_events == 1


def test_stopped_comes_from_camera():
    k = Kinematics()
    imu = ImuState()
    feed(imu, 0, 0.5, 0.8)  # sensor says accelerating ...
    k.imu = imu
    _cruise(k, 0, 2, speed=0.0)  # ... but the camera sees the car standing still
    assert k.state == STOPPED, k.state


def test_no_automatic_switch_to_camera():
    k = Kinematics()
    imu = ImuState()
    feed(imu, 0, 0.5, -0.8)
    k.imu = imu
    t, x = _cruise(k, 0, 2)
    assert k.state == BRAKING
    k.imu = None  # sensor unplugged: must NOT fall back to the camera
    _cruise(k, t, 1, x0=x)
    assert k.state == NO_SENSOR and k.accel == 0 and k.turn == STRAIGHT, (k.state, k.accel, k.turn)


def test_camera_mode_ignores_sensor():
    k = Kinematics()
    imu = ImuState()
    feed(imu, 0, 0.5, -0.8, -60.0)
    k.imu = imu
    C.MOTION_SOURCE = "camera"
    try:
        _cruise(k, 0, 2)
    finally:
        C.MOTION_SOURCE = "sensor"
    assert k.source == CAMERA and k.state == CRUISING and k.turn == STRAIGHT, (k.source, k.state, k.turn)


def test_camera_turn_in_camera_mode():
    """MOTION_SOURCE = "camera": driving a circle must show a turn (image y down: clockwise = right)."""
    import math
    C.MOTION_SOURCE = "camera"
    k = Kinematics()
    t, r, w = 0.0, 300.0, 1.0  # radius px, rad/s
    for i in range(int(3 * FPS)):
        a = w * t
        k.update(t, 640 + r * math.cos(a), 360 + r * math.sin(a), SIDE)  # angle grows -> clockwise on screen
        t += 1 / FPS
    C.MOTION_SOURCE = "sensor"
    assert k.turn == RIGHT, (k.turn, k.yaw_rate)
    assert abs(k.yaw_rate + 57.3) < 8, k.yaw_rate


def test_moving_and_estimates():
    imu = ImuState()
    t = 0.0
    for i in range(100):  # 1 s: +0.5 m/s^2 forward with motor vibration
        imu.update(t, 0.5 + (0.2 if i % 2 else -0.2), 0.0)
        t += 1 / HZ
    assert imu.moving
    assert abs(imu.speed - 50) < 5, imu.speed        # 0.5 m/s^2 for 1 s = 50 cm/s
    assert abs(imu.distance - 25) < 4, imu.distance  # 1/2 a t^2 = 25 cm
    t = feed(imu, t, 1.5, 0.0, 60.0)                 # turn left at 60 deg/s for 1.5 s
    assert abs(imu.heading - 90) < 5, imu.heading
    feed(imu, t, 1.0, 0.0, 0.0)                      # calm, no rotation: standing still
    assert not imu.moving and imu.speed == 0
    d = imu.distance
    imu.reset_estimates()
    assert imu.distance == 0 and imu.heading == 0 and d > 0


def test_reader_receives_udp():
    r = ImuReader(port=0)  # any free port
    assert r.start()
    try:
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        tx.sendto(b"IMU,1,1000,0.5,0,9.8,0,0,30\nIMU,2,1010,0.6,0,9.8,0,0,31\n", ("127.0.0.1", r.port))
        tx.close()
        deadline = time.perf_counter() + 2
        while r.count < 2 and time.perf_counter() < deadline:
            time.sleep(0.01)
        assert r.connected
        got = r.read()
        assert [s.ax for s in got] == [0.5, 0.6], got
        assert r.read() == []
    finally:
        r.stop()


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in tests:
        fn()
        print("PASS", fn.__name__)
    print(f"{len(tests)} tests passed")
