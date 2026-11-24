"""Run: python test_kinematics.py  (also works with pytest). No camera needed."""
import math

import config as C
from kinematics import Kinematics, ACCELERATING, BRAKING, CRUISING, STOPPED, NO_SIGNAL
from track import Track

FPS = 30
SIDE = 80.0  # px; with MARKER_SIZE_CM=8 -> 0.1 cm/px


def run(kin, t0, secs, speed_fn, x0=100.0, y=300.0):
    """Drive the marker along x. speed_fn(t_rel) is in cm/s. Returns (t_end, x_end)."""
    t, x = t0, x0
    for _ in range(int(secs * FPS)):
        x += speed_fn(t - t0) / 0.1 / FPS
        kin.update(t, x, y, SIDE)
        t += 1 / FPS
    return t, x


def test_constant_speed():
    k = Kinematics()
    run(k, 0, 3, lambda t: 30.0)
    assert k.state == CRUISING, k.state
    assert abs(k.speed - 30.0) < 1.5, k.speed


def test_braking():
    k = Kinematics()
    t, x = run(k, 0, 2, lambda t: 60.0)
    assert k.state == CRUISING
    run(k, t, 1.5, lambda r: max(0.0, 60 - 40 * r), x0=x)  # -40 cm/s^2
    assert k.brake_events == 1, k.brake_events
    assert k.max_brake > 25, k.max_brake
    assert k.brake_time > 0.5


def test_braking_state_while_slowing():
    k = Kinematics()
    t, x = run(k, 0, 2, lambda t: 60.0)
    run(k, t, 0.8, lambda r: 60 - 40 * r, x0=x)
    assert k.state == BRAKING, k.state
    assert k.accel < -25


def test_accelerating():
    k = Kinematics()
    run(k, 0, 1.5, lambda t: 40 * t)  # +40 cm/s^2
    assert k.state == ACCELERATING, k.state


def test_stopped():
    k = Kinematics()
    run(k, 0, 2, lambda t: 0.0)
    assert k.state == STOPPED
    assert k.speed < 1


def test_no_flicker_in_noise():
    k = Kinematics()
    states = []
    t = 0.0
    x = 100.0
    for i in range(FPS * 4):
        x += 30 / 0.1 / FPS + (2.0 if i % 2 else -2.0)  # +/-0.2 cm jitter
        k.update(t, x, 300.0, SIDE)
        states.append(k.state)
        t += 1 / FPS
    assert set(states[FPS:]) == {CRUISING}, set(states[FPS:])


def test_lost_marker_no_spike():
    k = Kinematics()
    t, x = run(k, 0, 2, lambda t: 30.0)
    for _ in range(15):
        k.update_lost(t)
        t += 1 / FPS
    assert k.state == NO_SIGNAL and k.speed == 0 and not k._pos and not k._spd
    peak = 0.0
    for i in range(30):  # reappears 500 px away
        k.update(t, x + 500, 300.0, SIDE)
        peak = max(peak, k.speed, abs(k.accel))
        t += 1 / FPS
    assert peak < 1.0, peak


def test_track_projection_square():
    tr = Track([(0, 0), (100, 0), (100, 100), (0, 100)], closed=True)
    r = tr.project((50, 0), 0.1)
    assert r["dist_cm"] == 0 and abs(r["progress_pct"] - 12.5) < 1e-6
    r = tr.project((100, 100), 0.1)
    assert abs(r["progress_pct"] - 50.0) < 1e-6
    r = tr.project((0, 50), 0.1)  # closing segment
    assert r["dist_cm"] == 0 and abs(r["progress_pct"] - 87.5) < 1e-6
    r = tr.project((50, -50), 0.1)  # 50 px * 0.1 = 5 cm
    assert abs(r["dist_cm"] - 5.0) < 1e-9 and r["on_track"]
    r = tr.project((50, -150), 0.1)  # 15 cm
    assert not r["on_track"]


def test_track_open_progress():
    tr = Track([(0, 0), (100, 0), (100, 100)], closed=False)
    assert tr.project((0, 0), 0.1)["progress_pct"] == 0
    assert abs(tr.project((100, 100), 0.1)["progress_pct"] - 100) < 1e-6


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in tests:
        fn()
        print("PASS", fn.__name__)
    print(f"{len(tests)} tests passed")
