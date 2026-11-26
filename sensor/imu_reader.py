"""Receives MPU-6050 readings from the ESP32 over WiFi.

Two ways of connecting (config.IMU_HOST picks one):
- IMU_HOST = None:  the ESP32 broadcasts UDP packets to IMU_PORT; we just listen.
      line format:  IMU,<seq>,<millis>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>   (offsets already removed)
- IMU_HOST = "ip":  the ESP32 runs a TCP server (telnet style); we connect to IMU_HOST:IMU_TCP_PORT.
      line format:  AccX:<ax>,AccY:<ay>,AccZ:<az>,GyroX:<gx>,GyroY:<gy>,GyroZ:<gz>
      This firmware does not remove the resting offset, so the first IMU_CALIBRATE_S seconds after
      connecting are averaged (keep the car STILL) and that offset is subtracted on the PC.

Runs in a background thread so the camera loop never waits for it. If no ESP32 is reachable,
nothing happens: `connected` stays False and the monitor keeps going without the sensor.
"""
import socket
import threading
import time
from collections import deque, namedtuple

import config as C

# t_dev: device clock (s) | ax..az: m/s^2 | gx..gz: deg/s | t_pc: PC time.perf_counter() on arrival
Sample = namedtuple("Sample", "t_dev ax ay az gx gy gz t_pc")

_KV_KEYS = ("AccX", "AccY", "AccZ", "GyroX", "GyroY", "GyroZ")


def parse(line, t_pc=0.0):
    """One text line -> Sample, or None if it is not a valid IMU line.
    Accepts both the 'IMU,...' format and the 'AccX:..,AccY:..' format (which has no device
    clock, so the PC arrival time is used as t_dev)."""
    line = line.strip()
    if line.startswith("AccX:"):
        try:
            kv = dict(p.split(":", 1) for p in line.split(","))
            ax, ay, az, gx, gy, gz = (float(kv[k]) for k in _KV_KEYS)
        except (ValueError, KeyError):
            return None
        return Sample(t_pc, ax, ay, az, gx, gy, gz, t_pc)
    parts = line.split(",")
    if len(parts) != 9 or parts[0] != "IMU":
        return None
    try:
        ms = int(parts[2])
        ax, ay, az, gx, gy, gz = (float(v) for v in parts[3:])
    except ValueError:
        return None
    return Sample(ms / 1000.0, ax, ay, az, gx, gy, gz, t_pc)


class ImuReader:
    def __init__(self, port=None, host=None):
        self.host = getattr(C, "IMU_HOST", None) if host is None else host
        if port is None:
            port = C.IMU_TCP_PORT if self.host else C.IMU_PORT
        self.port = port
        self.device = None       # IP address of the ESP32 once it has sent something
        self.count = 0           # readings received
        self.last_rx = None      # perf_counter() of the last reading
        self._buf = deque(maxlen=2000)
        self._lock = threading.Lock()
        self._sock = None
        self._thread = None
        self._running = False
        self._bias = None        # TCP mode: (ax, ay, gx, gy, gz) resting offset
        self._calib = []

    # ------------------------------------------------------------ start / stop
    def start(self):
        """Start receiving. Returns False (and the program carries on) if that is impossible."""
        if self.host:
            self._running = True
            self._thread = threading.Thread(target=self._run_tcp, name="imu-reader", daemon=True)
            self._thread.start()
            return True
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._sock.bind(("", self.port))
            self._sock.settimeout(0.2)
        except OSError as e:
            print(f"Sensor: could not listen on UDP port {self.port} ({e}). Continuing with camera only.")
            self._sock = None
            return False
        self.port = self._sock.getsockname()[1]
        self._running = True
        self._thread = threading.Thread(target=self._run_udp, name="imu-reader", daemon=True)
        self._thread.start()
        return True

    def stop(self):
        self._running = False
        sock, self._sock = self._sock, None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    @property
    def describe(self):
        return f"TCP {self.host}:{self.port}" if self.host else f"UDP port {self.port}"

    # ------------------------------------------------------------ UDP (ESP32 broadcasts)
    def _run_udp(self):
        while self._running:
            try:
                data, addr = self._sock.recvfrom(4096)
            except socket.timeout:
                continue
            except (OSError, AttributeError):
                break
            now = time.perf_counter()
            samples = [s for s in (parse(l, now) for l in data.decode("ascii", "ignore").splitlines()) if s]
            self._store(samples, addr[0], now)

    # ------------------------------------------------------------ TCP (ESP32 is the server)
    def _run_tcp(self):
        warned = False
        while self._running:
            try:
                sock = socket.create_connection((self.host, self.port), timeout=2.0)
            except OSError as e:
                if not warned:
                    print(f"Sensor: cannot reach {self.host}:{self.port} ({e}). Retrying in the background...")
                    warned = True
                time.sleep(1.0)
                continue
            warned = False
            sock.settimeout(max(C.IMU_TIMEOUT_S, 1.0) * 2)
            self._sock = sock
            self._bias, self._calib = None, []
            print(f"Sensor: TCP connected to {self.host}:{self.port}. "
                  f"Keep the car still for {C.IMU_CALIBRATE_S:g} s (calibrating)...")
            pending = b""
            try:
                while self._running:
                    data = sock.recv(4096)
                    if not data:
                        raise OSError("connection closed by the sensor")
                    pending += data
                    *lines, pending = pending.split(b"\n")
                    now = time.perf_counter()
                    samples = [s for s in (parse(l.decode("ascii", "ignore"), now) for l in lines) if s]
                    self._store([c for c in (self._calibrate(s) for s in samples) if c], self.host, now)
            except (OSError, socket.timeout) as e:
                if self._running:
                    print(f"Sensor: TCP connection lost ({e}). Reconnecting...")
            finally:
                self._sock = None
                try:
                    sock.close()
                except OSError:
                    pass
                time.sleep(0.5)

    def _calibrate(self, s):
        """Collect the resting offset first, then return offset-corrected samples."""
        if C.IMU_CALIBRATE_S <= 0:
            return s
        if self._bias is None:
            self._calib.append(s)
            if s.t_pc - self._calib[0].t_pc < C.IMU_CALIBRATE_S:
                return None
            n = len(self._calib)
            self._bias = tuple(sum(getattr(c, f) for c in self._calib) / n for f in ("ax", "ay", "gx", "gy", "gz"))
            self._calib = []
            b = self._bias
            print(f"Sensor: calibration done ({n} readings). Offset ax={b[0]:+.2f} ay={b[1]:+.2f} m/s^2, "
                  f"gyro {b[2]:+.2f} {b[3]:+.2f} {b[4]:+.2f} deg/s")
            return None
        bax, bay, bgx, bgy, bgz = self._bias
        return s._replace(ax=s.ax - bax, ay=s.ay - bay, gx=s.gx - bgx, gy=s.gy - bgy, gz=s.gz - bgz)

    # ------------------------------------------------------------ shared
    def _store(self, samples, device, now):
        if not samples:
            return
        with self._lock:
            self._buf.extend(samples)
            self.count += len(samples)
            self.last_rx = now
            self.device = device

    def read(self):
        """All readings received since the last call (oldest first)."""
        with self._lock:
            out = list(self._buf)
            self._buf.clear()
        return out

    @property
    def connected(self):
        """True while readings keep arriving (gap shorter than IMU_TIMEOUT_S)."""
        return self.last_rx is not None and time.perf_counter() - self.last_rx < C.IMU_TIMEOUT_S
