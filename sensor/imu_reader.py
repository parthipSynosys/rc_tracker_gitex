"""Receives MPU-6050 readings that the ESP32 broadcasts over WiFi (UDP).

Runs in a background thread so the camera loop never waits for it. If no ESP32 is sending,
nothing happens: `connected` stays False and the monitor keeps using the camera only.
Message format (one text line per reading, see sensor/firmware/esp32_mpu6050):
    IMU,<seq>,<millis>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>
"""
import socket
import threading
import time
from collections import deque, namedtuple

import config as C

# t_dev: ESP32 clock (s) | ax..az: m/s^2 | gx..gz: deg/s | t_pc: PC time.perf_counter() on arrival
Sample = namedtuple("Sample", "t_dev ax ay az gx gy gz t_pc")


def parse(line, t_pc=0.0):
    """One text line -> Sample, or None if it is not a valid IMU line."""
    parts = line.strip().split(",")
    if len(parts) != 9 or parts[0] != "IMU":
        return None
    try:
        ms = int(parts[2])
        ax, ay, az, gx, gy, gz = (float(v) for v in parts[3:])
    except ValueError:
        return None
    return Sample(ms / 1000.0, ax, ay, az, gx, gy, gz, t_pc)


class ImuReader:
    def __init__(self, port=None):
        self.port = C.IMU_PORT if port is None else port
        self.device = None       # IP address of the ESP32 once it has sent something
        self.count = 0           # readings received
        self.last_rx = None      # perf_counter() of the last reading
        self._buf = deque(maxlen=2000)
        self._lock = threading.Lock()
        self._sock = None
        self._thread = None
        self._running = False

    def start(self):
        """Start listening. Returns False (and the program carries on) if the port is unavailable."""
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
        self._thread = threading.Thread(target=self._run, name="imu-reader", daemon=True)
        self._thread.start()
        return True

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def _run(self):
        while self._running:
            try:
                data, addr = self._sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            now = time.perf_counter()
            samples = [s for s in (parse(l, now) for l in data.decode("ascii", "ignore").splitlines()) if s]
            if not samples:
                continue
            with self._lock:
                self._buf.extend(samples)
                self.count += len(samples)
                self.last_rx = now
                self.device = addr[0]

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
