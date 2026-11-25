# ArUco robot tracker

Tracks a printed ArUco marker (DICT_6X6_250) from an overhead webcam: speed, acceleration,
motion state, route following, live dashboard, CSV log.

## Setup
    pip install opencv-contrib-python numpy

## Workflow
1. `python generate_marker.py` -> `marker_0.png`. Print it so the **black square is exactly
   `MARKER_SIZE_CM` wide** (config.py, default 8 cm) and stick it flat on top of the object.
2. `python track_editor.py` -> draw the route, press `s` to save `track.json`.
3. `python monitor.py` -> full-screen animated city dashboard: the marker is drawn as a car driving
   through a generated Dubai-style city, with a small telemetry panel bottom-right.
   Logs to `logs/camera/run_<datetime>.csv`.

## City map
- The city is generated (not a real map) around your route: `track.json` becomes a highway,
  buildings keep clear of it, and the path the car drives is paved as a road behind it.
  Animated: water shimmer, twinkling windows, ambient traffic, tower beacon.
- The whole camera view is mapped onto the screen, so driving across the camera view drives the
  car across the city.
- `config.py`: `MAP_THEME` = `day` (default), `night`, `neon`, `blueprint` or `minimal`
  (`python preview_themes.py` renders all of them into `theme_previews/`); `MAP_SEED` gives a different city layout; `MAP_STYLE = "satellite"` / `"streets"` /
  `"dark"` switches to a real Downtown Dubai map instead (downloaded once into `map_cache/`).

## Motion sensor (optional): ESP32 + MPU-6050
The camera gives **position and speed**. **Accelerating / braking / turning come only from the
sensor**, and there is no automatic switch to the camera: without the sensor the state shows
**NO SENSOR** and the dashboard shows `(no sensor)` in orange (`(sensor)` in green when it is
working). STOPPED still comes from the camera speed. To use the camera for everything instead
(no sensor), set `MOTION_SOURCE = "camera"` in `config.py`.

**1. Wiring** (ESP32 unplugged while wiring)

| MPU-6050 | ESP32 |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SCL | GPIO 22 (D22) |
| SDA | GPIO 21 (D21) |

Leave XDA, XCL, AD0 and INT empty. Stick the sensor **flat and firm** on the car with the
**X arrow pointing forward**.

**2. Arduino IDE (one time)**: install Arduino IDE 2. Under File -> Preferences -> "Additional
boards manager URLs", add `https://espressif.github.io/arduino-esp32/package_esp32_index.json`.
Then install **esp32 by Espressif Systems** (Boards Manager) and **Adafruit MPU6050** (Library
Manager, "Install All").

**3. Upload**: open `sensor/firmware/esp32_mpu6050/esp32_mpu6050.ino` and type your WiFi name and
password at the top (2.4 GHz WiFi, the same network as this PC). Select Tools -> Board ->
**ESP32 Dev Module** and Tools -> Port -> your COM port, then click Upload (hold **BOOT** if it
hangs on "Connecting..."). In Tools -> Serial Monitor at 115200 baud you should see "WiFi
connected" and then `forward ... turn ...` lines. No COM port? Install the CP210x or CH340 USB
driver, whichever chip sits next to the USB socket.

**4. Run**: power the ESP32 (USB power bank on the car) and **keep the car still for 3 s**
(calibration). Then run `python monitor.py` as usual. The first time, Windows Firewall asks
about Python: allow it on **private networks**, otherwise the readings are blocked.

**Tuning** (`config.py`, section "motion sensor"): accelerating and braking swapped -> set
`IMU_FORWARD_SIGN = -1`. Left and right swapped -> set `IMU_YAW_SIGN = -1`. Bumps cause false
braking -> raise `IMU_ACCEL_THRESHOLD` / `IMU_BRAKE_THRESHOLD` or `IMU_FILTER_S`. Turning too
sensitive -> raise `TURN_THRESHOLD_DPS`.
The bottom-right panel has a **SENSOR (MPU-6050)** section: moving, turning, angle, acceleration,
speed and distance. Angle, speed and distance are added up from the sensor over time, so they drift
and are marked **(est.)**. The speed estimate goes back to 0 whenever the car stands still. If MOVING says YES
while the car is still, raise `IMU_MOVING_STD`. Sensor readings are logged to `logs/imu/imu_<datetime>.csv`.

## Keys
| Script | Keys |
|---|---|
| track_editor | left click add - right click / `u` undo - `c` clear - `l` loop on/off - `a` add marker position - `s` save - `q` quit |
| monitor | `r` reset stats (or click RESET STATS) - `f` full screen <-> windows (starts in windows: **Map**, **Data** and **Camera** each in their own window, drag them anywhere, e.g. the map onto a second screen) - `v` / Tab next view in full screen (camera, dashboard, map, data) - `m` map view in full screen - `q` / Esc quit |

## Other
- All settings (thresholds, windows, off-track distance, colours) are in `config.py`.
- `python tests/test_kinematics.py` and `python tests/test_imu.py` run the simulated tests (no camera or sensor needed).
- `python dashboard.py` renders `dashboard_preview.png` and `map_preview.png` from fake data.
- Only one program can use the webcam at a time: close the monitor before the editor.
