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
   Logs to `logs/run_<datetime>.csv`.

## City map
- The city is generated (not a real map) around your route: `track.json` becomes a highway,
  buildings keep clear of it, and the path the car drives is paved as a road behind it.
  Animated: water shimmer, twinkling windows, ambient traffic, tower beacon.
- The whole camera view is mapped onto the screen, so driving across the camera view drives the
  car across the city.
- `config.py`: `MAP_THEME` = `day` (default), `night`, `neon`, `blueprint` or `minimal`
  (`python preview_themes.py` renders all of them into `theme_previews/`); `MAP_SEED` gives a different city layout; `MAP_STYLE = "satellite"` / `"streets"` /
  `"dark"` switches to a real Downtown Dubai map instead (downloaded once into `map_cache/`).

## Keys
| Script | Keys |
|---|---|
| track_editor | left click add - right click / `u` undo - `c` clear - `l` loop on/off - `a` add marker position - `s` save - `q` quit |
| monitor | `r` reset stats (or click RESET STATS on the dashboard) - `v` / Tab next view in full screen (camera, dashboard, map) - `f` full screen <-> windows (starts full screen on the map) - `m` map (small window, or map view in full screen) - `q` / Esc quit |

## Other
- All settings (thresholds, windows, off-track distance, colours) are in `config.py`.
- `python test_kinematics.py` runs the simulated tests (no camera needed).
- `python dashboard.py` renders `dashboard_preview.png` and `map_preview.png` from fake data.
- Only one program can use the webcam at a time: close the monitor before the editor.
