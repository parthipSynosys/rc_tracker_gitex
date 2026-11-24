"""All tunable settings for the ArUco tracker. Edit values here, not in the code."""

# ---------------------------------------------------------------- camera
CAMERA_INDEX = 1            # which camera to open (on this PC index 0 doesn't open; 1 works)
FRAME_WIDTH = 1280          # requested capture width in px
FRAME_HEIGHT = 720          # requested capture height in px
CAMERA_FOURCC = "MJPG"      # MJPG gives full frame rate at 720p on the C930e ("" = driver default)

# ---------------------------------------------------------------- marker
ARUCO_DICT = "DICT_6X6_250"  # dictionary the marker was generated from
MARKER_ID = 0               # ID to track (None = track the first marker found)
MARKER_SIZE_CM = 8.0        # real printed side length of the black square, in cm
# ---------------------------------------------------------------- motion analysis
SCALE_EMA_ALPHA = 0.1       # EMA weight for the cm-per-pixel scale (smaller = smoother)
SPEED_WINDOW_S = 0.25       # speed = least-squares slope of position over this window (s)
ACCEL_WINDOW_S = 0.5        # accel = least-squares slope of speed over this window (s)
MIN_SPEED_SAMPLES = 3       # samples needed in the window before a speed is trusted

STOP_SPEED = 2.0            # cm/s: below this the object counts as STOPPED
STOP_HYST = 2.0             # cm/s: speed must exceed STOP_SPEED + this to leave STOPPED
ACCEL_THRESHOLD = 15.0      # cm/s^2: acceleration above this -> ACCELERATING
BRAKE_THRESHOLD = 15.0      # cm/s^2: deceleration below -this -> BRAKING
HYST_FRACTION = 0.5         # a state is kept until accel falls inside this fraction of its threshold

# ---------------------------------------------------------------- track following
TRACK_FILE = "track.json"   # where track_editor.py saves / monitor.py loads the route
OFF_TRACK_CM = 10.0         # farther than this from the line = OFF TRACK (corridor is +/- this)

# ---------------------------------------------------------------- logging / terminal
LOG_DIR = "logs"            # one run_<datetime>.csv is written here per run
CSV_FLUSH_EVERY = 30        # flush the CSV to disk every N rows
PRINT_INTERVAL_S = 5.0      # periodic terminal summary; state changes print immediately

# ---------------------------------------------------------------- dashboard
DASHBOARD_SIZE = (1920, 1080)  # default render size; the dashboard is redrawn at the real window size
GRAPH_SECONDS = 10.0        # time span shown by the two graphs
TRAIL_MAX = 600             # number of past positions kept for the map trail
ACCEL_BAR_MAX = 100.0       # cm/s^2 shown at the end of the acceleration bar
SHOW_DASHBOARD = True       # False = camera window only
FULLSCREEN = True           # False = two windows (Camera + Dashboard) side by side; True = one full-screen window (v switches view)
START_VIEW = "dashboard"       # first full-screen view: "camera", "dashboard" or "map"

# ---------------------------------------------------------------- map (dashboard background)
# "city" = animated, generated Dubai-style city built around the route (not a real map);
# "satellite" / "streets" / "dark" = real Downtown Dubai tiles (downloaded once, needs internet)
MAP_STYLE = "city"
MAP_THEME = "day"           # city look: "night", "day", "neon", "blueprint" or "minimal"
MAP_SEED = 7                # change to generate a different city layout
MAP_CENTER = (25.1972, 55.2744)  # real-map modes only: (lat, lon) map centre - Downtown Dubai / Burj Khalifa
MAP_ZOOM = 16               # tile zoom: 15 = wider area, 17 = closer streets
MAP_BRIGHTNESS = 0.75       # dim the map so the car and route stand out (1.0 = original)
MAP_IMAGE_SIZE = (2048, 1152)  # stitched map size in px (16:9 like the camera frame)
MAP_CACHE_DIR = "map_cache"  # downloaded map is stored here (delete to re-download)
MAP_LABEL = "CITY DRIVE SIMULATION"
# The whole camera frame is mapped onto the map area, so moving the car across the
# camera view drives it across this part of Dubai.

# ---------------------------------------------------------------- colours (BGR)
STATE_COLORS = {
    "ACCELERATING": (80, 200, 60),
    "CRUISING": (220, 170, 40),
    "BRAKING": (50, 50, 230),
    "STOPPED": (150, 150, 150),
    "NO SIGNAL": (60, 60, 60),
}
TRACK_COLOR = (255, 255, 255)
START_COLOR = (0, 200, 0)
FINISH_COLOR = (0, 0, 230)
CORRIDOR_COLOR = (200, 150, 40)
