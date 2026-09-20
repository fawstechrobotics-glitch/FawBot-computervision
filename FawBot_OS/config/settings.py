"""Central application configuration and physical robot constants."""
import os

# --- BASE DIRECTORIES ---
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MISSIONS_DIR = os.path.join(BASE_DIR, "missions")
LOGS_DIR = os.path.join(BASE_DIR, "logs")

# Ensure required runtime directories exist
os.makedirs(MISSIONS_DIR, exist_ok=True)
os.makedirs(LOGS_DIR, exist_ok=True)

# --- ROBOT NETWORK & UDP COMMUNICATION ---
ROBOT_HOST = "fawbot.local"
UDP_PORT = 8888
SOCKET_TIMEOUT = 0.5  # seconds

# --- ENVIRONMENT BOUNDARY DEFAULTS ---
ENV_WIDTH_CM = 150.0
ENV_HEIGHT_CM = 150.0

# --- ROBOT PHYSICAL DIMENSIONS (CHASSIS) ---
ROBOT_LENGTH_CM = 12.0
ROBOT_WIDTH_CM = 9.0
ROBOT_SAFETY_MARGIN_CM = 2.0

# --- HARDWARE SPEEDS & KINEMATICS ---
WHEEL_BASE_CM = 10.0         # Distance between driving wheels in cm
STEPS_PER_CM = 256.81        # Stepper motor steps per centimeter
STEP_DELAY_US = 850.0       # Delay per step in microseconds

# --- TEACH / RECORD MODE THRESHOLDS ---
PATH_RECORD_DISTANCE_THRESHOLD = 1.0   # Min distance moved (cm) before capturing point
PATH_RECORD_ANGLE_THRESHOLD = 2.0      # Min heading change (deg) before capturing point
PATH_RECORD_TIME_INTERVAL = 0.1        # Min time elapsed (s) between points if moved

# --- CAD / MAP RENDERING ---
INITIAL_GRID_STEP_CM = 5.0
ANIMATION_FPS = 30
ANIMATION_INTERVAL_MS = 33  # ~30 FPS

# --- UI CONSTANTS ---
WINDOW_TITLE = "FawBot OS | Mission Path Planning & Control"
DEFAULT_WINDOW_WIDTH = 1280
DEFAULT_WINDOW_HEIGHT = 800
