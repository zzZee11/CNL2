"""
autonomous_navigation/config.py
--------------------------------
Configuration constants for autonomous distance & Cartesian navigation with obstacle avoidance.
Matches the ESP32 firmware Config.h and the Raspberry Pi hardware wiring.
"""

import os
import json
import time
import math

# ==========================================
# SERIAL COMMUNICATION (ESP32-S3 UART)
# ==========================================
SERIAL_PORT = "/dev/serial0"
SERIAL_BAUD = 115200
TX_RATE_HZ = 20.0             # Continuous command transmission rate (50 ms)
WATCHDOG_TIMEOUT_MS = 500     # ESP32 auto-stops if link lost for > 500 ms

# ==========================================
# ROBOT PHYSICAL DIMENSIONS & ODOMETRY
# ==========================================
# Base physical specifications (factory defaults)
DEFAULT_WHEEL_RADIUS_M = 0.0215       # 2.15 cm radius default
DEFAULT_WHEEL_BASE_M = 0.134          # 13.4 cm track width default
DEFAULT_TICKS_PER_REV = 720.0         # Measured encoder counts per turn

WHEEL_RADIUS_M = DEFAULT_WHEEL_RADIUS_M
WHEEL_BASE_M = DEFAULT_WHEEL_BASE_M
ROBOT_LENGTH_M = 0.18                 # 18.0 cm physical body length
ROBOT_WIDTH_M = 0.134                 # 13.4 cm physical body width
TICKS_PER_REV = DEFAULT_TICKS_PER_REV

# Calibration Persistence (Solution 5)
CALIBRATION_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibration.json")

def load_calibration():
    """Loads calibrated physical dimensions from calibration.json if present."""
    global WHEEL_RADIUS_M, WHEEL_BASE_M, TICKS_PER_REV, ROBOT_WIDTH_M
    if os.path.exists(CALIBRATION_FILE):
        try:
            with open(CALIBRATION_FILE, "r") as f:
                data = json.load(f)
                if "wheel_radius_m" in data:
                    WHEEL_RADIUS_M = float(data["wheel_radius_m"])
                if "wheel_base_m" in data:
                    WHEEL_BASE_M = float(data["wheel_base_m"])
                    ROBOT_WIDTH_M = WHEEL_BASE_M
                if "ticks_per_rev" in data:
                    TICKS_PER_REV = float(data["ticks_per_rev"])
        except Exception:
            pass

def save_calibration(wheel_radius_m: float, wheel_base_m: float, ticks_per_rev: float = None):
    """Saves calibrated dimensions to calibration.json and updates active runtime constants."""
    global WHEEL_RADIUS_M, WHEEL_BASE_M, TICKS_PER_REV, ROBOT_WIDTH_M
    WHEEL_RADIUS_M = round(float(wheel_radius_m), 5)
    WHEEL_BASE_M = round(float(wheel_base_m), 5)
    ROBOT_WIDTH_M = WHEEL_BASE_M
    if ticks_per_rev is not None:
        TICKS_PER_REV = round(float(ticks_per_rev), 2)
    data = {
        "wheel_radius_m": WHEEL_RADIUS_M,
        "wheel_base_m": WHEEL_BASE_M,
        "ticks_per_rev": TICKS_PER_REV,
        "calibrated_at": time.strftime("%Y-%m-%d %H:%M:%S")
    }
    with open(CALIBRATION_FILE, "w") as f:
        json.dump(data, f, indent=2)
    return data

def reset_calibration():
    """Resets dimensions to factory defaults and removes calibration.json."""
    global WHEEL_RADIUS_M, WHEEL_BASE_M, TICKS_PER_REV, ROBOT_WIDTH_M
    WHEEL_RADIUS_M = DEFAULT_WHEEL_RADIUS_M
    WHEEL_BASE_M = DEFAULT_WHEEL_BASE_M
    ROBOT_WIDTH_M = WHEEL_BASE_M
    TICKS_PER_REV = DEFAULT_TICKS_PER_REV
    if os.path.exists(CALIBRATION_FILE):
        try:
            os.remove(CALIBRATION_FILE)
        except OSError:
            pass

load_calibration()

# ==========================================
# STARTUP PUSH & DEFAULT SPEEDS
# ==========================================
# Breakaway torque boost to overcome L298N & gearbox static friction
STARTUP_PUSH_SPEED = 0.45     # Gentle breakaway kick demand (m/s)
STARTUP_PUSH_DURATION_S = 0.15 # 150 ms startup kick (forward only)
DEFAULT_SPEED = 0.40          # Default cruising speed (m/s)
MAX_ALLOWED_SPEED = 1.0       # Safety clamp
MIN_MANEUVER_SPEED = 0.1     # Minimum turning/creeping speed

# Acceleration & Deceleration Slew-Rate Limits (Smooth Motion)
MAX_ACCEL_MPS2 = 0.80         # Maximum linear acceleration (m/s^2)
MAX_DECEL_MPS2 = 1.20         # Maximum linear deceleration (m/s^2)

# ==========================================
# 2D CARTESIAN PLANE & A* GRID MAPPING
# ==========================================
# Symmetric 30m x 30m field with (0, 0) at center (X in [-15, +15], Y in [-15, +15])
GRID_SIZE_X = 30.0            # 30 meters length (-15m to +15m)
GRID_SIZE_Y = 30.0            # 30 meters width (-15m to +15m)
GRID_RES = 0.10               # 10 cm per cell (resolution)
ORIGIN_OFFSET_X = 15.0        # (0, 0) starts at cell index (150, 150)
ORIGIN_OFFSET_Y = 15.0        # (0, 0) starts at cell index (150, 150)

ROBOT_CLEARANCE_RADIUS = 0.14 # 14 cm radius padding (matches 13.4 cm robot width with 7 cm margin)
CELLS_TO_INFLATE = int(math.ceil(ROBOT_CLEARANCE_RADIUS / GRID_RES))

# Default path to save trajectory and obstacle map image
DEFAULT_PLOT_PATH = "autonomous_navigation/navigation_result.png"

# ==========================================
# ARTIFICIAL POTENTIAL FIELD (APF) TUNING
# ==========================================
K_ATT = 1.4                   # Attractive force towards current waypoint
K_REP = 0.4                  # Repulsive force away from ToF obstacles (tuned to prevent spikes)
MAX_REP_FORCE = 1.0           # Saturation clamp on repulsive force to prevent erratic twitches
SAFE_DIST = 0.35              # Distance (meters) at which obstacles begin repelling robot
CRITICAL_STOP_DIST = 0.15     # Emergency stop distance (meters) - stops before contact
CRITICAL_PERSIST_TICKS = 2    # Consecutive cycles (100ms) required to confirm critical emergency hazard
SENSOR_ANGLES = [
    math.pi / 4.0,            # Left sensor (+45 deg)
    0.0,                      # Middle sensor (0 deg straight forward)
    -math.pi / 4.0            # Right sensor (-45 deg)
]

# In-Place Pivot Heading Thresholds (with Hysteresis)
PIVOT_ENTER_RAD = 0.85        # ~49 deg: enter in-place pivot if heading error exceeds this
PIVOT_EXIT_RAD = 0.35         # ~20 deg: exit in-place pivot only when aligned within this

# Heading Lock & Course Correction (PD Control)
KP_HEADING_NAV = 2.2          # Proportional angular steering gain
KD_HEADING_NAV = 0.14         # Derivative rate damping gain to prevent heading overshoots
HEADING_DEADBAND_RAD = 0.035  # ~2.0 deg deadband: drive straight when aligned

# Goal Arrival Precision, Terminal Docking & Pure Pursuit Lookahead
GOAL_TOLERANCE_M = 0.08       # Intermediate waypoint / standard arrival distance tolerance (meters)
TERMINAL_GOAL_TOLERANCE_M = 0.025 # High-precision terminal arrival distance tolerance (2.5 cm)
TERMINAL_DECEL_DIST_M = 0.35  # Distance (meters) where smooth multi-stage deceleration begins
TERMINAL_CREEP_DIST_M = 0.12  # Distance (meters) where terminal precision creeping begins
MIN_TERMINAL_CREEP_SPEED = 0.04 # Ultra-slow creeping speed (m/s) to eliminate inertia rollout
BRAKE_PULSE_DURATION_S = 0.035 # Duration (seconds) of active reverse counter-torque pulse
BRAKE_PULSE_SPEED = 0.18      # Speed demand (m/s) for active counter-torque braking pulse
LOOKAHEAD_DIST_M = 0.35       # Pure pursuit maximum lookahead distance (meters) along waypoint path
MIN_LOOKAHEAD_DIST_M = 0.08   # Pure pursuit minimum lookahead distance (meters) near destination
FINAL_HEADING_TOLERANCE_DEG = 3.0 # Final orientation alignment tolerance in degrees
KP_ALIGN_TURN = 0.85          # Proportional gain for in-place orientation alignment
KD_ALIGN_TURN = 0.05          # Gyro derivative damping gain for alignment
MIN_ALIGN_SPEED = 0.18        # Minimum pivot speed to break static friction
MAX_ALIGN_SPEED = 0.35        # Maximum pivot speed for final alignment

# Critical Obstacle Mapping & Noise Rejection Filter
TOF_MIN_VALID_M = 0.08        # Reject < 8cm readings (sensor casing crosstalk / lens smudges)
TOF_MAX_VALID_M = 1.80        # Reject > 1.8m readings (low indoor ToF SNR / timeout)
TOF_MEDIAN_WINDOW = 5         # Rolling median window (5 samples / 250ms) to discard single-tick spikes
OBSTACLE_MAP_MAX_DIST_M = 0.70 # Focus mapping on clean near-field (<=70cm) to prevent 25° floor glare
OBSTACLE_PERSISTENCE_COUNT = 5 # Number of spatial consensus hits required to confirm a real obstacle
OBSTACLE_CONFIRM_RADIUS_M = 0.12 # World-frame clustering radius (12cm) for spatial consensus
MIN_OBSTACLE_SPACING_M = 0.20  # Minimum 20 cm spacing to prevent duplicate dots along the same wall

# ==========================================
# TIME-OF-FLIGHT (VL53L0X) HARDWARE PINS (Pi BCM)
# ==========================================
XSHUT_LEFT = 17               # GPIO 17 (Pin 11)
XSHUT_MIDDLE = 22             # GPIO 27 (Pin 13)
XSHUT_RIGHT = 27              # GPIO 22 (Pin 15)

ADDR_LEFT = 0x30              # Readdressed I2C address
ADDR_MIDDLE = 0x31            # Readdressed I2C address
ADDR_RIGHT = 0x32             # Readdressed I2C address

TOF_MAX_RANGE_M = 2.0         # Maximum reliable ToF mapping distance (meters)
TOF_SAMPLE_INTERVAL_S = 0.05  # 20 Hz ToF polling rate

# ==========================================
# INERTIAL MEASUREMENT UNIT (MPU-6050)
# ==========================================
IMU_I2C_BUS = 1                  # Standard Raspberry Pi I2C bus 1
IMU_I2C_ADDR = 0x68              # Default MPU-6050 address (AD0 to GND)
IMU_SAMPLE_RATE_HZ = 50.0        # 50 Hz internal polling rate
IMU_DEADBAND_DPS = 0.25          # Gyro deadband filter to eliminate stationary creep
IMU_COLLISION_THRESHOLD_G = 1.8  # Horizontal acceleration spike threshold (g) for bump detection

