"""
autonomous_navigation/config.py
--------------------------------
Configuration constants for autonomous distance & Cartesian navigation with obstacle avoidance.
Matches the ESP32 firmware Config.h and the Raspberry Pi hardware wiring.
"""

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
# Must match RobotController/Config.h
WHEEL_RADIUS_M = 0.0215       # 2.15 cm radius
WHEEL_BASE_M = 0.134          # 13.4 cm track width between left and right wheels
TICKS_PER_REV = 720.0         # Measured encoder counts per single wheel turn (Config.h)

# ==========================================
# STARTUP PUSH & DEFAULT SPEEDS
# ==========================================
# Breakaway torque boost to overcome L298N & gearbox static friction
STARTUP_PUSH_SPEED = 0.85     # Max power speed demand (m/s) -> saturates ESP32 PID output to 255 PWM
STARTUP_PUSH_DURATION_S = 0.18 # 180 ms startup kick
DEFAULT_SPEED = 0.60          # Default cruising speed (m/s)
MAX_ALLOWED_SPEED = 1.0       # Safety clamp
MIN_MANEUVER_SPEED = 0.12     # Minimum turning/creeping speed

# ==========================================
# 2D CARTESIAN PLANE & A* GRID MAPPING
# ==========================================
# Symmetric 30m x 30m field with (0, 0) at center (X in [-15, +15], Y in [-15, +15])
GRID_SIZE_X = 30.0            # 30 meters length (-15m to +15m)
GRID_SIZE_Y = 30.0            # 30 meters width (-15m to +15m)
GRID_RES = 0.10               # 10 cm per cell (resolution)
ORIGIN_OFFSET_X = 15.0        # (0, 0) starts at cell index (150, 150)
ORIGIN_OFFSET_Y = 15.0        # (0, 0) starts at cell index (150, 150)

ROBOT_CLEARANCE_RADIUS = 0.22 # 22 cm radius padding for circular obstacle inflation
CELLS_TO_INFLATE = int(math.ceil(ROBOT_CLEARANCE_RADIUS / GRID_RES))

# Default path to save trajectory and obstacle map image
DEFAULT_PLOT_PATH = "autonomous_navigation/navigation_result.png"

# ==========================================
# ARTIFICIAL POTENTIAL FIELD (APF) TUNING
# ==========================================
K_ATT = 1.4                   # Attractive force towards current waypoint
K_REP = 1.0                   # Repulsive force away from ToF obstacles
SAFE_DIST = 0.55              # Distance (meters) at which obstacles begin repelling robot
CRITICAL_STOP_DIST = 0.18     # Emergency stop distance (meters)
SENSOR_ANGLES = [
    math.pi / 4.0,            # Left sensor (+45 deg)
    0.0,                      # Middle sensor (0 deg straight forward)
    -math.pi / 4.0            # Right sensor (-45 deg)
]

# ==========================================
# TIME-OF-FLIGHT (VL53L0X) HARDWARE PINS (Pi BCM)
# ==========================================
XSHUT_LEFT = 17               # GPIO 17 (Pin 11)
XSHUT_MIDDLE = 27             # GPIO 27 (Pin 13)
XSHUT_RIGHT = 22              # GPIO 22 (Pin 15)

ADDR_LEFT = 0x30              # Readdressed I2C address
ADDR_MIDDLE = 0x31            # Readdressed I2C address
ADDR_RIGHT = 0x32             # Readdressed I2C address

TOF_MAX_RANGE_M = 2.0         # Maximum reliable ToF mapping distance (meters)
TOF_SAMPLE_INTERVAL_S = 0.05  # 20 Hz ToF polling rate
