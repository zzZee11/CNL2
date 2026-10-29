"""
autonomous_navigation/config.py
--------------------------------
Configuration constants for autonomous distance navigation with obstacle avoidance.
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
DEFAULT_SPEED = 0.90          # Default cruising speed (m/s)
MAX_ALLOWED_SPEED = 1.2      # Safety clamp
MIN_MANEUVER_SPEED = 0.12     # Minimum turning/creeping speed

# ==========================================
# GRID MAPPING & A* PLANNER
# ==========================================
GRID_SIZE_X = 20.0            # 20 meters length
GRID_SIZE_Y = 10.0            # 10 meters width
GRID_RES = 0.10               # 10 cm per cell (resolution)
ORIGIN_OFFSET_X = 2.0         # Robot starts at x = 2.0 m, giving buffer behind it
ORIGIN_OFFSET_Y = 5.0         # Robot starts at y = 5.0 m (center line of width)

ROBOT_CLEARANCE_RADIUS = 0.22 # 22 cm radius padding for circular obstacle inflation
CELLS_TO_INFLATE = int(math.ceil(ROBOT_CLEARANCE_RADIUS / GRID_RES))

# ==========================================
# ARTIFICIAL POTENTIAL FIELD (APF) TUNING
# ==========================================
K_ATT = 1.4                   # Attractive force towards current waypoint
K_REP = 1.0                   # Repulsive force away from ToF obstacles
SAFE_DIST = 0.45              # Distance (meters) at which obstacles begin repelling robot
CRITICAL_STOP_DIST = 0.15     # Emergency stop distance (meters)
SENSOR_ANGLES = [
    math.pi / 4.0,            # Left sensor (+45 deg)
    0.0,                      # Middle sensor (0 deg straight forward)
    -math.pi / 4.0            # Right sensor (-45 deg)
]

# ==========================================
# TIME-OF-FLIGHT (VL53L0X) HARDWARE PINS (Pi BCM)
# ==========================================
XSHUT_LEFT = 17               # GPIO 17 (Pin 11)
XSHUT_MIDDLE = 27             # GPIO 22 (Pin 15)
XSHUT_RIGHT = 22             # GPIO 27 (Pin 13)

ADDR_LEFT = 0x30              # Readdressed I2C address
ADDR_MIDDLE = 0x31            # Readdressed I2C address
ADDR_RIGHT = 0x32             # Readdressed I2C address

TOF_MAX_RANGE_M = 2.0         # Maximum reliable ToF mapping distance (meters)
TOF_SAMPLE_INTERVAL_S = 0.05  # 20 Hz ToF polling rate

