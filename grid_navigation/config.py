"""
grid_navigation/config.py
-------------------------
Configuration parameters for clean grid-based autonomous navigation.
"""

import os
import math

# ==========================================
# ROBOT PHYSICAL DIMENSIONS & ODOMETRY
# ==========================================
WHEEL_RADIUS_M = 0.0215       # 2.15 cm wheel radius
WHEEL_BASE_M = 0.134          # 13.4 cm track width between left and right wheels
ROBOT_RADIUS_M = 0.14         # 14.0 cm circular safety padding (matches body width)
TICKS_PER_REV = 720.0         # Encoder ticks per single wheel revolution

# ==========================================
# 2D DISCRETE OCCUPANCY GRID SETUP
# ==========================================
GRID_SIZE_X_M = 20.0          # 20 meters length (-10.0m to +10.0m)
GRID_SIZE_Y_M = 20.0          # 20 meters width (-10.0m to +10.0m)
GRID_RES_M = 0.10             # 10 cm per cell resolution
ORIGIN_OFFSET_X = 10.0        # (0, 0) world is at cell index (100, 100)
ORIGIN_OFFSET_Y = 10.0        # (0, 0) world is at cell index (100, 100)

GRID_DIM_X = int(round(GRID_SIZE_X_M / GRID_RES_M))  # 200 cells
GRID_DIM_Y = int(round(GRID_SIZE_Y_M / GRID_RES_M))  # 200 cells
INFLATION_CELLS = int(math.ceil(ROBOT_RADIUS_M / GRID_RES_M))  # 2 cells padding

# ==========================================
# MOTION SPEEDS & CONTROLLER LIMITS
# ==========================================
CRUISE_SPEED = 0.35           # Nominal cruising speed (m/s)
MIN_CREEP_SPEED = 0.04        # High-precision terminal creeping speed (m/s)
PIVOT_SPEED = 0.20            # In-place rotation speed (m/s)
MAX_SPEED = 0.80              # Absolute velocity clamp (m/s)

# ==========================================
# PRECISION ARRIVAL & DOCKING TOLERANCES
# ==========================================
WAYPOINT_TOLERANCE_M = 0.15        # Waypoint pop distance when following path
TERMINAL_GOAL_TOLERANCE_M = 0.025  # High-precision landing distance tolerance (2.5 cm)
FINAL_HEADING_TOLERANCE_DEG = 3.0  # Final orientation alignment tolerance (degrees)
KP_HEADING = 1.8                   # Proportional heading steering gain
KD_HEADING = 0.12                  # Gyro rate damping gain

# ==========================================
# OBSTACLE DETECTION & SENSOR TUNING
# ==========================================
OBSTACLE_DETECT_DIST_M = 0.40      # Distance at which ToF obstacles are mapped into grid
CRITICAL_STOP_DIST_M = 0.14        # Emergency brake distance (stops before physical contact)
TOF_MIN_VALID_M = 0.08             # Reject < 8cm crosstalk
TOF_MAX_VALID_M = 1.80             # Reject > 1.8m weak returns

TOF_ANGLES = [
    math.pi / 4.0,                 # Left sensor (+45 deg)
    0.0,                           # Middle sensor (0 deg straight forward)
    -math.pi / 4.0                 # Right sensor (-45 deg)
]

# ==========================================
# HARDWARE PORTS & PINS
# ==========================================
SERIAL_PORT = "/dev/serial0"
SERIAL_BAUD = 115200
TX_RATE_HZ = 20.0
WATCHDOG_TIMEOUT_MS = 500

IMU_I2C_BUS = 1
IMU_I2C_ADDR = 0x68

XSHUT_LEFT = 17
XSHUT_MIDDLE = 22
XSHUT_RIGHT = 27

# ==========================================
# OUTPUT FILES
# ==========================================
DEFAULT_PLOT_PATH = "grid_navigation/grid_navigation_result.png"

