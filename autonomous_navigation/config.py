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
STARTUP_PUSH_SPEED = 0.30     # Gentle breakaway kick demand (m/s)
STARTUP_PUSH_DURATION_S = 0.15 # 150 ms startup kick (forward only)
DEFAULT_SPEED = 0.30          # Default cruising speed (m/s)
MAX_ALLOWED_SPEED = 0.50       # Safety clamp
MIN_MANEUVER_SPEED = 0.1     # Minimum turning/creeping speed

# Acceleration & Deceleration Slew-Rate Limits (Smooth Motion)
MAX_ACCEL_MPS2 = 0.50         # Maximum linear acceleration (m/s^2)
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

ROBOT_CLEARANCE_RADIUS = 0.15 # 15 cm radius padding for circular obstacle inflation
CELLS_TO_INFLATE = int(math.ceil(ROBOT_CLEARANCE_RADIUS / GRID_RES))

# Default path to save trajectory and obstacle map image
DEFAULT_PLOT_PATH = "autonomous_navigation/navigation_result.png"

# ==========================================
# ARTIFICIAL POTENTIAL FIELD (APF) TUNING
# ==========================================
K_ATT = 1.4                   # Attractive force towards current waypoint
K_REP = 0.35                  # Repulsive force away from ToF obstacles (tuned to prevent spikes)
MAX_REP_FORCE = 1.0           # Saturation clamp on repulsive force to prevent erratic twitches
SAFE_DIST = 0.45              # Local APF obstacle repulsion distance (0.45m prevents corridor ping-pong)
CRITICAL_STOP_DIST = 0.18     # Emergency brake distance (meters) to trigger halt and back-off
BACKOFF_DIST_M = 0.10         # Controlled reverse back-off distance (meters) to gain turning clearance
BACKOFF_SPEED_MPS = 0.14      # Reverse speed (m/s) during controlled back-off
SENSOR_ANGLES = [
    math.pi / 4.0,            # Left sensor (+45 deg)
    0.0,                      # Middle sensor (0 deg straight forward)
    -math.pi / 4.0            # Right sensor (-45 deg)
]

# In-Place Pivot Heading Thresholds & Speed Limits (Calm Turning)
PIVOT_ENTER_RAD = 0.85        # ~49 deg: enter in-place pivot if heading error exceeds this
PIVOT_EXIT_RAD = 0.35         # ~20 deg: exit in-place pivot only when aligned within this
MAX_PIVOT_SPEED = 0.25        # Capped pivot speed (m/s) for calm, controlled turns (default: 0.18 m/s)
MIN_PIVOT_SPEED = 0.15        # Minimum pivot speed (m/s) to overcome floor friction

# Heading Lock & Course Correction (PD Control)
KP_HEADING_NAV = 1.0          # Proportional angular steering gain (softened to eliminate limit-cycle weaving)
KD_HEADING_NAV = 0.18         # Derivative rate damping gain to prevent heading overshoots
HEADING_DEADBAND_RAD = 0.035  # ~2.0 deg deadband: drive straight when aligned

# Goal Arrival Precision & Pure Pursuit Lookahead
GOAL_TOLERANCE_M = 0.08       # Target arrival distance tolerance in meters
LOOKAHEAD_DIST_M = 0.55       # Pure pursuit lookahead distance (meters) along waypoint path (smooth path tracking)

# Obstacle Mapping Persistence & Distance Filter
OBSTACLE_PERSISTENCE_COUNT = 10     # Standard persistence filter for peripheral/distant obstacles (>0.35m)
OBSTACLE_PERSISTENCE_CRITICAL = 3  # Fast-track persistence for direct close hazards (<0.35m)
CRITICAL_CLOSE_DIST = 0.35          # Threshold (meters) to switch to fast-track persistence
OBSTACLE_MAP_MAX_DIST_M = 1.00     # Detect and map obstacles within 1.0 meter far

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
TOF_TIMING_BUDGET_US = 20000  # 20 ms timing budget (High-Speed mode: 20000 us vs default 33000 us)
TOF_SAMPLE_INTERVAL_S = 0.02  # 50 Hz target ToF polling loop rate (increased from 20 Hz / 0.05s)

# ==========================================
# INERTIAL MEASUREMENT UNIT (MPU-6050)
# ==========================================
IMU_I2C_BUS = 1                  # Standard Raspberry Pi I2C bus 1
IMU_I2C_ADDR = 0x68              # Default MPU-6050 address (AD0 to GND)
IMU_SAMPLE_RATE_HZ = 50.0        # 50 Hz internal polling rate
IMU_DEADBAND_DPS = 0.25          # Gyro deadband filter to eliminate stationary creep
IMU_COLLISION_THRESHOLD_G = 1.8  # Horizontal acceleration spike threshold (g) for bump detection

