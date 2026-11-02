# Autonomous 2D Cartesian & Obstacle Avoidance Navigator

A complete, production-grade autonomous navigation system for the differential-drive robot car combining:
- **3× VL53L0X Time-of-Flight (ToF) Sensors**: Left (+45°), Middle (0°), and Right (−45°) on Raspberry Pi I2C with hardware-level median filtering and floor-glare rejection.
- **MPU-6050 6-DOF IMU**: Gyroscopic heading integration and spatial yaw-rate damping for straight line tracking and closed-loop destination orientation alignment.
- **2D Cartesian Grid Planner (A\*)**: 30m × 30m occupancy grid (10 cm resolution) with circular obstacle inflation clearance and dynamic route replanning.
- **Artificial Potential Field (APF) Local Planner**: Real-time micro-evasion vector field that smoothly guides the robot around obstacles without stopping.
- **Exact-Path Retrace Return to Start**: Pivots in-place at the goal and retraces the exact cleared trajectory forward back to `(0, 0)`, restoring the initial starting orientation ($0.0^\circ$).
- **Multi-Cycle Patrol / Shuttle**: Autonomous $N$-cycle back-and-forth missions along established safe corridors, executing in-place pivot turns at each endpoint.
- **Continuous Obstacle Mapping & Swept-Path Plotting**: High-resolution visualization (`navigation_result.png`) displaying continuous barrier chains, physical robot dimensions ($18\text{ cm} \times 13.4\text{ cm}$), swept wheel track corridors, and companion telemetry JSON.
- **ESP32-S3 Serial Closed-Loop Velocity Control**: 20 Hz heartbeat stream over `/dev/serial0` (115200 baud) with stiction breakaway kick and encoder odometry.

---

## Architecture Overview

```text
┌────────────────────────────────────────────────────────────────────────┐
│                        RASPBERRY PI 4B                                 │
│                                                                        │
│  autonomous_navigation/main.py                                         │
│    ├── CLI Interface ('--goal 1.0 1.0 90 --patrol 3')                  │
│    └── Interactive Shell ('nav> goto 1 2 90 deg --return')             │
│                                                                        │
│  navigator.py (AutonomousNavigator)                                    │
│    ├── Odometry tracker (s, x, y, theta) fused with MPU-6050 IMU        │
│    ├── Spatial Consensus Obstacle Tracker (eliminates floor glare)     │
│    ├── A* global pathfinder & dynamic replanner                        │
│    ├── Pure Pursuit Waypoint Lookahead (35 cm)                         │
│    ├── Return-to-Start & Multi-Cycle Patrol Engine                     │
│    └── APF reactive vector evasion (F_att + F_rep)                     │
│              │                       │               │                 │
│  sensors.py (ToF)      imu.py (MPU6050)  serial_comm.py  plotter.py   │
│    ├── Median filter     ├── 50 Hz Gyro   ├── 20 Hz TX    ├── Swept Cor│
│    ├── XSHUT boot        └── Damping      ├── Odometry    └── Chains   │
└──────────────┬───────────────────────┬───────────────┬─────────────────┘
               │ I2C Bus 1             │               │ UART (/dev/serial0)
               ▼                       ▼               ▼
        3× VL53L0X Sensors         MPU-6050 IMU     ESP32-S3
        (L: 45°, M: 0°, R: -45°)   (0x68 on I2C-1)  (WheelSpeedPID + L298N)
```

---

## Hardware Pinout Reference

### Raspberry Pi 4B $\rightarrow$ 3× VL53L0X Time-of-Flight Sensors & MPU-6050 IMU (I2C)

| Signal | Raspberry Pi Pin | Hardware Pin | Purpose |
|---|---|---|---|
| I2C SCL | **GPIO 3** (Physical Pin 5) | SCL (shared) | Common I2C clock bus |
| I2C SDA | **GPIO 2** (Physical Pin 3) | SDA (shared) | Common I2C data bus |
| Power | **3.3V** (Physical Pin 1) | VCC / VIN | 3.3V logic power supply |
| Ground | **GND** (Physical Pin 6/9) | GND | Common Ground |
| Left XSHUT | **GPIO 17** (Physical Pin 11) | XSHUT (Left Sensor) | Readdresses Left to `0x30` |
| Middle XSHUT | **GPIO 22** (Physical Pin 15) | XSHUT (Mid Sensor) | Readdresses Middle to `0x31` |
| Right XSHUT | **GPIO 27** (Physical Pin 13) | XSHUT (Right Sensor) | Readdresses Right to `0x32` |
| MPU-6050 AD0 | **GND** (Physical Pin 34) | AD0 | Hardware I2C address `0x68` |

### Raspberry Pi 4B $\rightarrow$ ESP32-S3 UART Connection

| Raspberry Pi Pin | Function | ESP32-S3 Pin | Function |
|---|---|---|---|
| **Pin 8** (GPIO 14) | UART0 TXD | **GPIO 18** | ESP32 UART RX |
| **Pin 10** (GPIO 15) | UART0 RXD | **GPIO 17** | ESP32 UART TX |
| **Pin 6 or 9** | GND | **GND** | Common Ground |

---

## How to Run

### Option 1: Interactive Terminal Shell
Launch the interactive shell:
```bash
python3 -m autonomous_navigation.main
```
or in desktop simulation mode (without hardware):
```bash
python3 -m autonomous_navigation.main --mock
```

#### Shell Commands:
* **Cartesian Navigation (`goto` / `navigate`)**:
  ```text
  nav> goto 1.5 2.0                     # Move to (1.5m, 2.0m) avoiding obstacles
  nav> goto 1.0 1.0 90 deg              # Move to (1.0m, 1.0m) and align to +90°
  nav> goto 1.0 1.0 @ 180               # Move to (1.0m, 1.0m) and face backward (180°)
  nav> goto (-2.5, 3.0) 0.5 m/s 45 deg  # Move with custom cruise speed and heading
  nav> goto 1.0 1.0 --return            # Navigate to goal, then return to start
  nav> goto 1.0 1.0 90 deg roundtrip    # Roundtrip with target alignment
  ```

* **Exact-Path Return to Start (`return` / `back` / `home` / `retrace`)**:
  ```text
  nav> return                           # Pivots in-place, follows exact path forward to (0,0), aligns to 0°
  ```

* **Multi-Cycle Patrol (`patrol` / `shuttle`)**:
  ```text
  nav> patrol 1.0 1.0 3                 # Shuttle back and forth 3 times along the cleared path
  nav> patrol 1.2 0.8 5 90 deg          # 5 cycles with 90° heading at destination
  nav> shuttle 2.0 0.0 2 0.5 m/s        # 2 cycles at 0.5 m/s cruise speed
  ```

* **Straight Distance & Pivot Turn**:
  ```text
  nav> move 2.5 m 0.4                   # Move straight forward 2.5 meters at 0.4 m/s
  nav> move -1.0 m                      # Move straight backward 1.0 meter
  nav> turn 90 deg                      # Pivot in-place 90 degrees CCW (Left)
  nav> turn -45 deg                     # Pivot in-place 45 degrees CW (Right)
  ```

* **Diagnostics & Utility**:
  ```text
  nav> sensors                          # Live distance readings from Left, Middle, Right ToF
  nav> pose                             # Displays current (X, Y, Theta, Distance travelled)
  nav> plot                             # Generates navigation_result.png & JSON telemetry
  nav> stop                             # Emergency stop
  nav> quit                             # Clean exit
  ```

---

### Option 2: Direct Command Line One-Liners

* **Navigate to Coordinate with Final Orientation**:
  ```bash
  python3 -m autonomous_navigation.main --goal 1.5 1.0 90 --speed 0.4
  ```

* **Roundtrip (Navigate to Goal & Return Along Exact Path)**:
  ```bash
  python3 -m autonomous_navigation.main --goal 1.0 1.0 90 --return
  ```

* **Patrol Back and Forth $N$ Times**:
  ```bash
  python3 -m autonomous_navigation.main --goal 1.2 0.8 45 --patrol 3
  ```

* **Straight Line Distance**:
  ```bash
  python3 -m autonomous_navigation.main --distance 3.0 --speed 0.4
  ```

* **Desktop Mock Simulation (Generates visualization without physical robot)**:
  ```bash
  python3 -m autonomous_navigation.main --mock --goal 1.0 1.0 90 --patrol 2
  ```

---

## Critical Obstacle Filtering & Safety Shield

The navigation system features an advanced 4-stage filtering pipeline to prevent false obstacle detections from optical noise, dust, and floor glare:

| Filter Stage | Parameter | Default | Function |
|---|---|---|---|
| **Driver Median Filter** | `TOF_MEDIAN_WINDOW` | `5` samples | 5-sample running median filter in `DistanceSensorManager`. 100% rejects 1-tick/2-tick optical spikes. |
| **Crosstalk Cutoff** | `TOF_MIN_VALID_M` | `0.08 m` (8 cm) | Rejects readings $< 8\text{ cm}$ caused by sensor mounting bezel or optical cover crosstalk. |
| **Floor-Glare Cutoff** | `OBSTACLE_MAP_MAX_DIST_M` | `0.70 m` (70 cm) | Cuts off distant readings where the $25^\circ$ ToF cone grazes the floor surface. |
| **Spatial Consensus** | `OBSTACLE_PERSISTENCE_COUNT` | `5` hits | Projects hits into the $(X, Y)$ world frame. Static objects accumulate hits; moving floor glare drifts and is pruned. |
| **Cluster Radius** | `OBSTACLE_CONFIRM_RADIUS_M` | `0.12 m` (12 cm) | Spatial clustering radius to correlate repeated readings to the same static obstacle. |
| **Critical Brake Buffer** | `CRITICAL_STOP_DIST` | `0.12 m` (12 cm) | Emergency stop threshold that guarantees the robot halts safely before contact. |
| **Brake Persistence** | `CRITICAL_PERSIST_TICKS` | `2` cycles (100 ms) | Requires critical hazards to persist for $\ge 2$ consecutive cycles to reject transient optical spikes. |

---

## Map Visualization & Output Files

After every mission, the system automatically exports:
1. **`navigation_result.png`**:
   - Continuous obstacle chains linked with dashed boundary lines.
   - Circular safety clearance padding ($14\text{ cm}$ radius).
   - Robot swept track width corridor ($13.4\text{ cm}$ track width).
   - Start pose, waypoints, actual trajectory, and final target orientation arrow.
2. **`navigation_result_obstacles.json`**:
   - Exact metric Cartesian coordinates of all confirmed obstacles.
   - Final robot pose $(X, Y, \theta)$, total distance travelled, and elapsed time.
   - Mission status (`GOAL REACHED`, `RETURN COMPLETED`, `PATROL COMPLETED`).

---

## File Structure

```text
autonomous_navigation/
├── __init__.py           # Module package initialization
├── config.py             # System parameters, hardware pins, speeds, and safety thresholds
├── sensors.py            # 3x VL53L0X ToF driver with median filtering & sequential XSHUT booting
├── imu.py                # MPU-6050 spatial IMU driver with gyro calibration & collision detection
├── planner.py            # A* grid pathfinder, APF local vector field & differential kinematics
├── serial_comm.py        # ESP32 20 Hz UART stream, IMU-fused odometry, and startup kick
├── navigator.py          # Mission orchestrator, return-to-start, patrol, and spatial consensus
├── plotter.py            # Matplotlib swept corridor, continuous chains, and JSON telemetry exporter
├── main.py               # Main CLI and interactive entry point
├── tests/
│   ├── __init__.py
│   └── test_navigator.py # Full unit test suite (A*, APF, kinematics, retrace, patrol, filters)
└── README.md             # Documentation and usage guide
```
