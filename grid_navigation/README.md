# Grid-Based Autonomous Navigation System (`grid_navigation`)

A clean, deterministic, from-scratch 2D discrete grid-based navigation system designed for differential drive robots. Built to replace complex continuous potential field approaches with a transparent, robust state machine and A* grid search.

---

## 1. Key Features

- **Discrete Occupancy Grid & Circular Clearance Inflation**:
  - $10\text{ cm}$ discrete grid resolution with coordinate transforms between metric world coordinates $(x, y)\text{ meters}$ and discrete grid cells $(i, j)$.
  - Exact Euclidean circular obstacle dilation ($R = 14\text{ cm}$) ensuring the robot frame never approaches physical hazards.
- **8-Connected A\* Path Planning & Line-of-Sight Smoothing**:
  - 8-connected A* search with diagonal corner-cutting prevention (rejects diagonal steps if adjacent orthogonal cells are blocked).
  - Bresenham line-of-sight raycasting to eliminate jagged stair-stepping into direct, smooth straight-line paths.
- **In-Place Pivot Alignment**:
  - Pivots in place to face target path waypoints before initiating forward translation, eliminating wide arc drift and corner clipping.
- **3-Stage Deceleration & Terminal Creep Docking**:
  - Full cruise speed ($0.35\text{ m/s}$ default, up to $0.80\text{ m/s}$) when $> 0.35\text{ m}$ from goal.
  - Linear smooth deceleration between $0.35\text{ m}$ and $0.12\text{ m}$.
  - Precision creep deceleration ($0.04\text{ m/s}$) below $0.12\text{ m}$ down to $\le 2.5\text{ cm}$ arrival tolerance.
  - Active counter-torque electrical braking upon arrival to prevent inertial overshoot.
- **Exact-Path Forward Return to Start**:
  - Retraces the recorded outward trajectory back to $(0.0, 0.0)$ in **forward drive motion** (pivots in place at the goal first, never drives in reverse).
  - Restores exact initial heading ($0.0^\circ$).
- **Multi-Cycle Patrol Shuttle**:
  - Shuttles back and forth between start $(0, 0)$ and target $(x, y)$ for $N$ full cycles.
- **Dual-Verification Sensor Filtering**:
  - 5-sample rolling median filtering across 3x VL53L0X Time-of-Flight sensors ($30^\circ\text{ left}$, $0^\circ\text{ center}$, $-30^\circ\text{ right}$).
  - Stationary confirmation: halts briefly to re-sample distance before classifying an obstacle, eliminating false optical spikes.
- **Stationary Gyro Drift Re-Zeroing**:
  - MPU-6050 complementary gyro integration ($\alpha = 0.96$) with automatic stationary bias recalibration during docking pauses.
- **Hardware Abstraction & Native Simulation**:
  - Seamless dual operation: connects to ESP32 UART serial + I2C sensors on physical robot, or runs instant full-physics simulation via `--mock`.
- **Top-Down Visualizer**:
  - Generates crisp 2D map PNGs (via `matplotlib` or fallback `Pillow`) displaying obstacles, inflation halos, planned paths, actual trajectories, and companion JSON telemetry.

---

## 2. Directory Structure

```
grid_navigation/
├── __init__.py               # Package initializer
├── config.py                 # Central robot geometry, pinouts, speeds, and tolerances
├── grid_map.py               # OccupancyGridMap (transforms, Euclidean obstacle inflation)
├── planner.py                # 8-connected A* search & Bresenham line-of-sight smoothing
├── robot_hardware.py         # Hardware abstraction (ESP32 UART, MPU-6050, VL53L0X, Mock)
├── navigator.py              # GridNavigator state machine (docking, return, patrol, replanning)
├── visualizer.py             # PNG map generator (Matplotlib / Pillow) & JSON telemetry
├── main.py                   # CLI entrypoint & interactive navigation shell
├── README.md                 # System documentation & usage guide
└── tests/                    # Comprehensive unit and integration test suite
    ├── __init__.py
    ├── test_grid_map.py      # Grid transforms, obstacle inflation, collision checks
    ├── test_planner.py       # A* optimality, diagonal corner checks, path smoothing
    └── test_navigator.py     # End-to-end integration: docking accuracy, return, patrol
```

---

## 3. Quick Start & CLI Usage

All commands can run with `--mock` on any PC or directly on the robot hardware without flags.

### Navigate to Goal with Final Orientation
Navigates from `(0, 0)` to Cartesian coordinate `(1.0, 1.0)` meters and aligns to heading $+90.0^\circ$:
```bash
python3 grid_navigation/main.py --mock --goal 1.0 1.0 90
```

### Navigate to Goal and Retrace Forward Path to Start
Navigates to `(1.2, 0.6)`, docks within $\le 2.5\text{ cm}$, pivots in place, and retraces the outward trajectory forward back to `(0, 0)` facing $0^\circ$:
```bash
python3 grid_navigation/main.py --mock --goal 1.2 0.6 --return
```

### Multi-Cycle Patrol Mission
Shuttles back and forth between `(0, 0)` and `(1.0, 0.5)` for 3 complete round trips:
```bash
python3 grid_navigation/main.py --mock --goal 1.0 0.5 --patrol 3
```

### Custom Cruise Speed and Export Path
```bash
python3 grid_navigation/main.py --mock --goal 2.0 1.0 --speed 0.50 --plot my_mission.png
```

---

## 4. Interactive Navigation Shell

Launch the interactive CLI shell by running `main.py` without navigation arguments:
```bash
python3 grid_navigation/main.py --mock
```

Available shell commands:
| Command | Description |
|---|---|
| `goto <x> <y> [theta_deg]` | Plans and drives to coordinate `(x, y)` meters |
| `return` | Retraces exact path forward back to start point `(0, 0)` |
| `patrol <x> <y> <cycles> [theta_deg]` | Runs $N$ full cycles back and forth |
| `pose` | Prints live dead-reckoning pose `(x, y, theta)` and distance travelled |
| `add_obs <x> <y>` | Inserts an obstacle at `(x, y)` and updates grid inflation |
| `clear` | Clears all dynamic obstacles from grid map |
| `plot [path]` | Exports current navigation map and companion JSON |
| `stop` | Halts robot motors immediately |
| `exit` / `quit` | Closes hardware connection and exits shell |

---

## 5. Verification & Test Suite

Run the full automated test suite (grid map, A* planner, and navigator integration):
```bash
python3 -m unittest discover -s grid_navigation/tests -v
```

All 12 test cases verify:
- Coordinate conversions and bounds clamping.
- Circular obstacle dilation radius ($R = 14\text{ cm}$).
- Straight line and obstacle-detouring A* trajectories.
- Rejection of diagonal corner-cutting through obstacles.
- Sub-2.5 cm precision docking accuracy.
- Forward-motion trajectory retracing back to start.
- Multi-cycle patrol shuttling.
- Stationary ToF optical spike rejection.

