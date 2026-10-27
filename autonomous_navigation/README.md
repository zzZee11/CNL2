# Autonomous Distance & Obstacle Avoidance Navigator

A complete, self-contained autonomous navigation system for the differential-drive robot car combining:
- **3× VL53L0X Time-of-Flight (ToF) Sensors**: Left (+45°), Middle (0°), and Right (−45°) on Raspberry Pi I2C.
- **A\* Global Grid Planner**: Maps detected obstacles with circular robot inflation clearance and computes the shortest obstacle-free route.
- **Artificial Potential Field (APF) Local Planner**: Real-time micro-evasion vector field that smoothly guides the robot around obstacles without stopping.
- **ESP32-S3 Serial Closed-Loop Velocity Control**: 20 Hz heartbeat stream over `/dev/serial0` (115200 baud).
- **Startup Breakaway Kick**: 180 ms high-torque pulse (`0.85 m/s` demand $\to$ `255 PWM`) to overcome L298N gearbox static friction (stiction).
- **Odometry & Dead-Reckoning**: Continuous pose $(x, y, \theta)$ tracking and milestone distance completion.

---

## Architecture Overview

```
┌────────────────────────────────────────────────────────────────────────┐
│                        RASPBERRY PI 4B                                 │
│                                                                        │
│  autonomous_navigation/main.py                                         │
│    ├── CLI Interface ('move 5 m 0.8')                                  │
│    │                                                                   │
│  navigator.py (AutonomousNavigator)                                    │
│    ├── Odometry tracker (s, x, y, theta)                               │
│    ├── Obstacle mapper (circular inflation)                            │
│    ├── A* global pathfinder & dynamic replanner                        │
│    └── APF reactive vector evasion (F_att + F_rep)                     │
│              │                                     │                   │
│  sensors.py (DistanceSensorManager)    serial_comm.py (RobotSerialComm)│
│    ├── Sequential XSHUT boot                 ├── 20 Hz TX stream       │
│    ├── Left: 0x30, Mid: 0x31, Right: 0x32    ├── Telemetry RX thread   │
│    └── 20 Hz ToF polling                     └── Startup torque kick   │
└──────────────┬─────────────────────────────────────┬───────────────────┘
               │ I2C Bus 1 (SCL, SDA)                │ UART (/dev/serial0)
               ▼                                     ▼
        3× VL53L0X Sensors                        ESP32-S3
        (L: 45°, M: 0°, R: -45°)           (WheelSpeedPID + L298N)
```

---

## Hardware Pinout Reference

### Raspberry Pi 4B $\rightarrow$ 3× VL53L0X Time-of-Flight Sensors (I2C)

| Signal | Raspberry Pi Pin | Sensor Pin | Purpose |
|---|---|---|---|
| I2C SCL | **GPIO 3** (Physical Pin 5) | SCL (shared) | Common I2C clock bus |
| I2C SDA | **GPIO 2** (Physical Pin 3) | SDA (shared) | Common I2C data bus |
| Power | **3.3V** (Physical Pin 1) | VIN (shared) | 3.3V logic power supply |
| Ground | **GND** (Physical Pin 6/9) | GND (shared) | Common Ground |
| Left XSHUT | **GPIO 17** (Physical Pin 11) | XSHUT (Left Sensor) | Readdresses Left to `0x30` |
| Middle XSHUT | **GPIO 22** (Physical Pin 15) | XSHUT (Mid Sensor) | Readdresses Middle to `0x31` |
| Right XSHUT | **GPIO 27** (Physical Pin 13) | XSHUT (Right Sensor) | Readdresses Right to `0x32` |

### Raspberry Pi 4B $\rightarrow$ ESP32-S3 UART Connection

| Raspberry Pi Pin | Function | ESP32-S3 Pin | Function |
|---|---|---|---|
| **Pin 8** (GPIO 14) | UART0 TXD | **GPIO 18** | ESP32 UART RX |
| **Pin 10** (GPIO 15) | UART0 RXD | **GPIO 17** | ESP32 UART TX |
| **Pin 6 or 9** | GND | **GND** | Common Ground |

---

## How to Run

### Option 1: Direct Command Line Run (e.g. Move 5 meters at 0.8 m/s)
```bash
python3 -m autonomous_navigation.main --distance 5.0 --speed 0.8
```

### Option 2: Interactive Terminal Shell
```bash
python3 -m autonomous_navigation.main
```
In the interactive prompt:
```text
nav> move 5 m 0.8      # Moves 5 meters at 0.8 m/s with full obstacle avoidance
nav> move 2.5 m        # Moves 2.5 meters at default speed (0.8 m/s)
nav> sensors           # Prints live ToF distances (Left, Middle, Right)
nav> pose              # Displays current (X, Y, Heading, Distance travelled)
nav> stop              # Emergency brake
nav> quit              # Clean exit
```

### Option 3: Desktop Simulation / Testing (Without Physical Hardware)
```bash
python3 -m autonomous_navigation.main --mock --distance 5.0 --speed 0.8
```

---

## File Structure

```text
autonomous_navigation/
├── __init__.py           # Module package initialization
├── config.py             # System parameters, pins, speeds, and dimensions
├── sensors.py            # 3x VL53L0X ToF driver with sequential XSHUT booting
├── planner.py            # A* grid pathfinder & APF local vector field
├── serial_comm.py        # ESP32 20 Hz UART stream, odometry, and startup kick
├── navigator.py          # Mission orchestrator and real-time obstacle avoidance loop
├── main.py               # Main CLI and interactive entry point
└── README.md             # Documentation and usage guide
```

