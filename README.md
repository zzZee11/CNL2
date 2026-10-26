# CNL2 — Autonomous Human-Following Robot Car

A two-wheeled differential-drive robot car powered by an **ESP32-S3** microcontroller, driven over **USB Serial** from a host computer. The project combines real-time **WASD keyboard teleoperation** with a live telemetry dashboard, and includes a **vision-based autonomous human-following** pipeline designed for a Raspberry Pi with a camera.

---

## Table of Contents

- [Features (Implemented)](#features-implemented)
- [Planned Features (Roadmap)](#planned-features-roadmap)
- [System Architecture](#system-architecture)
- [Hardware Requirements](#hardware-requirements)
- [Wiring Diagram](#wiring-diagram)
- [Repository Structure](#repository-structure)
- [Installation & Setup](#installation--setup)
  - [1. ESP32-S3 Firmware](#1-esp32-s3-firmware)
  - [2. Teleoperation Host (PC / Laptop)](#2-teleoperation-host-pc--laptop)
  - [3. Vision Pipeline (Raspberry Pi)](#3-vision-pipeline-raspberry-pi)
- [Usage](#usage)
- [Telemetry Protocol](#telemetry-protocol)
- [Configuration Reference](#configuration-reference)
- [Troubleshooting](#troubleshooting)
- [License](#license)

---

## Features (Implemented)

### WASD Keyboard Teleoperation
- **Single-key controls**: `W` (forward), `S` (reverse), `A` (pivot left), `D` (pivot right), `Space` (active electronic brake)
- **Simultaneous multi-key detection**: `W+A` (forward-left arc), `W+D` (forward-right arc), `S+A` (reverse-left arc), `S+D` (reverse-right arc)
- **Adjustable speed**: `+` / `-` keys to change PWM duty cycle (range: 50–255)
- Dual-mode keyboard input:
  - **Desktop GUI**: Native key-press / key-release via `pynput`
  - **Headless SSH / terminal**: Raw `stdin` with 450 ms key-decay timing window

### Closed-Loop Motor Control
- **Straight-line encoder synchronization**: Proportional correction (Kp = 0.5) keeps both wheels turning at the same rate when driving forward or reverse, preventing drift
- **Bounded sync trim**: Correction is clamped to ±40 PWM to prevent motor saturation or runaway
- **Arc-turn kinematics**: Compound commands drive the outer wheel at full speed and the inner wheel at ⅓ speed for smooth curved paths

### Real-Time Telemetry Dashboard
- **Rich terminal UI** rendered at 20 Hz with color-coded panels:
  - Connection status, serial port, and baud rate
  - Active drive command and target PWM
  - Per-wheel RPM and cumulative encoder ticks
  - Watchdog health indicator
- Works over SSH (no GUI required)

### Safety Systems
- **350 ms dead-man watchdog**: If the host stops sending serial packets for 350 ms (USB disconnect, script crash, etc.), the ESP32 automatically stops both motors
- **20 Hz heartbeat transmitter**: The host continuously streams `CMD:` packets even when holding a key, keeping the watchdog fed
- **Active electronic braking**: `Space` key shorts both motor windings through the H-bridge for immediate mechanical lock

### Quadrature Encoder Odometry
- Hardware interrupt-driven (IRAM_ATTR ISRs) on Phase A rising edges with Phase B direction detection
- Thread-safe 64-bit tick access using FreeRTOS spinlocks
- Rolling RPM estimation every 50 ms

### Vision-Based Human Following (Raspberry Pi)
- YOLO pose estimation for person detection and tracking
- Gait/stance classification (standing vs. walking) using keypoint variance
- Vision-only collision avoidance with time-to-collision (TTC) emergency brake
- Autonomous APF (Artificial Potential Field) steering toward detected person
- Configurable follow distance, camera parameters, and tracking timeouts

---

## Planned Features (Roadmap)

- [ ] **Distance sensor integration** — VL53L0X ToF sensors for obstacle detection and proximity-based emergency stop
- [ ] **A\* grid-based waypoint navigation** — Autonomous path planning with dynamic obstacle mapping (scaffolding already exists in `rasberrypi_host/robot_main_1.py`)
- [ ] **Web dashboard** — Browser-based control panel with live video feed, telemetry charts, and mode switching
- [ ] **PID speed control** — Full PID (proportional-integral-derivative) velocity control using encoder RPM feedback instead of open-loop PWM
- [ ] **Wireless control** — Wi-Fi / Bluetooth teleoperation replacing wired USB serial
- [ ] **Odometry-based position estimation** — Dead-reckoning XY position tracking from encoder ticks and known wheel geometry
- [ ] **ROS 2 integration** — Standard robotics middleware for sensor fusion, mapping, and navigation stack
- [ ] **Multi-mode state machine** — Seamless switching between teleoperation, autonomous navigation, and human-following modes from a single interface

---

## System Architecture

```
┌─────────────────────────────────────────────────────┐
│             HOST COMPUTER / RASPBERRY PI            │
│                                                     │
│  test_robot.py / rasberrypi_host                    │
│    ├── User target speed input (Left/Right m/s)     │
│    ├── 20 Hz periodic command stream                │
│    └── Serial communications interface              │
└──────────────┬──────────────────────────────────────┘
               │  UART / USB Serial (115200 baud)
               │  <left_mps>,<right_mps>\n  ↓↑  Debug RPM
┌──────────────┴──────────────────────────────────────┐
│                    ESP32-S3                         │
│                                                     │
│  RobotController.ino                                │
│    ├── SerialComms update & watchdog freshness      │
│    ├── Velocity conversion: m/s → Target RPM        │
│    └── 50 ms loop execution                         │
│              │                                      │
│  SerialComms.cpp                                    │
│    ├── Non-blocking stream parser                   │
│    └── 500 ms safety timeout                        │
│              │                                      │
│  WheelSpeedPID.cpp                                  │
│    ├── Closed-loop discrete PID (Kp=2.0, Ki=0.5)    │
│    ├── Anti-windup integral clamping                │
│    └── Output PWM calculation (-255 to +255)        │
│              │                                      │
│  MotoController.cpp                                 │
│    ├── L298N direction GPIO control                 │
│    └── ESP32 LEDC PWM generation (20 kHz, 8-bit)    │
│              │                                      │
│  EncoderReader.cpp                                  │
│    ├── Hardware interrupt ISRs on encoder pins      │
│    └── Cumulative tick counting                     │
│              │                                      │
│  Config.h                                           │
│    └── All pinouts, wheel geometry, PID & UART cfg  │
└─────────────────────────────────────────────────────┘
```

---

## Hardware Requirements

| Component | Specification |
|---|---|
| Microcontroller | ESP32-S3-DevKitC-1 (native USB CDC) |
| Motor Driver | L298N Dual H-Bridge |
| Motors | 2× DC geared motors with built-in hall encoders (11 PPR, 30:1 gear ratio) |
| Power | 7–12V battery pack for motors, USB for ESP32 logic |
| Chassis | 2WD differential-drive robot car chassis |
| Host Computer | Linux PC/laptop with USB port (for teleoperation) |
| *(Optional)* | Raspberry Pi 4/5 with camera module (for vision pipeline) |

---

## Wiring Diagram

### ESP32-S3 → L298N Motor Driver

| Signal | ESP32-S3 GPIO | L298N Pin | Description |
|---|---|---|---|
| Left Motor PWM | **GPIO 4** | ENA | Speed control via LEDC (remove ENA jumper!) |
| Left Motor Dir 1 | **GPIO 6** | IN1 | Direction input 1 |
| Left Motor Dir 2 | **GPIO 5** | IN2 | Direction input 2 |
| Right Motor PWM | **GPIO 9** | ENB | Speed control via LEDC (remove ENB jumper!) |
| Right Motor Dir 1 | **GPIO 8** | IN3 | Direction input 3 |
| Right Motor Dir 2 | **GPIO 7** | IN4 | Direction input 4 |

> **⚠️ Important**: Remove the black jumper caps from **ENA** and **ENB** on the L298N board. These jumpers bypass PWM speed control and lock motors to full speed.

### ESP32-S3 → Quadrature Encoders

| Signal | ESP32-S3 GPIO | Encoder Wire | Description |
|---|---|---|---|
| Left Encoder Phase A (C1) | **GPIO 1** | Channel A (Yellow) | Interrupt trigger (`CHANGE`) |
| Left Encoder Phase B (C2) | **GPIO 2** | Channel B (Green) | Direction sensing |
| Right Encoder Phase A (C1) | **GPIO 12** | Channel A (Yellow) | Interrupt trigger (`CHANGE`) |
| Right Encoder Phase B (C2) | **GPIO 13** | Channel B (Green) | Direction sensing |

### Power Connections

| Connection | Details |
|---|---|
| L298N 12V input | Battery pack positive (+) terminal |
| L298N GND | Battery pack negative (−) terminal **AND** ESP32-S3 GND |
| ESP32-S3 power | USB cable from host computer / 5V step-down regulator |
| L298N OUT1/OUT2 | Left motor terminals |
| L298N OUT3/OUT4 | Right motor terminals |

> **⚠️ Important**: The ESP32-S3 GND and L298N GND **must** be connected together (common ground). Without this, serial communication and PWM signals will be unreliable.

### ESP32-S3 → Raspberry Pi 4B GPIO Serial Connection (Hardware UART)

You can connect the Raspberry Pi 4B directly to the ESP32-S3 via the 40-pin GPIO header without needing a USB cable. Both boards share compatible **3.3V logic levels** (no level shifter required).

| Raspberry Pi 4B Pin | Raspberry Pi Function | ESP32-S3 Pin | ESP32-S3 Function |
|---|---|---|---|
| **Pin 8** (GPIO 14) | UART0 TXD | **GPIO 18** | `UART_RX_PIN` (Receive) |
| **Pin 10** (GPIO 15) | UART0 RXD | **GPIO 17** | `UART_TX_PIN` (Transmit) |
| **Pin 6 or 9** | GND | **GND** | Common Ground |

> **⚠️ Critical Configuration on Raspberry Pi**:
> By default, Raspberry Pi OS attaches a serial login shell (console) to the GPIO pins. You must disable the login shell and enable the hardware serial port:
> 1. Run `sudo raspi-config`
> 2. Go to **Interface Options** → **Serial Port**
> 3. *Would you like a login shell to be accessible over serial?* → Select **No**
> 4. *Would you like the serial port hardware to be enabled?* → Select **Yes**
> 5. Finish and reboot the Pi (`sudo reboot`).
>
> Once rebooted, the port will be available as `/dev/serial0`. You can launch the robot controller with:
> ```bash
> python3 robot_main.py --port /dev/serial0 --ssh
> ```

### ESP32-S3 → 3× VL53L0X Time-of-Flight Distance Sensors (I2C)

The robot car uses 3 laser ToF distance sensors for front collision avoidance and dynamic A* obstacle grid mapping:
- **Left Sensor (S1)**: +45° angled
- **Center Sensor (S2)**: 0° straight forward (**No XSHUT connection required**)
- **Right Sensor (S3)**: −45° angled

| Signal | ESP32-S3 GPIO | Sensor Pin | Description |
|---|---|---|---|
| I2C SDA | **GPIO 11** | SDA / SDL (shared) | Common I2C Data bus |
| I2C SCL | **GPIO 10** | SCL (shared) | Common I2C Clock bus |
| Sensor Power | **3.3V / VIN** | VIN (shared) | 3.3V logic supply |
| Ground | **GND** | GND (shared) | Common Ground |
| Left XSHUT | **GPIO 1** | XSHUT (S1) | Software boot control (re-addressed to `0x30`) |
| Center XSHUT | *Not Connected* | *(None)* | **Always enabled** (re-addressed first from `0x29` → `0x31`) |
| Right XSHUT | **GPIO 2** | XSHUT (S3) | Software boot control (re-addressed to `0x32`) |

> **💡 How 2 XSHUT Pins Service 3 Sensors**:
> 1. At boot, ESP32 drives **Left XSHUT LOW** and **Right XSHUT LOW** to put them to sleep.
> 2. The **Center Sensor** has no XSHUT, so it is the only sensor active at factory default `0x29`. The firmware initializes it and immediately moves its address to `0x31`, freeing `0x29`.
> 3. The firmware releases **Left XSHUT HIGH** (boots at `0x29`), moves it to `0x30`, freeing `0x29` again.
> 4. The firmware releases **Right XSHUT HIGH** (boots at `0x29`) and moves it to `0x32`.
> 5. All 3 sensors now operate concurrently on unique I2C addresses with zero bus collisions.

---

## Repository Structure

```
CNL2/
├── README.md                          # ← Project documentation and setup guide
├── .gitignore
├── raspberrypi_host                   # Symlink to rasberrypi_host
│
├── RobotController/                   # ESP32-S3 closed-loop PID firmware (Arduino IDE)
│   ├── RobotController.ino            # Main sketch (setup, 50 ms loop, velocity conversion)
│   ├── Config.h                       # Pin assignments, wheel geometry, PID gains & timeouts
│   ├── MotorController.h              # Motor driver interface
│   ├── MotoController.cpp             # L298N direction & LEDC PWM implementation (20 kHz)
│   ├── EncoderReader.h                # Interrupt-based encoder counter interface
│   ├── EncoderReader.cpp              # Hardware interrupt ISRs (CHANGE) for ticks
│   ├── WheelSpeedPID.h                # Discrete PID speed controller declaration
│   ├── WheelSpeedPID.cpp              # Dual-wheel discrete PID with anti-windup clamping
│   ├── SerialComms.h                  # UART packet receiver & watchdog declaration
│   └── SerialComms.cpp                # Non-blocking newline parser & 500 ms safety timeout
│
├── test_robot.py                      # Standalone interactive Python test script (m/s velocity commands)
│
├── teleop_host/                       # Standalone teleoperation & serial drivers (Python)
│   ├── README.md
│   ├── requirements.txt               # pyserial, pynput, rich
│   ├── teleop_controller.py           # Standalone keyboard teleop controller
│   ├── serial_interface.py            # Threaded serial TX/RX manager & heartbeat
│   └── telemetry_model.py             # Telemetry data model & parser
│
├── rasberrypi_host/                   # Unified Autonomous & Teleop Robot Pipeline
│   ├── master_controller.py           # ★ Central Master State Machine Controller (WASD + Nav + Follow)
│   ├── robot_main.py                  # Primary entry point launching Master State Machine
│   ├── robot_main_1.py                # Backward-compatible entry point
│   ├── a_search_main.py               # A* grid path planning & APF obstacle avoidance
│   ├── vision_bridge.py               # Bridge between vision pipeline & robot controller
│   └── vision/                        # Computer vision subsystem
│       ├── config.py                  #   YOLO, camera, gait, & safety parameters
│       ├── main.py                    #   Vision pipeline entry point
│       ├── detector.py                #   Multi-backend detector (TFLite/ONNX/PyTorch)
│       ├── tracker.py                 #   Multi-object tracker
│       ├── target_selector.py         #   Target person selection logic
│       ├── vision.py                  #   Full vision pipeline orchestrator
│       ├── motion.py                  #   Gait/stance classification & stop detection
│       ├── requirements.txt           #   Lightweight edge requirements (tflite-runtime)
│       ├── requirements-desktop.txt   #   Desktop GPU requirements (ultralytics, torch)
│       └── models/                    #   Pre-trained model weights
│           ├── yolo26n.pt
│           ├── yolo26n-pose.pt
│           ├── detect.tflite
│           └── labelmap.txt
```

---

## Installation & Setup

### Prerequisites

- **Python 3.10+** on the host computer
- **Arduino IDE 2.x** or **PlatformIO** for flashing the ESP32
- **Git** for cloning the repository

```bash
git clone https://github.com/zzZee11/CNL2.git
cd CNL2
```

---

### 1. ESP32-S3 Firmware

#### Arduino IDE 2.x

1. **Install the ESP32 board package**:
   - Open Arduino IDE → **File → Preferences**
   - Add this URL to *Additional Board Manager URLs*:
     ```
     https://espressif.github.io/arduino-esp32/package_esp32_index.json
     ```
   - Go to **Tools → Board → Boards Manager** → Search `esp32` → Install **esp32 by Espressif Systems** (version 3.x recommended)

2. **Configure the board**:
   - **Tools → Board**: `ESP32S3 Dev Module`
   - **Tools → USB CDC On Boot**: `Enabled`
   - **Tools → USB Mode**: `Hardware CDC and JTAG`
   - **Tools → Upload Speed**: `921600`
   - **Tools → Port**: Select your ESP32's USB port (e.g., `/dev/ttyACM0` or `/dev/ttyUSB0`)

3. **Open and upload**:
   - Open `RobotController/RobotController.ino` in Arduino IDE
   - Click **Upload** (→ button)
   - Open the **Serial Monitor** at `115200 baud` to view debug RPM telemetry and confirm initialization:
     ```
     Robot controller initialized.
     ```

---

### 2. Teleoperation Host (PC / Laptop)

1. **Create a virtual environment** (recommended):
   ```bash
   cd teleop_host
   python3 -m venv venv
   source venv/bin/activate
   ```

2. **Install Python dependencies**:
   ```bash
   pip install -r requirements.txt
   ```
   This installs:
   | Package | Purpose |
   |---|---|
   | `pyserial` | USB serial communication with ESP32 |
   | `pynput` | Native OS keyboard event capture (desktop environments) |
   | `rich` | Terminal UI dashboard rendering |

3. **Grant serial port permissions** (Linux, one-time):
   ```bash
   sudo usermod -aG dialout $USER
   ```
   Log out and log back in for this to take effect.

---

### 3. Vision Pipeline (Raspberry Pi 4B)

> **Note**: For edge devices like the Raspberry Pi 4B (4GB RAM, 16GB MicroSD), do **not** install full PyTorch (`torch`), which requires >3GB disk space and exhausts SD card storage. Use the lightweight **TFLite runtime** (~3MB) which runs the included `detect.tflite` model at 15–25 FPS.

1. **Reclaim SD card space if you encountered "No space left on device"**:
   ```bash
   # Clear failed pip downloads and cache
   pip cache purge
   rm -rf ~/.cache/pip /tmp/pip-*
   sudo apt clean && sudo apt autoremove -y

   # Optional: ensure your SD card root partition is fully expanded to 16GB
   sudo raspi-config
   # Navigate to: Advanced Options -> Expand Filesystem -> Finish & Reboot
   ```

2. **Install lightweight dependencies (< 60 MB total)**:
   ```bash
   # Install hardware-optimized OpenCV and NumPy from Raspberry Pi OS apt repository
   sudo apt update
   sudo apt install -y python3-opencv python3-numpy python3-pip

   # Install the ultra-lightweight TFLite runtime (~3MB)
   cd rasberrypi_host/vision
   pip3 install -r requirements.txt --no-cache-dir
   ```

3. **Configure** `rasberrypi_host/robot_main_1.py`:
   - Set `SERIAL_PORT` to your ESP32 serial device (e.g., `/dev/ttyUSB0` or `/dev/ttyACM0`)
   - The system automatically loads `models/detect.tflite` (4MB SSD MobileNet) with zero PyTorch dependencies.
   - *(Desktop GPU users with ample storage can install `requirements-desktop.txt` to use PyTorch YOLO models).*

---

## Usage

### Master State Machine Controller (Recommended)

Run the unified Master State Machine on your Raspberry Pi (or PC connected via USB):

```bash
cd rasberrypi_host
python3 robot_main.py
```
*(Or specify port and speed: `python3 robot_main.py --port /dev/ttyACM0 --speed 220`)*

The system launches an interactive live Rich dashboard. All driving commands route directly through the Master State Machine:

#### Mode Switching & Shortcuts:
| Key | Mode / Action | Description |
|---|---|---|
| `T` or `1` | **TELEOP Mode** | Direct WASD manual keyboard control |
| `F` or `2` | **HUMAN_FOLLOW Mode** | Autonomous camera tracking & following |
| `N` or `3` | **GRID_NAV Mode** | Autonomous A* path planning & obstacle avoidance |
| `I` or `0` | **IDLE Mode** | Stop motors and enter safe standby |
| `Space` | **ACTIVE BRAKE** | Electronic brake — locks both wheels instantly |
| `+` / `-` | **Throttle Speed** | Increase / decrease drive PWM between 50 and 255 |
| `Q` | **Quit** | Stop motors and exit cleanly |

#### WASD Key Controls (Active in TELEOP Mode):
| Key(s) | Action | Kinematics |
|---|---|---|
| `W` | Forward | Both wheels forward, encoder-synced straight line |
| `S` | Reverse | Both wheels reverse, encoder-synced straight line |
| `A` | Pivot Left | Only left wheel spins (car pivots on right wheel) |
| `D` | Pivot Right | Only right wheel spins (car pivots on left wheel) |
| `W` + `A` | Forward-Left Arc | Right wheel full speed, left wheel ⅓ speed |
| `W` + `D` | Forward-Right Arc | Left wheel full speed, right wheel ⅓ speed |
| `S` + `A` | Reverse-Left Arc | Reverse arc turning left |
| `S` + `D` | Reverse-Right Arc | Reverse arc turning right |
| `Space` | Active Brake | Electronic brake — locks both wheels instantly |

### Standalone Interactive Speed Test (`test_robot.py`)

A script to directly test closed-loop velocity commands and PID motor response:

```bash
python3 test_robot.py
```
*(By default connects to `/dev/ttyACM0` at `115200` baud. Edit `SERIAL_PORT` in `test_robot.py` to `/dev/serial0` if connecting via Raspberry Pi GPIO UART).*

- **Format**: Enter left and right speeds in m/s (e.g., `0.20,0.20` to drive forward, or `-0.15,0.15` to spin).
- **Stop**: Type `stop` to halt both motors.
- **Quit**: Type `q` to send zero speed and close the serial link.

---

## Communication & Control Protocol

### Host → ESP32 (Velocity Commands)

Commands are sent across the serial connection as ASCII strings terminated by a newline (`\n`):

```
<left_mps>,<right_mps>\n
```

| Field | Range | Description | Example |
|---|---|---|---|
| `left_mps` | `-2.0` to `+2.0` m/s | Target linear speed for left wheel | `0.20` |
| `right_mps` | `-2.0` to `+2.0` m/s | Target linear speed for right wheel | `0.20` |

**Examples:**
- `0.20,0.20\n` → Drive forward at 0.20 m/s
- `-0.20,-0.20\n` → Drive backward at 0.20 m/s
- `0.0,0.0\n` or `stop` → Stop both motors
- `0.20,-0.20\n` → Pivot turn clockwise

> **⚠️ Watchdog Safety**: The ESP32 enforces a 500 ms safety timeout (`COMMAND_TIMEOUT_MS`). The host must continuously stream velocity packets (recommended rate: 20 Hz / 50 ms). If no valid command packet arrives within 500 ms, target velocities automatically drop to zero.

### ESP32 → Host / USB Monitor (Debug Telemetry)

The firmware outputs real-time RPM tracking over USB serial at the PID loop rate (20 Hz):

```
Target RPM: <target_left>, <target_right> | Actual RPM: <actual_left>, <actual_right>
```

Example:
```
Target RPM: 38.20, 38.20 | Actual RPM: 37.95, 38.10
```

---

## Configuration Reference

All active firmware configuration constants are defined in [`RobotController/Config.h`](RobotController/Config.h):

### Motor Driver Pinouts & PWM

| Parameter | Value | Description |
|---|---|---|
| `LEFT_MOTOR_IN1` | `GPIO 6` | Left motor direction pin 1 |
| `LEFT_MOTOR_IN2` | `GPIO 5` | Left motor direction pin 2 |
| `LEFT_MOTOR_PWM` | `GPIO 4` | Left motor PWM pin (LEDC speed control) |
| `RIGHT_MOTOR_IN1` | `GPIO 8` | Right motor direction pin 1 |
| `RIGHT_MOTOR_IN2` | `GPIO 7` | Right motor direction pin 2 |
| `RIGHT_MOTOR_PWM` | `GPIO 9` | Right motor PWM pin (LEDC speed control) |
| `MOTOR_PWM_FREQUENCY` | `20000` Hz (20 kHz) | Ultrasonic PWM frequency (eliminates audible motor whine) |
| `MOTOR_PWM_RESOLUTION` | `8` bits | PWM resolution (0–255 duty cycle) |
| `MOTOR_PWM_MAX` | `255` | Maximum clamped PWM value |

### Quadrature Encoders

| Parameter | Value | Description |
|---|---|---|
| `LEFT_ENCODER_A` | `GPIO 1` | Left encoder channel A (interrupt attached on `CHANGE`) |
| `LEFT_ENCODER_B` | `GPIO 2` | Left encoder channel B (direction sensing) |
| `RIGHT_ENCODER_A` | `GPIO 12` | Right encoder channel A (interrupt attached on `CHANGE`) |
| `RIGHT_ENCODER_B` | `GPIO 13` | Right encoder channel B (direction sensing) |
| `TICKS_PER_REVOLUTION` | `360.0f` | Encoder tick counts per single wheel revolution |

### Wheel Geometry

| Parameter | Value | Description |
|---|---|---|
| `WHEEL_RADIUS_M` | `0.05f` (5 cm) | Wheel radius in metres (converts linear m/s to angular RPM) |
| `WHEEL_BASE_M` | `0.20f` (20 cm) | Track width / distance between left and right wheel centres |

### Closed-Loop PID Speed Controller

| Parameter | Value | Description |
|---|---|---|
| `PID_SAMPLE_TIME_MS` | `50` ms (20 Hz) | Discrete PID update and velocity calculation interval |
| `PID_KP` | `2.0f` | Proportional gain |
| `PID_KI` | `0.5f` | Integral gain (with anti-windup clamping) |
| `PID_KD` | `0.0f` | Derivative gain |

### UART Communication & Watchdog

| Parameter | Value | Description |
|---|---|---|
| `UART_BAUD_RATE` | `115200` baud | Baud rate for Raspberry Pi UART link |
| `UART_RX_PIN` | `GPIO 18` | ESP32 UART RX pin (connects to Raspberry Pi TX Pin 8) |
| `UART_TX_PIN` | `GPIO 17` | ESP32 UART TX pin (connects to Raspberry Pi RX Pin 10) |
| `COMMAND_TIMEOUT_MS` | `500` ms | Failsafe watchdog timeout (stops motors if link is lost) |
| `TELEMETRY_INTERVAL_MS` | `100` ms | Serial telemetry broadcast interval |

---

## Troubleshooting

### ESP32 not detected on USB
- Ensure the cable is a **data cable**, not a charge-only cable
- Check `ls /dev/ttyACM*` or `ls /dev/ttyUSB*`
- Install the ESP32 USB driver if needed: `sudo apt install linux-tools-common`

### Motors don't spin
- Verify the **ENA** and **ENB** jumpers are **removed** from the L298N board
- Check that the battery pack is connected and charged (7–12V)
- Confirm **common ground** between ESP32 and L298N

### Wheels spin in reverse or mismatched direction
- If a motor rotates backward when positive speed is requested, swap the motor's `IN1` and `IN2` wires on the L298N terminals or swap `LEFT_MOTOR_IN1`/`IN2` (or `RIGHT_MOTOR_IN1`/`IN2`) in `Config.h`.
- If encoder ticks count backward during forward motion, swap `Channel A` and `Channel B` encoder signal wires.

### Motors stop after ~500 ms (Watchdog Tripped)
- The firmware includes a safety watchdog (`COMMAND_TIMEOUT_MS = 500`). The sender must periodically stream commands at ~20 Hz (e.g. using `test_robot.py` or the host controller loop).

### Permission denied on serial port
```bash
sudo usermod -aG dialout $USER
# Then log out and log back in
```

---

## License

This project is developed as part of an academic/personal robotics project. License TBD.
