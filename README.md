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
│                   HOST COMPUTER                     │
│                                                     │
│  teleop_controller.py                               │
│    ├── Keyboard capture (pynput / raw terminal)     │
│    ├── Multi-key chord resolution (W+A → "WA")      │
│    └── Rich live terminal dashboard                 │
│              │                                      │
│  serial_interface.py                                │
│    ├── 20 Hz TX heartbeat thread  ──┐               │
│    ├── RX telemetry parser thread   │               │
│    └── Auto port detection          │               │
│              │                      │               │
│  telemetry_model.py                 │               │
│    └── TELE: packet dataclass       │               │
└──────────────┼──────────────────────┘               │
               │  USB Serial (115200 baud)            │
               │  CMD:WA,200\n  ↓↑  TELE:...          │
┌──────────────┼──────────────────────────────────────┐
│              │        ESP32-S3                      │
│                                                     │
│  main.cpp                                           │
│    ├── Serial packet parser (CMD:<DIR>,<SPEED>)     │
│    ├── 350 ms safety watchdog                       │
│    └── 20 Hz telemetry emitter                      │
│              │                                      │
│  pid_controller.cpp                                 │
│    ├── Drive state → motor PWM mapping              │
│    ├── Straight-line encoder sync (Kp trimming)     │
│    └── Arc-turn kinematics (⅓ inner wheel)          │
│              │                                      │
│  motor_driver.cpp                                   │
│    ├── L298N H-bridge GPIO direction control        │
│    ├── ESP32 LEDC PWM generation (20 kHz)           │
│    └── Active electronic brake                      │
│              │                                      │
│  encoder.cpp                                        │
│    ├── IRAM_ATTR ISRs on Phase A rising edges       │
│    ├── Phase B direction detection                  │
│    └── 50 ms rolling RPM calculation                │
│              │                                      │
│  config.h                                           │
│    └── All pin mappings, PWM, encoder, PID params   │
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
| Left Motor PWM | **GPIO 9** | ENA | Speed control (remove ENA jumper!) |
| Left Motor Dir 1 | **GPIO 7** | IN1 | Forward: HIGH |
| Left Motor Dir 2 | **GPIO 8** | IN2 | Forward: LOW |
| Right Motor PWM | **GPIO 4** | ENB | Speed control (remove ENB jumper!) |
| Right Motor Dir 1 | **GPIO 5** | IN3 | Forward: HIGH |
| Right Motor Dir 2 | **GPIO 6** | IN4 | Forward: LOW |

> **⚠️ Important**: Remove the black jumper caps from **ENA** and **ENB** on the L298N board. These jumpers bypass PWM speed control and lock motors to full speed.

### ESP32-S3 → Quadrature Encoders

| Signal | ESP32-S3 GPIO | Encoder Wire | Description |
|---|---|---|---|
| Left Encoder Phase A | **GPIO 12** | Channel A (Yellow) | Interrupt trigger (RISING) |
| Left Encoder Phase B | **GPIO 13** | Channel B (Green) | Direction sensing |
| Right Encoder Phase A | **GPIO 10** | Channel A (Yellow) | Interrupt trigger (RISING) |
| Right Encoder Phase B | **GPIO 11** | Channel B (Green) | Direction sensing |

### Power Connections

| Connection | Details |
|---|---|
| L298N 12V input | Battery pack positive (+) terminal |
| L298N GND | Battery pack negative (−) terminal **AND** ESP32-S3 GND |
| ESP32-S3 power | USB cable from host computer |
| L298N OUT1/OUT2 | Left motor terminals |
| L298N OUT3/OUT4 | Right motor terminals |

> **⚠️ Important**: The ESP32-S3 GND and L298N GND **must** be connected together (common ground). Without this, serial communication and PWM signals will be unreliable.

### ESP32-S3 → Raspberry Pi 4B GPIO Serial Connection (Hardware UART)

You can connect the Raspberry Pi 4B directly to the ESP32-S3 via the 40-pin GPIO header without needing a USB cable. Both boards share compatible **3.3V logic levels** (no level shifter required).

| Raspberry Pi 4B Pin | Raspberry Pi Function | ESP32-S3 Pin | ESP32-S3 Function |
|---|---|---|---|
| **Pin 8** (GPIO 14) | UART0 TXD | **GPIO 18** | `PIN_UART_RPI_RX` (Receive) |
| **Pin 10** (GPIO 15) | UART0 RXD | **GPIO 17** | `PIN_UART_RPI_TX` (Transmit) |
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
| I2C SDA | **GPIO 15** | SDA (shared) | Common I2C Data bus (can be changed to any free GPIO) |
| I2C SCL | **GPIO 16** | SCL (shared) | Common I2C Clock bus (can be changed to any free GPIO) |
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
├── README.md                          # ← This file
├── .gitignore
├── raspberrypi_host                   # Symlink to rasberrypi_host
│
├── esp32_firmware/                    # ESP32-S3 C++ firmware
│   ├── esp32_firmware.ino             # Arduino IDE entry point
│   ├── platformio.ini                 # PlatformIO project config
│   ├── src/                           # Source files (Arduino IDE compatible)
│   │   ├── config.h                   #   Pin mappings, PWM, encoder, ToF, PID params
│   │   ├── main.cpp                   #   Setup/loop, dual-serial parser, telemetry
│   │   ├── motor_driver.h             #   MotorDriver class declaration
│   │   ├── motor_driver.cpp           #   L298N H-bridge GPIO/PWM control
│   │   ├── encoder.h                  #   EncoderManager class declaration
│   │   ├── encoder.cpp                #   ISR-driven quadrature tick counting & RPM
│   │   ├── pid_controller.h           #   RobotController class declaration
│   │   ├── pid_controller.cpp         #   Drive state mapping, sync trim, arc turns
│   │   ├── distance_sensors.h         #   DistanceSensors class declaration (3x VL53L0X)
│   │   └── distance_sensors.cpp       #   Sequential XSHUT I2C init & continuous ranging
│   ├── include/                       # PlatformIO include path (mirrors src/ headers)
│   │   ├── config.h
│   │   ├── encoder.h
│   │   ├── motor_driver.h
│   │   ├── pid_controller.h
│   │   └── distance_sensors.h
│   └── tests/                         # Standalone test sketches
│       ├── distance_sensor_test.cpp
│       ├── motor_driver_test.cpp
│       └── vl53l0x_distance_sensors/  # Standalone 3x VL53L0X distance sensor test sketch
│           └── vl53l0x_distance_sensors.ino
│
├── teleop_host/                       # Standalone teleoperation & serial drivers (Python)
│   ├── README.md
│   ├── requirements.txt               # pyserial, pynput, rich
│   ├── teleop_controller.py           # Standalone keyboard teleop controller
│   ├── serial_interface.py            # Threaded serial TX/RX manager & heartbeat
│   └── telemetry_model.py             # TELE: packet data model & parser
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

#### Option A: Arduino IDE 2.x (Recommended for beginners)

1. **Install the ESP32 board package**:
   - Open Arduino IDE → **File → Preferences**
   - Add this URL to *Additional Board Manager URLs*:
     ```
     https://espressif.github.io/arduino-esp32/package_esp32_index.json
     ```
   - Go to **Tools → Board → Boards Manager** → Search `esp32` → Install **esp32 by Espressif Systems**

2. **Configure the board**:
   - **Tools → Board**: `ESP32S3 Dev Module`
   - **Tools → USB CDC On Boot**: `Enabled`
   - **Tools → USB Mode**: `Hardware CDC and JTAG`
   - **Tools → Upload Speed**: `921600`
   - **Tools → Port**: Select your ESP32's USB port (e.g., `/dev/ttyACM0`)

3. **Open and upload**:
   - Open `esp32_firmware/esp32_firmware.ino` in Arduino IDE
   - Click **Upload** (→ button)
   - Wait for "Done uploading" message

#### Option B: PlatformIO (VS Code)

1. Install the [PlatformIO IDE extension](https://platformio.org/install/ide?install=vscode) for VS Code

2. Open the firmware folder and build:
   ```bash
   cd esp32_firmware
   pio run --target upload
   pio device monitor    # Optional: view serial output
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

---

## Telemetry Protocol

### Host → ESP32 (Command Packets)
```
CMD:<DIRECTION>,<SPEED>\n
CMD:LR,<LEFT_PWM>,<RIGHT_PWM>\n
```
| Format | Description | Example |
|---|---|---|
| `CMD:<DIR>,<SPEED>` | Directional drive & arc commands | `CMD:W,200\n`, `CMD:WA,200\n`, `CMD:B,0\n` |
| `CMD:LR,<L>,<R>` | Differential drive from state machine (-255 to +255) | `CMD:LR,180,180\n`, `CMD:LR,140,210\n` |

Example: `CMD:WA,200\n` → Forward-left arc at PWM 200

### ESP32 → Host (Telemetry Packets)
```
TELE:<rpmL>,<rpmR>,<ticksL>,<ticksR>,<state>,<flags>\n
```
| Field | Type | Description |
|---|---|---|
| rpmL | float | Left wheel RPM |
| rpmR | float | Right wheel RPM |
| ticksL | int64 | Left wheel cumulative encoder ticks |
| ticksR | int64 | Right wheel cumulative encoder ticks |
| state | string | Current drive state (`FWD`, `REV`, `LEFT`, `RIGHT`, `FWD_L`, `FWD_R`, `STOP`, `BRAKE`) |
| flags | string | `OK` or `WDG` (watchdog tripped) |

Example: `TELE:45.2,44.8,13200,13180,FWD,OK\n`

---

## Configuration Reference

All firmware tuning parameters are in [`esp32_firmware/src/config.h`](esp32_firmware/src/config.h):

| Parameter | Default | Description |
|---|---|---|
| `DEFAULT_DRIVE_SPEED` | `200` | Default PWM when speed not specified (0–255) |
| `DEFAULT_TURN_SPEED` | `180` | PWM for single-wheel pivot turns |
| `PWM_FREQUENCY` | `20000` Hz | Ultrasonic PWM frequency (eliminates motor whine) |
| `WHEEL_PULSES_PER_REV` | `330.0` | Pulses per full revolution (turn) of the wheel |
| `SYNC_KP` | `0.5` | Straight-line proportional sync gain |
| `MAX_SYNC_TRIM_PWM` | `40` | Maximum correction applied per wheel (±40 PWM) |
| `WATCHDOG_TIMEOUT_MS` | `350` ms | Auto-stop if no command received within this window |
| `TELEMETRY_INTERVAL_MS` | `50` ms | Telemetry broadcast interval (20 Hz) |
| `INVERT_LEFT_ENCODER` | `true` | Flip left encoder counting direction |
| `INVERT_RIGHT_ENCODER` | `false` | Flip right encoder counting direction |

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

### Robot drifts to one side during forward drive
- The encoder sync should handle this automatically. If drift is severe:
  - Increase `SYNC_KP` in `config.h` (try `1.0`)
  - Increase `MAX_SYNC_TRIM_PWM` (try `60`)
- If a wheel reads negative RPM when driving forward, toggle `INVERT_LEFT_ENCODER` or `INVERT_RIGHT_ENCODER` in `config.h`

### Dashboard shows "DISCONNECTED" or "WDG" (Watchdog Tripped)
- The ESP32 may not be sending telemetry — open Arduino Serial Monitor at 115200 baud to verify
- Check that the correct serial port is selected
- Ensure no other program (Arduino Serial Monitor, screen, minicom) is holding the port open

### Permission denied on serial port
```bash
sudo usermod -aG dialout $USER
# Then log out and log back in
```

### `pynput` import error in SSH / headless terminal
- This is expected. The controller automatically falls back to raw terminal mode (`termios`/`tty`)
- Multi-key detection still works via the 450 ms key-decay window

---

## License

This project is developed as part of an academic/personal robotics project. License TBD.
