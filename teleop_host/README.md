# Robot Car Teleoperation & Telemetry Host

Interactive keyboard teleoperation controller and live telemetry dashboard for the ESP32-S3 robot car.

## Features
- **WASD Teleoperation**:
  - `W`: Drive forward
  - `S`: Drive reverse
  - `A`: Pivot turn left
  - `D`: Pivot turn right
  - `Space`: Active brake
  - `Release`: Coast stop
  - `+/-`: Adjust drive PWM speed (50 to 255)
  - `Q`: Quit
- **Watchdog Heartbeat**: Continuously streams `CMD:...` packets at 20 Hz to feed the ESP32's 300 ms safety watchdog.
- **Real-Time Rich Dashboard**:
  - Live RPM gauge and velocity readouts for left and right wheels
  - Cumulative quadrature encoder ticks
  - Watchdog and connection status indicators
  - Fallback support for both local GUI desktop (`pynput`) and headless SSH terminals (`termios`/`tty`)

## Installation & Setup

1. Install requirements:
   ```bash
   pip install -r requirements.txt
   ```

2. Run teleoperation controller:
   ```bash
   python3 teleop_controller.py
   ```

   To specify a specific serial port or speed:
   ```bash
   python3 teleop_controller.py --port /dev/ttyACM0 --speed 220
   ```

