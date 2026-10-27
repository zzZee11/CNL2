"""
autonomous_navigation/serial_comm.py
------------------------------------
Bidirectional serial communication with ESP32-S3 over Raspberry Pi UART (/dev/serial0).
- TX thread (20 Hz): Continuously transmits speed commands <left_mps>,<right_mps>\n to feed the watchdog.
- RX thread: Reads encoder telemetry ENC,<leftTicks>,<rightTicks>,... and computes continuous (x, y, theta) odometry.
- Startup kick: Provides a 180 ms high-torque boost to break static friction whenever moving from a stop.
"""

import time
import math
import threading
import serial

try:
    from .config import (
        SERIAL_PORT, SERIAL_BAUD, TX_RATE_HZ,
        WHEEL_RADIUS_M, WHEEL_BASE_M, TICKS_PER_REV,
        STARTUP_PUSH_SPEED, STARTUP_PUSH_DURATION_S,
        MAX_ALLOWED_SPEED
    )
except ImportError:
    from config import (
        SERIAL_PORT, SERIAL_BAUD, TX_RATE_HZ,
        WHEEL_RADIUS_M, WHEEL_BASE_M, TICKS_PER_REV,
        STARTUP_PUSH_SPEED, STARTUP_PUSH_DURATION_S,
        MAX_ALLOWED_SPEED
    )

class RobotSerialComm:
    def __init__(self, port=SERIAL_PORT, baud=SERIAL_BAUD, use_mock=False):
        self.port = port
        self.baud = baud
        self.use_mock = use_mock
        self.ser = None
        self.running = False

        # Commanded wheel speeds (m/s)
        self.cmd_left = 0.0
        self.cmd_right = 0.0
        self.push_until = 0.0
        self.cmd_lock = threading.Lock()

        # Telemetry & Odometry State
        self.left_ticks = 0
        self.right_ticks = 0
        self.left_rpm = 0.0
        self.right_rpm = 0.0
        self.has_first_encoder = False

        self.prev_left_ticks = 0
        self.prev_right_ticks = 0

        # Robot Pose in local coordinate frame (meters and radians)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.total_distance = 0.0

        self.state_lock = threading.Lock()
        self.tx_thread = None
        self.rx_thread = None

    def connect(self):
        """Opens the hardware serial port or initializes mock serial."""
        if self.use_mock:
            print("[Serial] Running in MOCK / SIMULATION mode.")
            self.has_first_encoder = True
            return True

        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=0.05,
                write_timeout=0.5
            )
            print(f"[Serial] Connected to ESP32 on {self.port} at {self.baud} baud.")
            time.sleep(1.0)
            return True
        except serial.SerialException as e:
            print(f"[Serial] Could not open {self.port}: {e}")
            print("[Serial] Falling back to MOCK mode for simulation.")
            self.use_mock = True
            self.has_first_encoder = True
            return True

    def start(self):
        """Starts background communication threads."""
        self.running = True
        self.tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.tx_thread.start()
        self.rx_thread.start()

    def set_speeds(self, left, right):
        """Sets target wheel speeds in m/s with automatic breakaway kick if starting from rest."""
        left = max(-MAX_ALLOWED_SPEED, min(MAX_ALLOWED_SPEED, float(left)))
        right = max(-MAX_ALLOWED_SPEED, min(MAX_ALLOWED_SPEED, float(right)))

        with self.cmd_lock:
            was_stopped = (abs(self.cmd_left) < 0.01 and abs(self.cmd_right) < 0.01)
            is_moving = (abs(left) > 0.01 or abs(right) > 0.01)

            # Trigger 180 ms startup kick to break static friction
            if was_stopped and is_moving:
                self.push_until = time.time() + STARTUP_PUSH_DURATION_S

            self.cmd_left = left
            self.cmd_right = right

    def stop(self):
        """Immediately commands zero speed."""
        with self.cmd_lock:
            self.push_until = 0.0
            self.cmd_left = 0.0
            self.cmd_right = 0.0

    def reset_odometry(self, start_x=0.0, start_y=0.0, start_theta=0.0):
        """Resets the dead reckoning pose to a known starting coordinate."""
        with self.state_lock:
            self.x = float(start_x)
            self.y = float(start_y)
            self.theta = float(start_theta)
            self.total_distance = 0.0
            self.prev_left_ticks = self.left_ticks
            self.prev_right_ticks = self.right_ticks

    def get_pose(self):
        """Returns (x, y, theta, total_distance_travelled)."""
        with self.state_lock:
            return self.x, self.y, self.theta, self.total_distance

    def _update_odometry(self, l_ticks, r_ticks):
        """Computes differential drive dead-reckoning from encoder tick changes."""
        with self.state_lock:
            if not self.has_first_encoder:
                self.prev_left_ticks = l_ticks
                self.prev_right_ticks = r_ticks
                self.has_first_encoder = True
                return

            delta_l = l_ticks - self.prev_left_ticks
            delta_r = r_ticks - self.prev_right_ticks
            self.prev_left_ticks = l_ticks
            self.prev_right_ticks = r_ticks

            # Distance moved by each wheel (meters)
            dist_l = (delta_l / TICKS_PER_REV) * (2.0 * math.pi * WHEEL_RADIUS_M)
            dist_r = (delta_r / TICKS_PER_REV) * (2.0 * math.pi * WHEEL_RADIUS_M)

            # Center displacement and heading change
            delta_s = (dist_l + dist_r) / 2.0
            delta_theta = (dist_r - dist_l) / WHEEL_BASE_M

            # Midpoint integration for smooth curved paths
            mid_theta = self.theta + (delta_theta / 2.0)
            self.x += delta_s * math.cos(mid_theta)
            self.y += delta_s * math.sin(mid_theta)
            self.theta = math.atan2(math.sin(self.theta + delta_theta), math.cos(self.theta + delta_theta))
            self.total_distance += abs(delta_s)

    def _tx_loop(self):
        """Transmits 20 Hz velocity packets to feed the ESP32 safety watchdog."""
        interval = 1.0 / TX_RATE_HZ
        while self.running:
            t0 = time.time()
            with self.cmd_lock:
                l = self.cmd_left
                r = self.cmd_right
                pushing = time.time() < self.push_until

            # Apply startup breakaway kick if active
            if pushing:
                out_l = (STARTUP_PUSH_SPEED if l > 0 else -STARTUP_PUSH_SPEED) if abs(l) > 0.01 else 0.0
                out_r = (STARTUP_PUSH_SPEED if r > 0 else -STARTUP_PUSH_SPEED) if abs(r) > 0.01 else 0.0
            else:
                out_l = l
                out_r = r

            if not self.use_mock and self.ser and self.ser.is_open:
                try:
                    msg = f"{out_l:.2f},{out_r:.2f}\n".encode("ascii")
                    self.ser.write(msg)
                    self.ser.flush()
                except (serial.SerialException, OSError):
                    pass

            elapsed = time.time() - t0
            sleep_t = interval - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    def _rx_loop(self):
        """Reads incoming telemetry packets and updates odometry."""
        while self.running:
            if self.use_mock:
                # Simulated odometry progression based on commanded speed
                time.sleep(0.05)
                with self.cmd_lock:
                    l = self.cmd_left
                    r = self.cmd_right
                if abs(l) > 0.01 or abs(r) > 0.01:
                    delta_ticks_l = int(l * 0.05 * TICKS_PER_REV / (2.0 * math.pi * WHEEL_RADIUS_M))
                    delta_ticks_r = int(r * 0.05 * TICKS_PER_REV / (2.0 * math.pi * WHEEL_RADIUS_M))
                    with self.state_lock:
                        self.left_ticks += delta_ticks_l
                        self.right_ticks += delta_ticks_r
                        curr_l = self.left_ticks
                        curr_r = self.right_ticks
                    self._update_odometry(curr_l, curr_r)
                continue

            try:
                if not self.ser or not self.ser.is_open:
                    time.sleep(0.05)
                    continue

                line = self.ser.readline().decode("ascii", errors="ignore").strip()
                if not line:
                    continue

                parts = [p.strip() for p in line.split(",")]
                if parts and parts[0] == "ENC" and len(parts) >= 3:
                    l_ticks = int(parts[1])
                    r_ticks = int(parts[2])
                    with self.state_lock:
                        self.left_ticks = l_ticks
                        self.right_ticks = r_ticks
                        if len(parts) >= 5:
                            self.left_rpm = float(parts[3])
                            self.right_rpm = float(parts[4])

                    self._update_odometry(l_ticks, r_ticks)

            except (ValueError, serial.SerialException):
                time.sleep(0.01)

    def close(self):
        """Stops the robot and closes serial connection."""
        self.running = False
        self.stop()
        if not self.use_mock and self.ser and self.ser.is_open:
            try:
                for _ in range(3):
                    self.ser.write(b"0.00,0.00\n")
                    self.ser.flush()
                    time.sleep(0.04)
                self.ser.close()
            except Exception:
                pass
        print("[Serial] Port closed. Robot stopped.")
