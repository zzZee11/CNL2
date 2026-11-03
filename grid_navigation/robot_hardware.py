"""
grid_navigation/robot_hardware.py
---------------------------------
Unified hardware abstraction layer for CNL2 robot.
Integrates ESP32 UART serial comms, MPU-6050 spatial IMU, and 3x VL53L0X ToF distance sensors,
with native mock simulation support.
"""

import os
import time
import math
import struct
import threading
from typing import Tuple, List, Optional
from collections import deque
import statistics

try:
    from .config import (
        SERIAL_PORT, SERIAL_BAUD, TX_RATE_HZ,
        WHEEL_RADIUS_M, WHEEL_BASE_M, TICKS_PER_REV,
        IMU_I2C_BUS, IMU_I2C_ADDR,
        XSHUT_LEFT, XSHUT_MIDDLE, XSHUT_RIGHT,
        MAX_SPEED, TOF_MIN_VALID_M, TOF_MAX_VALID_M
    )
except ImportError:
    from config import (
        SERIAL_PORT, SERIAL_BAUD, TX_RATE_HZ,
        WHEEL_RADIUS_M, WHEEL_BASE_M, TICKS_PER_REV,
        IMU_I2C_BUS, IMU_I2C_ADDR,
        XSHUT_LEFT, XSHUT_MIDDLE, XSHUT_RIGHT,
        MAX_SPEED, TOF_MIN_VALID_M, TOF_MAX_VALID_M
    )


class RobotHardware:
    """
    Unified hardware controller for Motors, Encoders, Gyro, and Distance Sensors.
    Thread-safe and supports mock mode for desktop testing.
    """

    def __init__(
        self,
        port: str = SERIAL_PORT,
        baud: int = SERIAL_BAUD,
        use_mock: bool = False
    ):
        self.port = port
        self.baud = baud
        self.use_mock = use_mock
        self.running = False

        # Serial & Thread Locks
        self.ser = None
        self.cmd_lock = threading.Lock()
        self.state_lock = threading.Lock()

        # Commanded Wheel Speeds (m/s)
        self.cmd_left = 0.0
        self.cmd_right = 0.0

        # Odometry State (Metric & Radians)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.total_dist = 0.0

        self.left_ticks = 0
        self.right_ticks = 0
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.has_first_encoder = False

        # IMU Gyro State
        self.has_imu = False
        self.imu_i2c = None
        self.gyro_bias_z_dps = 0.0
        self.gyro_yaw_rate_rad_s = 0.0
        self.imu_heading_rad = 0.0
        self.last_imu_time = time.monotonic()

        # Distance Sensors State (3x VL53L0X)
        self.has_tof = False
        self.tof_sensors = []
        self.tof_windows = [deque([9.99] * 5, maxlen=5) for _ in range(3)]
        self.mock_distances = [9.99, 9.99, 9.99]

        # Background Threads
        self.tx_thread = None
        self.rx_thread = None

    def initialize(self) -> bool:
        """Initializes serial connection, IMU, and ToF sensors."""
        if self.use_mock:
            print("[Hardware] Initializing in MOCK / SIMULATION mode.")
            self.has_first_encoder = True
            self.running = True
            return True

        # 1. Connect Serial to ESP32
        try:
            import serial
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=0.05,
                write_timeout=0.5
            )
            print(f"[Hardware] Connected to ESP32 on {self.port} at {self.baud} baud.")
            time.sleep(0.5)
        except Exception as e:
            print(f"[Hardware] ESP32 connection failed: {e}. Switching to MOCK mode.")
            self.use_mock = True
            self.has_first_encoder = True
            self.running = True
            return True

        # 2. Connect MPU-6050 IMU
        self._init_imu()

        # 3. Connect VL53L0X ToF Sensors
        self._init_tof()

        self.running = True
        return True

    def start(self) -> None:
        """Starts 20 Hz communication and telemetry loop threads."""
        self.running = True
        self.tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.tx_thread.start()
        self.rx_thread.start()

    def set_speeds(self, left_mps: float, right_mps: float) -> None:
        """Sets target wheel speeds in m/s with safety clamping."""
        left = max(-MAX_SPEED, min(MAX_SPEED, float(left_mps)))
        right = max(-MAX_SPEED, min(MAX_SPEED, float(right_mps)))
        with self.cmd_lock:
            self.cmd_left = left
            self.cmd_right = right

    def stop(self) -> None:
        """Halts motors immediately."""
        with self.cmd_lock:
            self.cmd_left = 0.0
            self.cmd_right = 0.0

    def active_brake(self, reverse_speed: float = 0.16, duration_s: float = 0.035) -> None:
        """
        Applies a brief counter-torque pulse opposite to the current travel direction
        to cancel robot kinetic momentum instantly, then cuts command to zero.
        """
        if self.use_mock:
            self.stop()
            return

        with self.cmd_lock:
            sgn_l = -1.0 if self.cmd_left > 0.02 else (1.0 if self.cmd_left < -0.02 else 0.0)
            sgn_r = -1.0 if self.cmd_right > 0.02 else (1.0 if self.cmd_right < -0.02 else 0.0)
            if sgn_l != 0.0 or sgn_r != 0.0:
                self.cmd_left = sgn_l * reverse_speed
                self.cmd_right = sgn_r * reverse_speed
            else:
                self.cmd_left = 0.0
                self.cmd_right = 0.0

        if self.ser and self.ser.is_open:
            try:
                msg = f"{self.cmd_left:.2f},{self.cmd_right:.2f}\n".encode("ascii")
                self.ser.write(msg)
                self.ser.flush()
            except Exception:
                pass

        time.sleep(duration_s)

        with self.cmd_lock:
            self.cmd_left = 0.0
            self.cmd_right = 0.0

        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b"0.00,0.00\n")
                self.ser.flush()
            except Exception:
                pass

    def reset_pose(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> None:
        """Resets the dead-reckoning pose to (x, y, theta)."""
        with self.state_lock:
            self.x = float(x)
            self.y = float(y)
            self.theta = float(theta)
            self.total_dist = 0.0
            self.prev_left_ticks = self.left_ticks
            self.prev_right_ticks = self.right_ticks
            self.imu_heading_rad = float(theta)

    def get_pose(self) -> Tuple[float, float, float, float]:
        """Returns current pose (x, y, theta_rad, total_distance_m)."""
        with self.state_lock:
            return self.x, self.y, self.theta, self.total_dist

    def get_tof_distances(self) -> List[float]:
        """Returns filtered distances [left_m, middle_m, right_m]."""
        if self.use_mock:
            return list(self.mock_distances)

        # In physical mode, read real VL53L0X sensors with rolling median filtering
        readings = [9.99, 9.99, 9.99]
        for i, sensor in enumerate(self.tof_sensors):
            try:
                mm = sensor.range
                m = mm / 1000.0
                if TOF_MIN_VALID_M <= m <= TOF_MAX_VALID_M:
                    self.tof_windows[i].append(m)
                else:
                    self.tof_windows[i].append(9.99)
                readings[i] = float(statistics.median(self.tof_windows[i]))
            except Exception:
                readings[i] = 9.99
        return readings

    def get_yaw_rate(self) -> float:
        """Returns current angular velocity in rad/s from gyro (or 0.0)."""
        with self.state_lock:
            return self.gyro_yaw_rate_rad_s

    def recalibrate_gyro_bias(self, num_samples: int = 30) -> None:
        """Samples resting gyro to re-zero drift bias while preserving current heading."""
        if self.use_mock or not self.has_imu:
            return

        sum_gz = 0.0
        count = 0
        for _ in range(num_samples):
            try:
                # Read 2 bytes from MPU6050 Gyro Z (0x47)
                raw = self.imu_i2c.read_i2c_block_data(IMU_I2C_ADDR, 0x47, 2)
                gz_r = struct.unpack('>h', bytes(raw))[0]
                sum_gz += (gz_r / 131.0)
                count += 1
            except Exception:
                pass
            time.sleep(0.01)

        if count > 0:
            with self.state_lock:
                self.gyro_bias_z_dps = sum_gz / count

    def close(self) -> None:
        """Closes all hardware connections and stops motors."""
        self.running = False
        self.stop()
        if not self.use_mock and self.ser and self.ser.is_open:
            try:
                for _ in range(3):
                    self.ser.write(b"0.00,0.00\n")
                    self.ser.flush()
                    time.sleep(0.03)
                self.ser.close()
            except Exception:
                pass
        print("[Hardware] Hardware connection closed.")

    # -------------------------------------------------------------
    # INTERNAL HARDWARE INITIALIZATION & LOOPS
    # -------------------------------------------------------------
    def _init_imu(self) -> None:
        try:
            import smbus2
            self.imu_i2c = smbus2.SMBus(IMU_I2C_BUS)
            # Wake MPU-6050
            self.imu_i2c.write_byte_data(IMU_I2C_ADDR, 0x6B, 0x01)
            time.sleep(0.05)
            # Gyro range ±250 deg/s
            self.imu_i2c.write_byte_data(IMU_I2C_ADDR, 0x1B, 0x00)
            self.has_imu = True
            self.recalibrate_gyro_bias(num_samples=30)
            print("[Hardware] MPU-6050 gyro initialized.")
        except Exception as e:
            print(f"[Hardware] IMU init warning: {e}. Falling back to encoder odometry.")
            self.has_imu = False

    def _init_tof(self) -> None:
        try:
            import board
            import digitalio
            import adafruit_vl53l0x
            i2c_bus = board.I2C()

            # Sequential XSHUT readdressing
            pins = [digitalio.DigitalInOut(p) for p in (board.D17, board.D22, board.D27)]
            for p in pins:
                p.switch_to_output(value=False)
            time.sleep(0.05)

            addrs = [0x30, 0x31, 0x32]
            for i, p in enumerate(pins):
                p.value = True
                time.sleep(0.02)
                sensor = adafruit_vl53l0x.VL53L0X(i2c_bus)
                sensor.set_address(addrs[i])
                self.tof_sensors.append(sensor)
            self.has_tof = True
            print("[Hardware] 3x VL53L0X ToF sensors initialized.")
        except Exception as e:
            print(f"[Hardware] ToF sensor init warning: {e}. Distance mapping disabled.")
            self.has_tof = False

    def _update_odometry(self, l_ticks: int, r_ticks: int) -> None:
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

            # Metric distance traveled per wheel
            dist_l = (delta_l / TICKS_PER_REV) * (2.0 * math.pi * WHEEL_RADIUS_M)
            dist_r = (delta_r / TICKS_PER_REV) * (2.0 * math.pi * WHEEL_RADIUS_M)
            delta_s = (dist_l + dist_r) / 2.0

            if self.has_imu and not self.use_mock:
                # Fused with physical gyro
                mid_theta = self.theta
                self.theta = self.imu_heading_rad
            else:
                # Differential wheel kinematics
                delta_theta = (dist_r - dist_l) / WHEEL_BASE_M
                mid_theta = self.theta + (delta_theta / 2.0)
                self.theta = math.atan2(math.sin(self.theta + delta_theta), math.cos(self.theta + delta_theta))

            self.x += delta_s * math.cos(mid_theta)
            self.y += delta_s * math.sin(mid_theta)
            self.total_dist += abs(delta_s)

    def _tx_loop(self) -> None:
        interval = 1.0 / TX_RATE_HZ
        while self.running:
            t0 = time.time()
            with self.cmd_lock:
                l = self.cmd_left
                r = self.cmd_right

            if not self.use_mock and self.ser and self.ser.is_open:
                try:
                    msg = f"{l:.2f},{r:.2f}\n".encode("ascii")
                    self.ser.write(msg)
                    self.ser.flush()
                except Exception:
                    pass

            elapsed = time.time() - t0
            sleep_t = interval - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    def _rx_loop(self) -> None:
        while self.running:
            if self.use_mock:
                # Simulated odometry progression based on commanded speeds
                time.sleep(0.05)
                with self.cmd_lock:
                    l = self.cmd_left
                    r = self.cmd_right

                if abs(l) > 0.005 or abs(r) > 0.005:
                    delta_ticks_l = int(l * 0.05 * TICKS_PER_REV / (2.0 * math.pi * WHEEL_RADIUS_M))
                    delta_ticks_r = int(r * 0.05 * TICKS_PER_REV / (2.0 * math.pi * WHEEL_RADIUS_M))
                    with self.state_lock:
                        self.left_ticks += delta_ticks_l
                        self.right_ticks += delta_ticks_r
                        curr_l = self.left_ticks
                        curr_r = self.right_ticks
                    self._update_odometry(curr_l, curr_r)
                continue

            # Read physical ESP32 telemetry packet "ENC,<leftTicks>,<rightTicks>\n"
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
                    self._update_odometry(l_ticks, r_ticks)

                # Poll physical MPU-6050 gyro if present
                if self.has_imu and self.imu_i2c:
                    now = time.monotonic()
                    dt = max(0.001, min(0.10, now - self.last_imu_time))
                    self.last_imu_time = now
                    raw = self.imu_i2c.read_i2c_block_data(IMU_I2C_ADDR, 0x47, 2)
                    gz_r = struct.unpack('>h', bytes(raw))[0]
                    gz_dps = (gz_r / 131.0) - self.gyro_bias_z_dps
                    if abs(gz_dps) < 0.25:  # Deadband
                        gz_dps = 0.0
                    gz_rad_s = math.radians(gz_dps)
                    with self.state_lock:
                        self.gyro_yaw_rate_rad_s = gz_rad_s
                        self.imu_heading_rad += gz_rad_s * dt
                        self.imu_heading_rad = math.atan2(
                            math.sin(self.imu_heading_rad),
                            math.cos(self.imu_heading_rad)
                        )

            except Exception:
                time.sleep(0.01)

