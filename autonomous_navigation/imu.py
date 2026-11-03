#!/usr/bin/env python3
"""
CNL2 Autonomous Navigation — Inertial Measurement Unit (IMU) Module
===================================================================
Provides real-time orientation tracking, gyro turn-rate feedback,
and tilt calculation using an MPU-6050 (GY-521) 6-DOF sensor.

Features:
  - Multi-threaded 50 Hz background polling loop.
  - Automatic zero-rate gyro drift bias calibration at startup.
  - Zero-drift stationary deadband filter.
  - Continuous yaw heading tracking (rad & deg) normalized to [-π, +π].
  - Real-time angular velocity (yaw-rate) for steering damping.
  - Static mounting tilt (pitch/roll) calculation from gravity vector.
  - Collision / impact shock spike detection.
  - Safe automatic fallback to MOCK mode when hardware is not present.
"""

import os
import time
import math
import struct
import threading

# Register definitions
MPU6050_ADDR_PRIMARY = 0x68
MPU6050_ADDR_ALT     = 0x69

REG_SMPLRT_DIV   = 0x19
REG_CONFIG       = 0x1A
REG_GYRO_CONFIG  = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_ACCEL_XOUT_H = 0x3B
REG_PWR_MGMT_1   = 0x6B
REG_WHO_AM_I     = 0x75

ACCEL_SCALE_LSB_PER_G  = 16384.0
GYRO_SCALE_LSB_PER_DPS = 131.0
GRAVITY_MPS2           = 9.80665


class I2CInterface:
    """Lightweight I2C communication interface with smbus fallback."""

    def __init__(self, bus_num=1, address=MPU6050_ADDR_PRIMARY):
        self.bus_num = bus_num
        self.address = address
        self.fd = None
        self.use_smbus = False
        self.bus = None

        try:
            import smbus2 as smbus_module
            self.bus = smbus_module.SMBus(self.bus_num)
            self.use_smbus = True
        except ImportError:
            try:
                import smbus as smbus_module
                self.bus = smbus_module.SMBus(self.bus_num)
                self.use_smbus = True
            except ImportError:
                self.use_smbus = False

    def open(self):
        if self.use_smbus:
            return True

        import fcntl
        dev_path = f"/dev/i2c-{self.bus_num}"
        if not os.path.exists(dev_path):
            raise FileNotFoundError(f"I2C device '{dev_path}' not available.")

        self.fd = os.open(dev_path, os.O_RDWR)
        # 0x0703 is I2C_SLAVE
        fcntl.ioctl(self.fd, 0x0703, self.address)
        return True

    def write_byte(self, reg, val):
        if self.use_smbus:
            self.bus.write_byte_data(self.address, reg, val)
        else:
            os.write(self.fd, bytes([reg, val & 0xFF]))

    def read_byte(self, reg):
        if self.use_smbus:
            return self.bus.read_byte_data(self.address, reg)
        else:
            os.write(self.fd, bytes([reg]))
            buf = os.read(self.fd, 1)
            return buf[0]

    def read_block(self, reg, length):
        if self.use_smbus:
            return self.bus.read_i2c_block_data(self.address, reg, length)
        else:
            os.write(self.fd, bytes([reg]))
            buf = os.read(self.fd, length)
            return list(buf)

    def close(self):
        if self.use_smbus and self.bus:
            try:
                self.bus.close()
            except Exception:
                pass
        elif self.fd is not None:
            try:
                os.close(self.fd)
            except Exception:
                pass
            self.fd = None


class IMUManager:
    """
    Manages continuous orientation and inertial tracking from MPU-6050.
    Thread-safe and supports automatic mock fallback.
    """

    def __init__(self, bus_num=1, address=MPU6050_ADDR_PRIMARY, sample_rate_hz=50.0, use_mock=False):
        self.bus_num = bus_num
        self.address = address
        self.sample_rate_hz = sample_rate_hz
        self.interval_s = 1.0 / max(1.0, sample_rate_hz)
        self.use_mock = use_mock

        self.i2c = I2CInterface(bus_num=bus_num, address=address)
        self.lock = threading.Lock()
        self.running = False
        self.thread = None

        # Static Biases (calculated during resting calibration)
        self.bias_gx_dps = 0.0
        self.bias_gy_dps = 0.0
        self.bias_gz_dps = 0.0

        # Mounting tilt angles (degrees)
        self.tilt_pitch_deg = 0.0
        self.tilt_roll_deg = 0.0

        # Live State
        self.heading_rad = 0.0
        self.yaw_rate_rad_s = 0.0
        self.yaw_rate_dps = 0.0
        self.accel_g = (0.0, 0.0, 1.0)
        self.accel_mps2 = (0.0, 0.0, GRAVITY_MPS2)
        self.temp_c = 25.0
        self.last_update_time = time.monotonic()
        self.healthy = False
        self.collision_detected = False

        # Configurable thresholds
        self.deadband_dps = 0.25         # Ignore rotational noise below this (deg/s)
        self.collision_threshold_g = 1.8 # Accel spike indicating physical bump/collision

    def initialize(self, calibrate_samples=60):
        """Initializes hardware connection and calibrates zero-rate bias."""
        if self.use_mock:
            print("[IMU] Initializing in MOCK / SIMULATION mode.")
            self.healthy = True
            return True

        detected = False
        for addr in (self.address, MPU6050_ADDR_ALT if self.address == MPU6050_ADDR_PRIMARY else MPU6050_ADDR_PRIMARY):
            try:
                self.i2c.address = addr
                self.i2c.open()
                who = self.i2c.read_byte(REG_WHO_AM_I)
                if who in (0x68, 0x70, 0x71, 0x72):
                    self.address = addr
                    detected = True
                    break
                self.i2c.close()
            except Exception:
                if self.i2c.fd is not None:
                    self.i2c.close()

        if not detected:
            print("[IMU] MPU-6050 not detected on I2C bus. Switching to MOCK mode.")
            self.use_mock = True
            self.healthy = True
            return False

        try:
            # Wake up chip and set clock source to Gyro PLL for reduced drift
            self.i2c.write_byte(REG_PWR_MGMT_1, 0x01)
            time.sleep(0.04)

            # Set DLPF to ~42 Hz (DLPF_CFG = 3) to dampen high-frequency motor vibration
            self.i2c.write_byte(REG_CONFIG, 0x03)

            # Sample rate divider -> 1 kHz / (1 + 9) = 100 Hz internal sampling
            self.i2c.write_byte(REG_SMPLRT_DIV, 0x09)

            # Gyro range: ±250 °/s (0x00) -> 131.0 LSB/(°/s)
            self.i2c.write_byte(REG_GYRO_CONFIG, 0x00)

            # Accel range: ±2g (0x00) -> 16384.0 LSB/g
            self.i2c.write_byte(REG_ACCEL_CONFIG, 0x00)
            time.sleep(0.04)

            # Run baseline calibration
            self._calibrate(num_samples=calibrate_samples)
            self.healthy = True
            print(f"[IMU] MPU-6050 initialized at 0x{self.address:02X} (Pitch: {self.tilt_pitch_deg:+.1f}°, Roll: {self.tilt_roll_deg:+.1f}°)")
            return True

        except Exception as e:
            print(f"[IMU] Hardware setup failed ({e}). Switching to MOCK mode.")
            self.use_mock = True
            self.healthy = True
            return False

    def calibrate(self, num_samples=60, quiet=False):
        """Samples resting values to calculate gyro zero-drift bias and mounting tilt."""
        if self.use_mock:
            self.healthy = True
            return True

        if not quiet:
            print(f"[IMU] Calibrating resting gyro bias ({num_samples} samples)...")

        sum_gx = 0.0
        sum_gy = 0.0
        sum_gz = 0.0
        sum_ax = 0.0
        sum_ay = 0.0
        sum_az = 0.0

        for _ in range(num_samples):
            try:
                raw_bytes = self.i2c.read_block(REG_ACCEL_XOUT_H, 14)
                ax_r, ay_r, az_r, _, gx_r, gy_r, gz_r = struct.unpack('>7h', bytes(raw_bytes))

                sum_ax += (ax_r / ACCEL_SCALE_LSB_PER_G)
                sum_ay += (ay_r / ACCEL_SCALE_LSB_PER_G)
                sum_az += (az_r / ACCEL_SCALE_LSB_PER_G)
                sum_gx += (gx_r / GYRO_SCALE_LSB_PER_DPS)
                sum_gy += (gy_r / GYRO_SCALE_LSB_PER_DPS)
                sum_gz += (gz_r / GYRO_SCALE_LSB_PER_DPS)
            except Exception:
                pass
            time.sleep(0.015)

        n = max(1, num_samples)
        self.bias_gx_dps = sum_gx / n
        self.bias_gy_dps = sum_gy / n
        self.bias_gz_dps = sum_gz / n

        avg_ax = sum_ax / n
        avg_ay = sum_ay / n
        avg_az = sum_az / n

        # Pitch and roll tilt angles calculated from static gravity components
        self.tilt_pitch_deg = math.atan2(avg_ax, math.sqrt(avg_ay**2 + avg_az**2)) * (180.0 / math.pi)
        self.tilt_roll_deg  = math.atan2(avg_ay, math.sqrt(avg_ax**2 + avg_az**2)) * (180.0 / math.pi)

        if not quiet:
            print(f"[IMU] Calibration ready: Gyro bias Z = {self.bias_gz_dps:+.3f}°/s | Pitch: {self.tilt_pitch_deg:+.1f}°, Roll: {self.tilt_roll_deg:+.1f}°")
        return True

    def _calibrate(self, num_samples=60):
        return self.calibrate(num_samples=num_samples, quiet=True)

    def quick_recalibrate_bias(self, num_samples=25):
        """
        Samples resting Z-axis angular velocity to re-zero gyro drift bias (Solution 4)
        while strictly preserving the current accumulated yaw heading.
        """
        if self.use_mock:
            return True

        sum_gz = 0.0
        samples_taken = 0
        for _ in range(num_samples):
            try:
                raw_bytes = self.i2c.read_block(REG_ACCEL_XOUT_H, 14)
                _, _, _, _, _, _, gz_r = struct.unpack('>7h', bytes(raw_bytes))
                sum_gz += (gz_r / GYRO_SCALE_LSB_PER_DPS)
                samples_taken += 1
            except Exception:
                pass
            time.sleep(0.01)

        if samples_taken > 0:
            with self.lock:
                self.bias_gz_dps = sum_gz / samples_taken
        return True

    def start(self):
        """Starts background 50 Hz polling thread."""
        self.running = True
        self.last_update_time = time.monotonic()
        self.thread = threading.Thread(target=self._poll_loop, daemon=True)
        self.thread.start()
        return True

    def stop(self):
        """Stops background thread and releases hardware bus."""
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=0.5)
        if not self.use_mock:
            self.i2c.close()

    def reset_heading(self, target_rad=0.0):
        """Resets the accumulated yaw heading to a specific orientation (default 0.0 rad)."""
        with self.lock:
            self.heading_rad = float(target_rad)

    def get_heading_rad(self):
        """Returns current heading in radians [-π, +π]. Positive = turned left (CCW)."""
        with self.lock:
            return self.heading_rad

    def get_heading_deg(self):
        """Returns current heading in degrees [-180, +180]."""
        with self.lock:
            return math.degrees(self.heading_rad)

    def get_yaw_rate_rad_s(self):
        """Returns current angular velocity around Z axis in radians/second."""
        with self.lock:
            return self.yaw_rate_rad_s

    def get_yaw_rate_deg_s(self):
        """Returns current angular velocity around Z axis in degrees/second."""
        with self.lock:
            return self.yaw_rate_dps

    def get_accel_mps2(self):
        """Returns (ax, ay, az) in m/s²."""
        with self.lock:
            return self.accel_mps2

    def get_linear_accel(self):
        """Alias for get_accel_mps2(). Returns (ax, ay, az) in m/s²."""
        return self.get_accel_mps2()

    def get_accel_g(self):
        """Returns (ax, ay, az) in units of g."""
        with self.lock:
            return self.accel_g

    def get_tilt_deg(self):
        """Returns (pitch_deg, roll_deg) representing mounting tilt angle."""
        with self.lock:
            return self.tilt_pitch_deg, self.tilt_roll_deg

    def is_healthy(self):
        """Returns True if the IMU is connected and updating."""
        with self.lock:
            if self.use_mock:
                return True
            # Unhealthy if no update received in the last 200 ms
            return (time.monotonic() - self.last_update_time) < 0.20

    def is_ready(self):
        """Alias for is_healthy(). Returns True if IMU is ready and streaming."""
        return self.is_healthy()

    def has_collision(self):
        """Returns and resets collision / sudden impact shock flag."""
        with self.lock:
            c = self.collision_detected
            self.collision_detected = False
            return c

    def _poll_loop(self):
        """Background thread running at sample_rate_hz."""
        while self.running:
            t_start = time.monotonic()

            if self.use_mock:
                time.sleep(self.interval_s)
                continue

            try:
                raw_bytes = self.i2c.read_block(REG_ACCEL_XOUT_H, 14)
                ax_r, ay_r, az_r, t_r, gx_r, gy_r, gz_r = struct.unpack('>7h', bytes(raw_bytes))

                now = time.monotonic()
                dt = max(0.001, min(0.10, now - self.last_update_time))
                self.last_update_time = now

                # Accelerometer
                ax_g = ax_r / ACCEL_SCALE_LSB_PER_G
                ay_g = ay_r / ACCEL_SCALE_LSB_PER_G
                az_g = az_r / ACCEL_SCALE_LSB_PER_G

                # Gyroscope Z (yaw rate)
                gz_dps = (gz_r / GYRO_SCALE_LSB_PER_DPS) - self.bias_gz_dps

                # Apply deadband filter to eliminate stationary creep
                if abs(gz_dps) < self.deadband_dps:
                    gz_dps = 0.0

                gz_rad_s = math.radians(gz_dps)

                # Check collision shock spike (|a_horizontal| > threshold)
                total_horiz_g = math.sqrt(ax_g**2 + ay_g**2)
                collision = total_horiz_g > self.collision_threshold_g

                with self.lock:
                    self.yaw_rate_dps = gz_dps
                    self.yaw_rate_rad_s = gz_rad_s
                    self.heading_rad += gz_rad_s * dt

                    # Wrap heading to [-π, +π]
                    self.heading_rad = math.atan2(math.sin(self.heading_rad), math.cos(self.heading_rad))

                    self.accel_g = (ax_g, ay_g, az_g)
                    self.accel_mps2 = (ax_g * GRAVITY_MPS2, ay_g * GRAVITY_MPS2, az_g * GRAVITY_MPS2)
                    self.temp_c = (t_r / 340.0) + 36.53

                    if collision:
                        self.collision_detected = True

            except Exception:
                # Intermittent I2C read error
                pass

            # Sleep remainder of the cycle
            elapsed = time.monotonic() - t_start
            sleep_time = self.interval_s - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
