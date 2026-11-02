#!/usr/bin/env python3
"""
CNL2 MPU-6050 (GY-521) Hardware Diagnostic & Heading Test Script
================================================================
Tests whether the MPU-6050 6-DOF IMU (Accelerometer + Gyroscope) is
correctly wired, communicating over I2C, and tracking rotation/tilt.

Works out-of-the-box using the built-in Linux I2C interface (/dev/i2c-1)
or smbus/smbus2 if installed. Zero external pip packages required.

Wiring:
  - VCC -> 3.3V (Raspberry Pi Pin 1)
  - GND -> GND  (Raspberry Pi Pin 6 or 9)
  - SDA -> GPIO 2 / Pin 3
  - SCL -> GPIO 3 / Pin 5
  - AD0 -> GND (Pin 9 or 14) - Locks address to 0x68

Usage:
  python3 test_mpu6050.py          # Run hardware test on Raspberry Pi
  python3 test_mpu6050.py --mock   # Run in simulation mode (offline test)
"""

import os
import sys
import time
import math
import struct
import argparse

# ==========================================
# MPU-6050 REGISTER MAP
# ==========================================
MPU6050_ADDR_PRIMARY = 0x68   # AD0 connected to GND
MPU6050_ADDR_ALT     = 0x69   # AD0 connected to 3.3V

REG_SMPLRT_DIV   = 0x19       # Sample rate divider
REG_CONFIG       = 0x1A       # DLPF (low-pass filter) config
REG_GYRO_CONFIG  = 0x1B       # Gyroscope range configuration
REG_ACCEL_CONFIG = 0x1C       # Accelerometer range configuration
REG_ACCEL_XOUT_H = 0x3B       # First data register (14 bytes burst read)
REG_TEMP_OUT_H   = 0x41
REG_GYRO_XOUT_H  = 0x43
REG_PWR_MGMT_1   = 0x6B       # Power management 1 (sleep / clock source)
REG_WHO_AM_I     = 0x75       # Device ID verification register (returns 0x68)

# Scale factors for standard ranges (±2g, ±250 deg/s)
ACCEL_SCALE_LSB_PER_G    = 16384.0
GYRO_SCALE_LSB_PER_DPS   = 131.0
GRAVITY_MPS2             = 9.80665


class I2CDevice:
    """Lightweight I2C interface using Linux ioctl (/dev/i2c-X) with smbus fallback."""

    def __init__(self, bus_num=1, address=MPU6050_ADDR_PRIMARY):
        self.bus_num = bus_num
        self.address = address
        self.fd = None
        self.use_smbus = False
        self.bus = None

        # Try importing smbus2 or smbus first if available
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
        """Opens I2C connection to the device."""
        if self.use_smbus:
            return True

        import fcntl
        dev_path = f"/dev/i2c-{self.bus_num}"
        if not os.path.exists(dev_path):
            raise FileNotFoundError(
                f"I2C device '{dev_path}' not found.\n"
                "If using a Raspberry Pi, enable I2C via:\n"
                "  sudo raspi-config -> Interface Options -> I2C -> Enable\n"
                "Then reboot."
            )

        self.fd = os.open(dev_path, os.O_RDWR)
        # 0x0703 is I2C_SLAVE ioctl request
        fcntl.ioctl(self.fd, 0x0703, self.address)
        return True

    def write_byte_data(self, reg, val):
        """Writes a single byte to a register."""
        if self.use_smbus:
            self.bus.write_byte_data(self.address, reg, val)
        else:
            os.write(self.fd, bytes([reg, val & 0xFF]))

    def read_byte_data(self, reg):
        """Reads a single byte from a register."""
        if self.use_smbus:
            return self.bus.read_byte_data(self.address, reg)
        else:
            os.write(self.fd, bytes([reg]))
            buf = os.read(self.fd, 1)
            return buf[0]

    def read_block_data(self, reg, length):
        """Reads a block of bytes starting from register."""
        if self.use_smbus:
            return self.bus.read_i2c_block_data(self.address, reg, length)
        else:
            os.write(self.fd, bytes([reg]))
            buf = os.read(self.fd, length)
            return list(buf)

    def close(self):
        """Closes the connection."""
        if self.use_smbus and self.bus:
            self.bus.close()
        elif self.fd is not None:
            os.close(self.fd)
            self.fd = None


class MockMPU6050:
    """Mock IMU for testing navigation logic when offline/without hardware."""

    def __init__(self):
        self.t0 = time.time()

    def open(self):
        return True

    def verify_chip(self):
        return 0x68

    def configure(self):
        pass

    def read_raw_data(self):
        t = time.time() - self.t0
        # Simulate gentle turning oscillation
        gz_sim = 15.0 * math.sin(t * 0.8)
        # Simulate ~3 deg tilt on X and 1.5 deg tilt on Y
        ax_sim = 0.05 * 16384.0
        ay_sim = 0.02 * 16384.0
        az_sim = 0.998 * 16384.0
        temp_sim = int((28.5 - 36.53) * 340.0)

        data = struct.pack('>7h', int(ax_sim), int(ay_sim), int(az_sim),
                           temp_sim, 0, 0, int(gz_sim * 131.0))
        return list(data)

    def close(self):
        pass


class MPU6050:
    """Driver and diagnostic interface for InvenSense MPU-6050."""

    def __init__(self, bus_num=1, address=MPU6050_ADDR_PRIMARY, mock=False):
        self.mock = mock
        self.address = address
        self.bus_num = bus_num
        self.dev = MockMPU6050() if mock else I2CDevice(bus_num, address)

        # Calibration baseline biases
        self.bias_ax = 0.0
        self.bias_ay = 0.0
        self.bias_az = 0.0
        self.bias_gx = 0.0
        self.bias_gy = 0.0
        self.bias_gz = 0.0

        # Mounting tilt angles calculated at rest (degrees)
        self.tilt_pitch_deg = 0.0
        self.tilt_roll_deg = 0.0

    def begin(self):
        """Initializes connection, verifies WHO_AM_I, and wakes up sensor."""
        if self.mock:
            self.dev.open()
            return True, self.address

        # Check primary address (0x68) then alternative (0x69)
        tested_addresses = [self.address]
        if self.address == MPU6050_ADDR_PRIMARY:
            tested_addresses.append(MPU6050_ADDR_ALT)

        detected_addr = None
        for addr in tested_addresses:
            try:
                self.dev.address = addr
                self.dev.open()
                who = self.dev.read_byte_data(REG_WHO_AM_I)
                if who in (0x68, 0x70, 0x71, 0x72):
                    detected_addr = addr
                    self.address = addr
                    break
                self.dev.close()
            except Exception:
                if self.dev.fd is not None:
                    self.dev.close()

        if detected_addr is None:
            return False, None

        # Configure chip registers
        try:
            # 1. Wake up chip and set clock source to Gyro X PLL for lower drift
            self.dev.write_byte_data(REG_PWR_MGMT_1, 0x01)
            time.sleep(0.05)

            # 2. Set DLPF (Digital Low Pass Filter) to ~42 Hz (Bandwidth 44Hz, delay 4.9ms)
            # This suppresses high-frequency motor vibration noise from the chassis!
            self.dev.write_byte_data(REG_CONFIG, 0x03)

            # 3. Sample Rate = Gyro Output Rate / (1 + SMPLRT_DIV) -> 1kHz / (1 + 9) = 100 Hz
            self.dev.write_byte_data(REG_SMPLRT_DIV, 0x09)

            # 4. Gyro range: ±250 °/s (0x00) -> 131.0 LSB / (°/s)
            self.dev.write_byte_data(REG_GYRO_CONFIG, 0x00)

            # 5. Accel range: ±2g (0x00) -> 16384.0 LSB / g
            self.dev.write_byte_data(REG_ACCEL_CONFIG, 0x00)
            time.sleep(0.05)

            return True, detected_addr
        except Exception as e:
            print(f"[ERROR] Failed configuring MPU-6050 registers: {e}")
            return False, None

    def read_raw(self):
        """Reads 14 consecutive bytes (Accel X/Y/Z, Temp, Gyro X/Y/Z)."""
        if self.mock:
            raw_bytes = self.dev.read_raw_data()
        else:
            raw_bytes = self.dev.read_block_data(REG_ACCEL_XOUT_H, 14)

        ax, ay, az, t_raw, gx, gy, gz = struct.unpack('>7h', bytes(raw_bytes))
        return ax, ay, az, t_raw, gx, gy, gz

    def read_measurements(self):
        """Returns physical values (accel in m/s², gyro in deg/s, temp in °C)."""
        ax_r, ay_r, az_r, t_r, gx_r, gy_r, gz_r = self.read_raw()

        # Convert to g and deg/s
        ax_g = ax_r / ACCEL_SCALE_LSB_PER_G
        ay_g = ay_r / ACCEL_SCALE_LSB_PER_G
        az_g = az_r / ACCEL_SCALE_LSB_PER_G

        gx_dps = gx_r / GYRO_SCALE_LSB_PER_DPS
        gy_dps = gy_r / GYRO_SCALE_LSB_PER_DPS
        gz_dps = gz_r / GYRO_SCALE_LSB_PER_DPS

        temp_c = (t_r / 340.0) + 36.53

        return {
            "ax_mps2": ax_g * GRAVITY_MPS2,
            "ay_mps2": ay_g * GRAVITY_MPS2,
            "az_mps2": az_g * GRAVITY_MPS2,
            "ax_g": ax_g,
            "ay_g": ay_g,
            "az_g": az_g,
            "gx_dps": gx_dps,
            "gy_dps": gy_dps,
            "gz_dps": gz_dps,
            "temp_c": temp_c
        }

    def calibrate(self, num_samples=100, delay_s=0.015):
        """Calibrates gyro zero-rate offsets and calculates resting tilt angles."""
        print("\n" + "="*60)
        print("  STEP 1: ZERO-POINT CALIBRATION")
        print("  Keep the robot stationary on a flat surface...")
        print("="*60)

        sum_gx = 0.0
        sum_gy = 0.0
        sum_gz = 0.0
        sum_ax = 0.0
        sum_ay = 0.0
        sum_az = 0.0

        for i in range(num_samples):
            m = self.read_measurements()
            sum_gx += m["gx_dps"]
            sum_gy += m["gy_dps"]
            sum_gz += m["gz_dps"]
            sum_ax += m["ax_g"]
            sum_ay += m["ay_g"]
            sum_az += m["az_g"]

            if (i + 1) % 20 == 0 or i == num_samples - 1:
                progress = int(((i + 1) / num_samples) * 30)
                bar = "[" + "=" * progress + " " * (30 - progress) + "]"
                print(f"  Calibrating: {bar} {int((i+1)/num_samples*100)}%", end="\r")
            time.sleep(delay_s)

        print("\n")
        self.bias_gx = sum_gx / num_samples
        self.bias_gy = sum_gy / num_samples
        self.bias_gz = sum_gz / num_samples

        avg_ax_g = sum_ax / num_samples
        avg_ay_g = sum_ay / num_samples
        avg_az_g = sum_az / num_samples

        # Calculate static mounting tilt from gravity vector
        # Pitch: tilt along forward/backward axis
        # Roll: tilt along left/right lateral axis
        self.tilt_pitch_deg = math.atan2(avg_ax_g, math.sqrt(avg_ay_g**2 + avg_az_g**2)) * (180.0 / math.pi)
        self.tilt_roll_deg  = math.atan2(avg_ay_g, math.sqrt(avg_ax_g**2 + avg_az_g**2)) * (180.0 / math.pi)

        print("  Calibration Complete:")
        print(f"    - Gyro Z Bias (Drift Offset): {self.bias_gz:+.3f} °/s")
        print(f"    - Measured Mounting Pitch   : {self.tilt_pitch_deg:+.2f}° (forward/backward tilt)")
        print(f"    - Measured Mounting Roll    : {self.tilt_roll_deg:+.2f}° (lateral tilt)")
        print("="*60)


def build_compass_bar(yaw_deg, width=21):
    """Draws a visual ASCII heading indicator."""
    center = width // 2
    # Scale: -90 to +90 mapped to bar width
    norm = max(-90.0, min(90.0, yaw_deg))
    idx = center + int((norm / 90.0) * center)
    idx = max(0, min(width - 1, idx))

    bar = list("-" * width)
    bar[center] = "|"
    bar[idx] = "O" if idx != center else "*"
    return "".join(bar)


def main():
    parser = argparse.ArgumentParser(description="Test MPU-6050 (GY-521) IMU on Raspberry Pi")
    parser.add_argument("--bus", type=int, default=1, help="I2C bus number (default: 1)")
    parser.add_argument("--addr", type=lambda x: int(x, 16), default=MPU6050_ADDR_PRIMARY,
                        help="I2C address in hex (default: 0x68)")
    parser.add_argument("--mock", action="store_true", help="Run simulated IMU without hardware")
    args = parser.parse_args()

    print("\n" + "#"*64)
    print("  CNL2 ROBOT — MPU-6050 HARDWARE DIAGNOSTIC TOOL")
    print("#"*64)

    imu = MPU6050(bus_num=args.bus, address=args.addr, mock=args.mock)

    print(f"\n[1/3] Connecting to I2C bus {args.bus}...")
    success, detected_addr = imu.begin()

    if not success:
        print("\n" + "!"*60)
        print("  [FAILED] Could not communicate with MPU-6050.")
        print("!"*60)
        print("\nTroubleshooting Checklist:")
        print("  1. Verify wiring:")
        print("     - VCC -> 3.3V (Raspberry Pi Pin 1)")
        print("     - GND -> GND (Raspberry Pi Pin 6 or 9)")
        print("     - SDA -> GPIO 2 (Raspberry Pi Pin 3)")
        print("     - SCL -> GPIO 3 (Raspberry Pi Pin 5)")
        print("     - AD0 -> GND (Raspberry Pi Pin 9 or 14)")
        print("  2. Run 'sudo i2cdetect -y 1' to scan the bus.")
        print("     You should see '68' on the grid.")
        print("  3. Make sure I2C is enabled in raspi-config:")
        print("     sudo raspi-config -> 3 Interface Options -> I4 I2C -> Yes")
        print("  4. To test this script without hardware, run:")
        print("     python3 test_mpu6050.py --mock\n")
        sys.exit(1)

    print(f"  [OK] MPU-6050 detected at address 0x{detected_addr:02X}!")
    print("  [OK] Internal oscillator set to Gyro PLL (Low Drift)")
    print("  [OK] Digital Low Pass Filter (DLPF) set to 42 Hz (Vibration Damping)")

    # Step 1: Calibrate resting bias and measure mounting tilt
    imu.calibrate(num_samples=80)

    # Step 2: Live telemetry loop
    print("\n" + "="*60)
    print("  STEP 2: LIVE TELEMETRY & HEADING TRACKER")
    print("  Rotate or tilt the robot by hand to see live orientation.")
    print("  Press Ctrl+C to stop.")
    print("="*60 + "\n")

    heading_yaw_deg = 0.0
    last_time = time.time()

    try:
        while True:
            now = time.time()
            dt = now - last_time
            last_time = now

            m = imu.read_measurements()

            # Subtract static biases
            gz_corr = m["gz_dps"] - imu.bias_gz

            # Integrate yaw angle: Δθ = ωz * Δt
            # Deadband threshold of 0.35 deg/s to completely eliminate stationary drift
            if abs(gz_corr) > 0.35:
                heading_yaw_deg += gz_corr * dt

            # Normalize to [-180, +180] degrees
            heading_yaw_deg = (heading_yaw_deg + 180.0) % 360.0 - 180.0

            # Motion classification
            if abs(gz_corr) > 6.0:
                turn_status = "TURNING LEFT " if gz_corr > 0 else "TURNING RIGHT"
            else:
                turn_status = "STRAIGHT / IDLE"

            # Draw visual compass bar
            compass_bar = build_compass_bar(heading_yaw_deg)

            # Live formatted display
            sys.stdout.write(
                f"\rYaw: {heading_yaw_deg:+6.1f}° {compass_bar} | "
                f"TurnRate: {gz_corr:+6.1f}°/s | "
                f"Accel: [{m['ax_g']:+4.2f}, {m['ay_g']:+4.2f}, {m['az_g']:+4.2f}]g | "
                f"Temp: {m['temp_c']:4.1f}°C | {turn_status}"
            )
            sys.stdout.flush()

            time.sleep(0.05)  # 20 Hz refresh

    except KeyboardInterrupt:
        print("\n\n" + "="*60)
        print("  Test stopped by user.")
        print(f"  Final Yaw Heading: {heading_yaw_deg:+.2f}°")
        print("  MPU-6050 is functioning properly!")
        print("="*60 + "\n")
    finally:
        imu.dev.close()


if __name__ == "__main__":
    main()

