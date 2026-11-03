"""
autonomous_navigation/sensors.py
---------------------------------
Driver for 3x VL53L0X Time-of-Flight distance sensors connected to the Raspberry Pi.
Uses sequential XSHUT booting to readdress sensors to 0x30, 0x31, 0x32.
Runs background polling at 20 Hz with thread-safe distance retrieval.
Includes automatic fallback/mock mode for desktop testing.
"""

import time
import threading

try:
    from .config import (
        XSHUT_LEFT, XSHUT_MIDDLE, XSHUT_RIGHT,
        ADDR_LEFT, ADDR_MIDDLE, ADDR_RIGHT,
        TOF_SAMPLE_INTERVAL_S, TOF_MAX_RANGE_M
    )
except ImportError:
    from config import (
        XSHUT_LEFT, XSHUT_MIDDLE, XSHUT_RIGHT,
        ADDR_LEFT, ADDR_MIDDLE, ADDR_RIGHT,
        TOF_SAMPLE_INTERVAL_S, TOF_MAX_RANGE_M
    )

class DistanceSensorManager:
    def __init__(self, use_mock=False):
        self.use_mock = use_mock
        self.running = False
        self.thread = None
        self.lock = threading.Lock()

        # Distances in meters [Left, Middle, Right]. 9.99 means no obstacle in sight.
        self.distances = [9.99, 9.99, 9.99]
        self.last_update = 0.0

        self.i2c = None
        self.xshuts = []
        self.sensors = []

    def initialize(self):
        """Initializes the 3 VL53L0X sensors via sequential XSHUT readdressing."""
        if self.use_mock:
            print("[Sensors] Initializing in MOCK / SIMULATION mode.")
            return True

        try:
            import board
            import busio
            from gpiozero import DigitalOutputDevice
            import adafruit_vl53l0x
        except ImportError as e:
            print(f"[Sensors] Hardware libraries not available ({e}).")
            print("[Sensors] Falling back to MOCK mode for testing.")
            self.use_mock = True
            return True

        try:
            print("[Sensors] Initializing I2C bus 1 (board.SCL, board.SDA)...")
            self.i2c = busio.I2C(board.SCL, board.SDA)

            # Create XSHUT controls (LOW = hold in shutdown)
            left_xshut = DigitalOutputDevice(XSHUT_LEFT, initial_value=False)
            middle_xshut = DigitalOutputDevice(XSHUT_MIDDLE, initial_value=False)
            right_xshut = DigitalOutputDevice(XSHUT_RIGHT, initial_value=False)
            self.xshuts = [left_xshut, middle_xshut, right_xshut]

            # Ensure all sensors are off
            for xs in self.xshuts:
                xs.off()
            time.sleep(0.1)

            # Initialize each sensor sequentially
            configs = [
                ("Left", left_xshut, ADDR_LEFT),
                ("Middle", middle_xshut, ADDR_MIDDLE),
                ("Right", right_xshut, ADDR_RIGHT)
            ]

            for name, xshut, address in configs:
                xshut.on()
                time.sleep(0.08)
                sensor = adafruit_vl53l0x.VL53L0X(self.i2c)
                sensor.set_address(address)
                # Adjust measurement timing budget if needed (default is 33ms)
                sensor.measurement_timing_budget = 33000
                self.sensors.append(sensor)
                print(f"[Sensors] {name} ToF initialized at 0x{address:02X}")

            print("[Sensors] All 3 VL53L0X sensors initialized successfully.")
            return True

        except Exception as e:
            print(f"[Sensors] Error initializing hardware sensors: {e}")
            print("[Sensors] Switching to MOCK mode.")
            self.use_mock = True
            return False

    def start(self):
        """Starts background polling thread."""
        self.running = True
        self.thread = threading.Thread(target=self._poll_loop, daemon=True)
        self.thread.start()

    def get_distances(self):
        """Returns the latest distances in meters [left, middle, right]."""
        with self.lock:
            return list(self.distances)

    def set_mock_distances(self, left_m, middle_m, right_m):
        """Allows testing obstacle avoidance manually in mock mode."""
        if self.use_mock:
            with self.lock:
                self.distances = [float(left_m), float(middle_m), float(right_m)]
                self.last_update = time.time()

    def _poll_loop(self):
        while self.running:
            if self.use_mock:
                time.sleep(TOF_SAMPLE_INTERVAL_S)
                continue

            readings = []
            for sensor in self.sensors:
                try:
                    # sensor.range is in millimeters
                    dist_mm = sensor.range
                    dist_m = float(dist_mm) / 1000.0
                    # Filter out-of-range or sensor noise
                    if dist_m > TOF_MAX_RANGE_M or dist_m <= 0.02:
                        dist_m = 9.99
                except (OSError, RuntimeError):
                    dist_m = 9.99
                readings.append(dist_m)

            with self.lock:
                self.distances = readings
                self.last_update = time.time()

            time.sleep(TOF_SAMPLE_INTERVAL_S)

    def close(self):
        """Powers down sensors and releases GPIO."""
        self.running = False
        for xs in self.xshuts:
            try:
                xs.off()
                xs.close()
            except Exception:
                pass
        self.sensors.clear()
        self.xshuts.clear()
        print("[Sensors] Shut down complete.")
