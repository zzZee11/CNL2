"""
human_follower/turret_follower.py
---------------------------------
Stage 1 Stationary Turret Follower:
Rotates the robot on the spot (pivoting) at a steady velocity (default 0.20 m/s)
to keep a detected person centered in the camera.
Startup kicks have been removed for smooth, gentle turning.
Forward velocity is strictly 0.0 m/s.
"""

import math
import os
import sys
import time
import threading
from typing import Dict, Any, Optional, Tuple

# Enable importing vision package from project root
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from .config import FollowerConfig
from vision.config import VisionConfig
from vision.vision_service import VisionService
from vision.target_data import TargetObservation


class TurretFollower:
    """
    Stage 1 Controller: Pivots on the spot at a steady, gentle speed (0.20 m/s)
    with zero startup kicks and hysteresis deadband.
    """

    def __init__(self, config: Optional[FollowerConfig] = None) -> None:
        self.config = config or FollowerConfig()
        self.config.validate()

        # Serial state
        self._ser: Any = None
        self._serial_lock = threading.Lock()
        self._target_v_left: float = 0.0
        self._target_v_right: float = 0.0

        # State tracking
        self.state: str = "INITIALIZING"
        self._running: bool = False
        self._tx_thread: Optional[threading.Thread] = None
        self._loop_thread: Optional[threading.Thread] = None

        # Tracking controller state (Hysteresis)
        self._is_pivoting: bool = False

        # Vision Service initialization
        v_config = VisionConfig(
            camera_index=self.config.camera_index,
            camera_width=self.config.camera_width,
            camera_height=self.config.camera_height,
            model_backend="mock" if self.config.mock else self.config.model_backend,
            model_path=self.config.model_path,
            confidence_threshold=self.config.confidence_threshold,
            enable_preview=self.config.enable_preview and not self.config.headless,
            headless=self.config.headless,
        )
        self.vision = VisionService(v_config)

        # Telemetry
        self._last_observation = TargetObservation.invalid()
        self._telemetry: Dict[str, Any] = {}

    def start(self) -> bool:
        """Start the vision service and the 20 Hz serial transmission loop."""
        if self._running:
            return True

        self._init_serial()
        self.vision.start()

        self._running = True
        self._tx_thread = threading.Thread(target=self._serial_tx_worker, daemon=True)
        self._tx_thread.start()

        self._loop_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._loop_thread.start()

        self.state = "SEARCHING"
        return True

    def _init_serial(self) -> None:
        """Initialize serial connection to ESP32 or fallback to mock."""
        if self.config.mock:
            self._ser = None
            return

        try:
            import serial
            self._ser = serial.Serial(
                port=self.config.serial_port,
                baudrate=self.config.serial_baud,
                timeout=0.05,
                write_timeout=0.2,
            )
            time.sleep(1.0)  # Allow link stabilization
        except Exception as e:
            print(f"[TurretFollower] Could not open serial port {self.config.serial_port} ({e}).")
            print("[TurretFollower] Running in mock serial mode.")
            self._ser = None

    def _serial_tx_worker(self) -> None:
        """Continuously streams velocity packets at 20 Hz to keep the ESP32 watchdog refreshed."""
        interval = 1.0 / self.config.tx_rate_hz

        while self._running:
            t0 = time.monotonic()

            with self._serial_lock:
                cmd_l = self._target_v_left
                cmd_r = self._target_v_right

            # Send over UART: "<left_mps>,<right_mps>\n"
            if self._ser is not None:
                try:
                    packet = f"{cmd_l:.2f},{cmd_r:.2f}\n".encode("ascii")
                    self._ser.write(packet)
                    self._ser.flush()
                except Exception:
                    pass

            elapsed = time.monotonic() - t0
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _set_wheel_velocities(self, vl: float, vr: float) -> None:
        """Update target wheel speeds."""
        with self._serial_lock:
            self._target_v_left = vl
            self._target_v_right = vr

    def step(self, obs: Optional[TargetObservation] = None) -> Tuple[float, float, str]:
        """
        Execute one control step based on a target observation.
        Pivots at steady turn_speed (default 0.20 m/s) with zero kicks.
        Returns: (v_left, v_right, state)
        """
        if obs is None:
            obs = self.vision.get_latest_observation()

        self._last_observation = obs
        now = time.monotonic()

        # Check if target is valid and fresh
        is_fresh = (now - obs.timestamp) <= self.config.target_timeout_s
        if not obs.target_valid or not is_fresh or obs.confidence < self.config.confidence_threshold:
            self._set_wheel_velocities(0.0, 0.0)
            self._is_pivoting = False
            self.state = "SEARCHING"
            return 0.0, 0.0, self.state

        h_err = obs.horizontal_error  # -1.0 (left) to +1.0 (right)

        # ── 1. Hysteresis Deadband Evaluation ─────────────────────────────────
        # If already pivoting: must get inside deadband_enter (5%) to settle and stop
        # If currently idle: must cross outside deadband_exit (10%) to trigger pivoting
        active_deadband = self.config.deadband_enter if self._is_pivoting else self.config.deadband_exit

        if abs(h_err) <= active_deadband:
            self._set_wheel_velocities(0.0, 0.0)
            self._is_pivoting = False
            self.state = "LOCKED_CENTER"
            return 0.0, 0.0, self.state

        self._is_pivoting = True
        speed = self.config.turn_speed  # Steady pivot speed (0.20 m/s)

        # ── 2. Assign Wheel Directions (Steady 0.20 m/s Pivot) ────────────────
        # Target to RIGHT (h_err > 0): Left wheel forward (+0.20), Right wheel backward (-0.20)
        # Target to LEFT (h_err < 0): Left wheel backward (-0.20), Right wheel forward (+0.20)
        if h_err > 0:
            vl = +speed
            vr = -speed
            self.state = "PIVOT_RIGHT"
        else:
            vl = -speed
            vr = +speed
            self.state = "PIVOT_LEFT"

        self._set_wheel_velocities(vl, vr)
        return vl, vr, self.state

    def _control_loop(self) -> None:
        """20 Hz control loop updating the turret pivot velocities."""
        interval = 1.0 / self.config.tx_rate_hz

        while self._running:
            t0 = time.monotonic()
            vl, vr, state = self.step()

            # Record telemetry
            obs = self._last_observation
            v_telem = self.vision.get_telemetry()
            self._telemetry = {
                "state": state,
                "v_left": round(vl, 2),
                "v_right": round(vr, 2),
                "h_err": round(obs.horizontal_error, 3),
                "bearing_deg": round(obs.bearing_deg, 1),
                "confidence": round(obs.confidence, 2),
                "target_valid": obs.target_valid,
                "fps": v_telem.get("fps", 0.0),
                "inf_ms": v_telem.get("inference_ms", 0.0),
            }

            elapsed = time.monotonic() - t0
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def get_telemetry(self) -> Dict[str, Any]:
        """Return latest follower telemetry."""
        return dict(self._telemetry)

    def stop(self) -> None:
        """Safely halt robot motors and release hardware resources."""
        self._running = False
        self._set_wheel_velocities(0.0, 0.0)

        # Send multiple zero packets to ensure ESP32 stops
        if self._ser is not None:
            try:
                for _ in range(3):
                    self._ser.write(b"0.00,0.00\n")
                    self._ser.flush()
                    time.sleep(0.02)
                self._ser.close()
            except Exception:
                pass
            self._ser = None

        self.vision.stop()
        self.state = "STOPPED"
