"""
human_follower/stage2_follower.py
---------------------------------
Stage 2 Human Follower: Combined Distance Tracking & Heading Steering.

Objective:
In open space, follows a moving human at a calibrated target distance (default ~1.2m).
1. Distance Control: Measures distance via Center VL53L0X ToF laser (close-range, high precision)
   and vision bounding-box height proxy (long-range). Drives forward when person walks away,
   smoothly decelerates and halts when person is within the sweet spot (1.0m - 1.4m).
2. Heading Control: Uses Stage 1 PD anti-overshoot steering to keep person centered.
3. Safety Supervisor: Emergency stop if distance drops below safety buffer (< 0.45m).
"""

import math
import os
import sys
import time
import threading
from dataclasses import dataclass
from typing import Dict, Any, Optional, Tuple

# Enable importing packages from project root
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from vision.config import VisionConfig
from vision.vision_service import VisionService
from vision.target_data import TargetObservation
from autonomous_navigation.sensors import DistanceSensorManager


@dataclass
class Stage2Config:
    # ── UART & Serial Communication ───────────────────────────────────────────
    serial_port: str = "/dev/serial0"
    serial_baud: int = 115200
    tx_rate_hz: float = 20.0           # 20 Hz transmission to satisfy ESP32 500ms watchdog
    watchdog_timeout_s: float = 0.45   # Serial command timeout
    
    # ── Stage 2 Distance Regulation Controls ──────────────────────────────────
    target_distance_m: float = 1.20    # Desired distance to maintain from the human (meters)
    distance_deadband_m: float = 0.20  # +/- 20 cm sweet spot: between (target - 0.2) and (target + 0.2), v_lin = 0
    kp_dist: float = 0.55              # Proportional gain for forward velocity: v = Kp * distance_error
    min_forward_speed: float = 0.15    # Minimum forward speed (m/s) to overcome DC motor gearbox stiction
    max_forward_speed: float = 0.45    # Maximum cruising forward speed (m/s)
    emergency_stop_dist_m: float = 0.45 # Distance threshold below which robot halts immediately
    
    # ── Stage 1 Heading Controls (PD Anti-Overshoot) ───────────────────────────
    kp_turn: float = 0.35              # Proportional turn gain
    kd_turn: float = 0.12              # Derivative damping gain (virtual brake on center approach)
    deadband_turn_exit: float = 0.10   # Trigger threshold to start steering (10% of frame center)
    deadband_turn_enter: float = 0.05  # Settle threshold to stop steering (5% of frame center)
    min_turn_speed: float = 0.12       # Minimum gentle turn speed (m/s)
    max_turn_speed: float = 0.25       # Maximum turn speed clamp (m/s)
    
    # ── Forward Startup Kick ──────────────────────────────────────────────────
    # Kick is applied ONLY when starting forward drive from a complete stop,
    # overcoming static friction of the entire vehicle. (Pivots do NOT trigger kick).
    enable_forward_kick: bool = True
    forward_kick_speed: float = 0.50
    forward_kick_duration_s: float = 0.12
    
    # ── Safety & Timeouts ─────────────────────────────────────────────────────
    target_timeout_s: float = 1.0      # If target is lost for > 1.0s, decelerate to 0
    confidence_threshold: float = 0.40 # Minimum detection confidence
    
    # ── Hardware & Camera Settings ────────────────────────────────────────────
    wheel_base_m: float = 0.13
    wheel_radius_m: float = 0.02
    camera_index: int = 0
    camera_width: int = 640
    camera_height: int = 480
    model_backend: str = "tflite"
    model_path: str = "vision/models/detect.tflite"
    enable_preview: bool = True
    headless: bool = False
    mock: bool = False

    def validate(self) -> None:
        if self.target_distance_m <= self.emergency_stop_dist_m:
            raise ValueError("target_distance_m must be greater than emergency_stop_dist_m.")
        if self.min_forward_speed > self.max_forward_speed:
            raise ValueError("min_forward_speed cannot exceed max_forward_speed.")


class Stage2DistanceFollower:
    """
    Stage 2 Controller: Follows human with distance regulation and heading alignment.
    """

    def __init__(self, config: Optional[Stage2Config] = None) -> None:
        self.config = config or Stage2Config()
        self.config.validate()

        # Serial state
        self._ser: Any = None
        self._serial_lock = threading.Lock()
        self._target_v_left: float = 0.0
        self._target_v_right: float = 0.0
        self._forward_kick_until: float = 0.0

        # State tracking
        self.state: str = "INITIALIZING"
        self._running: bool = False
        self._tx_thread: Optional[threading.Thread] = None
        self._loop_thread: Optional[threading.Thread] = None

        # Heading PD & Hysteresis state
        self._is_steering: bool = False
        self._last_h_err: float = 0.0
        self._last_h_err_time: float = 0.0

        # Subsystem 1: Vision Service
        v_config = VisionConfig(
            camera_index=self.config.camera_index,
            camera_width=self.config.camera_width,
            camera_height=self.config.camera_height,
            model_backend="mock" if self.config.mock else self.config.model_backend,
            model_path=self.config.model_path,
            confidence_threshold=self.config.confidence_threshold,
            distance_calibration_enabled=True,
            enable_preview=self.config.enable_preview and not self.config.headless,
            headless=self.config.headless,
        )
        self.vision = VisionService(v_config)

        # Subsystem 2: Distance Sensor Manager (3x VL53L0X ToF lasers)
        self.sensors = DistanceSensorManager(use_mock=self.config.mock)

        # Telemetry
        self._last_observation = TargetObservation.invalid()
        self._telemetry: Dict[str, Any] = {}

    def start(self) -> bool:
        """Start vision, sensors, serial loop, and control loop."""
        if self._running:
            return True

        self._init_serial()
        self.sensors.initialize()
        self.sensors.start()
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
            time.sleep(1.0)
        except Exception as e:
            print(f"[Stage2Follower] Could not open serial port {self.config.serial_port} ({e}).")
            print("[Stage2Follower] Running in mock serial mode.")
            self._ser = None

    def _serial_tx_worker(self) -> None:
        """Continuously streams velocity packets at 20 Hz to refresh ESP32 watchdog."""
        interval = 1.0 / self.config.tx_rate_hz

        while self._running:
            t0 = time.monotonic()

            with self._serial_lock:
                vl = self._target_v_left
                vr = self._target_v_right
                is_kicking = (t0 < self._forward_kick_until)

            # Apply forward breakaway kick if active
            if is_kicking:
                kick = self.config.forward_kick_speed
                cmd_l = kick if vl > 0.05 else vl
                cmd_r = kick if vr > 0.05 else vr
            else:
                cmd_l = vl
                cmd_r = vr

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

    def _set_wheel_velocities(self, vl: float, vr: float, is_forward_accel: bool = False) -> None:
        """Update commanded wheel velocities with forward kick trigger."""
        now = time.monotonic()
        with self._serial_lock:
            was_stopped = (abs(self._target_v_left) < 0.02 and abs(self._target_v_right) < 0.02)
            moving_forward = (vl > 0.05 and vr > 0.05)

            if was_stopped and moving_forward and is_forward_accel and self.config.enable_forward_kick:
                self._forward_kick_until = now + self.config.forward_kick_duration_s

            self._target_v_left = vl
            self._target_v_right = vr

    def _estimate_human_distance(self, obs: TargetObservation, mid_dist: float) -> Tuple[float, str]:
        """
        Sensor Fusion:
        1. If Center ToF sensor has a valid reading (< 1.85m) and person is roughly centered -> Use ToF laser.
        2. Else -> Use calibrated vision height proxy.
        """
        is_centered = abs(obs.horizontal_error) < 0.25
        if is_centered and 0.05 < mid_dist < 1.85:
            return mid_dist, "TOF_LASER"

        if obs.distance is not None and 0.3 < obs.distance < 8.0:
            return obs.distance, "VISION_PROXY"

        # If person is centered but ToF reads > 1.85m (out of range)
        if mid_dist >= 1.85:
            return 2.2, "TOF_OUT_OF_RANGE"

        return 1.20, "DEFAULT"

    def step(self, obs: Optional[TargetObservation] = None, mock_mid_tof: Optional[float] = None) -> Tuple[float, float, str]:
        """
        Execute one control cycle:
        1. Evaluate distance error -> linear velocity (v_lin)
        2. Evaluate heading error -> angular turn velocity (v_turn)
        3. Differential kinematics fusion -> (v_left, v_right)
        """
        if obs is None:
            obs = self.vision.get_latest_observation()

        self._last_observation = obs
        now = time.monotonic()

        # Check target freshness and validity
        is_fresh = (now - obs.timestamp) <= self.config.target_timeout_s
        if not obs.target_valid or not is_fresh or obs.confidence < self.config.confidence_threshold:
            self._set_wheel_velocities(0.0, 0.0)
            self._is_steering = False
            self._last_h_err = 0.0
            self._last_h_err_time = now
            self.state = "SEARCHING"
            return 0.0, 0.0, self.state

        # Get ToF distance readings [left, middle, right]
        if mock_mid_tof is not None:
            mid_dist = mock_mid_tof
        else:
            _, mid_dist, _ = self.sensors.get_distances()

        # Sensor fusion for human distance
        dist_m, dist_source = self._estimate_human_distance(obs, mid_dist)

        # ── 1. Safety Check: Emergency Stop ───────────────────────────────────
        if dist_m < self.config.emergency_stop_dist_m:
            self._set_wheel_velocities(0.0, 0.0)
            self.state = "SAFETY_STOP"
            return 0.0, 0.0, self.state

        # ── 2. Distance Regulation (Linear Velocity) ──────────────────────────
        sweet_spot_min = self.config.target_distance_m - self.config.distance_deadband_m  # e.g. 1.0m
        sweet_spot_max = self.config.target_distance_m + self.config.distance_deadband_m  # e.g. 1.4m

        if dist_m > sweet_spot_max:
            # Person is walking away: drive forward
            dist_error = dist_m - self.config.target_distance_m
            raw_v_lin = self.config.kp_dist * dist_error
            v_linear = max(self.config.min_forward_speed, min(self.config.max_forward_speed, raw_v_lin))
            is_forward_drive = True
        else:
            # Inside sweet spot (1.0m - 1.4m) or close: hold forward position
            v_linear = 0.0
            is_forward_drive = False

        # ── 3. Heading Regulation (Angular Velocity using PD Anti-Overshoot) ──
        h_err = obs.horizontal_error
        active_turn_deadband = self.config.deadband_turn_enter if self._is_steering else self.config.deadband_turn_exit

        if abs(h_err) <= active_turn_deadband:
            v_turn = 0.0
            self._is_steering = False
            self._last_h_err = h_err
            self._last_h_err_time = now
        else:
            self._is_steering = True
            dt = max(0.01, now - self._last_h_err_time) if self._last_h_err_time > 0 else 0.05
            d_err = (h_err - self._last_h_err) / dt
            self._last_h_err = h_err
            self._last_h_err_time = now

            # Smooth power easing
            err_mag = abs(h_err)
            span = max(0.01, 1.0 - self.config.deadband_turn_enter)
            norm_mag = min(1.0, max(0.0, (err_mag - self.config.deadband_turn_enter) / span))
            base_turn_speed = self.config.min_turn_speed + (
                self.config.max_turn_speed - self.config.min_turn_speed
            ) * (norm_mag ** 1.3)

            # Derivative damping counteraction
            is_closing_in = (h_err > 0 and d_err < -0.2) or (h_err < 0 and d_err > 0.2)
            if is_closing_in:
                damping_factor = max(0.45, 1.0 - 0.20 * abs(d_err))
                turn_speed = base_turn_speed * damping_factor
            else:
                turn_speed = base_turn_speed

            turn_speed = max(self.config.min_turn_speed, min(self.config.max_turn_speed, turn_speed))
            # Direction: positive h_err (right) requires left wheel faster / right wheel slower
            v_turn = turn_speed if h_err > 0 else -turn_speed

        # ── 4. Turn Factor Reduction on Linear Speed ──────────────────────────
        # If robot needs to turn sharply (|h_err| > 0.25), scale down forward speed so it doesn't drift away
        if abs(h_err) > 0.20:
            scale_factor = max(0.30, math.cos(math.radians(obs.bearing_deg)))
            v_linear *= scale_factor

        # ── 5. Kinematics Fusion ──────────────────────────────────────────────
        # Left wheel = v_linear + v_turn
        # Right wheel = v_linear - v_turn
        # (If v_linear=0, left=+v_turn, right=-v_turn -> pivots in place like Stage 1)
        vl = v_linear + v_turn
        vr = v_linear - v_turn

        # Clamp wheel speeds to safe hardware limits
        max_limit = self.config.max_forward_speed + 0.05
        vl = max(-max_limit, min(max_limit, vl))
        vr = max(-max_limit, min(max_limit, vr))

        # Determine human-readable state
        if v_linear > 0.05:
            if abs(v_turn) > 0.05:
                self.state = "FOLLOW_CURVE_RIGHT" if v_turn > 0 else "FOLLOW_CURVE_LEFT"
            else:
                self.state = "FOLLOW_STRAIGHT"
        else:
            if abs(v_turn) > 0.05:
                self.state = "PIVOT_RIGHT" if v_turn > 0 else "PIVOT_LEFT"
            else:
                self.state = "HOLD_SWEET_SPOT"

        self._set_wheel_velocities(vl, vr, is_forward_accel=is_forward_drive)
        return vl, vr, self.state

    def _control_loop(self) -> None:
        """20 Hz control loop executing distance and heading tracking."""
        interval = 1.0 / self.config.tx_rate_hz

        while self._running:
            t0 = time.monotonic()
            vl, vr, state = self.step()

            # Record telemetry
            obs = self._last_observation
            v_telem = self.vision.get_telemetry()
            _, mid_d, _ = self.sensors.get_distances()
            dist_m, source = self._estimate_human_distance(obs, mid_dist=mid_d)

            self._telemetry = {
                "state": state,
                "dist_m": round(dist_m, 2),
                "dist_source": source,
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

        self.sensors.running = False
        self.vision.stop()
        self.state = "STOPPED"
