"""
human_follower/stage2_follower.py
---------------------------------
Stage 2 Human Follower: Combined Distance Tracking & Heading Steering.

Objective:
In open space, follows a moving human at a calibrated target distance (default ~1.2m).
1. Hard Hardware Safety Shield:
   Reads all 3 ToF laser distance sensors (Left, Middle, Right). If any obstacle or human
   is closer than the emergency threshold (< 0.45m), immediately commands an instant hardware halt.
2. Robust Distance Fusion:
   Prioritizes high-precision direct ToF laser measurements. If the target is beyond ToF range
   (> 1.85m), uses a calibrated vision bounding-box height proxy with physical sanity bounds
   to prevent false far-distance estimates.
3. Smooth Forward Curvature Steering (Anti-Fish-tailing):
   While driving forward, wheels are already in motion (zero stiction). Applies smooth,
   continuous curvature steering with wider deadbands to reject natural human walking sway,
   completely eliminating left-right hunting oscillations.
4. Precision Turret Pivoting:
   When stopped (holding the sweet spot), uses Stage 1 PD anti-overshoot easing with virtual brake damping.
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
    kp_dist: float = 0.50              # Proportional gain for forward velocity
    min_forward_speed: float = 0.15    # Minimum forward speed (m/s) to overcome DC motor gearbox stiction
    max_forward_speed: float = 0.40    # Maximum cruising forward speed (m/s)
    emergency_stop_dist_m: float = 0.45 # Distance threshold below which robot halts immediately
    
    # ── Stationary Turret Heading Controls (PD Anti-Overshoot) ────────────────
    # Active ONLY when v_linear == 0 (pivoting on the spot)
    kp_turn: float = 0.30              # Proportional turn gain
    kd_turn: float = 0.10              # Derivative damping gain (virtual brake on center approach)
    deadband_turn_exit: float = 0.10   # Trigger threshold to start steering (10% of frame center)
    deadband_turn_enter: float = 0.05  # Settle threshold to stop steering (5% of frame center)
    min_turn_speed: float = 0.10       # Minimum gentle turn speed (m/s)
    max_turn_speed: float = 0.22       # Maximum turn speed clamp (m/s)
    
    # ── Forward Motion Curvature Steering (Anti-Fish-tailing) ─────────────────
    # Active when v_linear > 0 (curving while driving forward)
    # Uses continuous proportional curvature without discrete stiction jumps
    deadband_forward_exit: float = 0.14   # Ignores human walking sway (< 14% of frame center)
    deadband_forward_enter: float = 0.07  # Settles back to straight drive
    kp_turn_forward: float = 0.25         # Gentle proportional curvature gain
    kd_turn_forward: float = 0.08         # Derivative rate damping during forward motion
    max_turn_forward_speed: float = 0.14  # Maximum differential turn offset while cruising
    
    # ── Forward Startup Kick ──────────────────────────────────────────────────
    # Kick is applied ONLY when starting forward drive from a complete stop,
    # overcoming static friction of the entire vehicle. (Pivots do NOT trigger kick).
    enable_forward_kick: bool = True
    forward_kick_speed: float = 0.45
    forward_kick_duration_s: float = 0.12

    # ── Smooth Stop & Acceleration Ramping ────────────────────────────────────
    enable_smooth_stop: bool = True     # Soft deceleration ramp when stopping or changing speeds
    max_accel_mps2: float = 0.70        # Slew-rate acceleration limit (m/s^2)
    max_decel_mps2: float = 1.00        # Slew-rate deceleration limit (m/s^2) for responsive sweet-spot entry

    # ── Smart Target Recovery (Search in Last Moved Direction) ────────────────
    enable_search_recovery: bool = True # Turn in last known direction instead of stopping blind
    search_turn_speed: float = 0.14     # Gentle pivot speed while searching (m/s)
    search_duration_s: float = 2.0      # How long to sweep in the last direction before giving up (seconds)
    
    # ── Safety & Timeouts ─────────────────────────────────────────────────────
    target_timeout_s: float = 1.0      # If target is lost for > 1.0s, begin search recovery or decelerate to 0
    confidence_threshold: float = 0.40 # Minimum detection confidence
    
    # ── Hardware & Camera Settings ────────────────────────────────────────────
    wheel_base_m: float = 0.134
    wheel_radius_m: float = 0.0215
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
        self._emergency_stop: bool = False

        # State tracking
        self.state: str = "INITIALIZING"
        self._running: bool = False
        self._tx_thread: Optional[threading.Thread] = None
        self._loop_thread: Optional[threading.Thread] = None

        # Heading PD & Hysteresis state (synchronized to camera frames)
        self._is_steering: bool = False
        self._last_error: float = 0.0
        self._last_obs_timestamp: float = 0.0
        self._smoothed_d_err: float = 0.0

        # Smart target recovery tracking (memory of last seen position)
        self._last_seen_direction: Optional[str] = None
        self._last_seen_time: float = 0.0

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
        """Continuously streams velocity packets at 20 Hz with smooth acceleration and deceleration ramping."""
        interval = 1.0 / self.config.tx_rate_hz
        max_delta_accel = self.config.max_accel_mps2 * interval
        max_delta_decel = self.config.max_decel_mps2 * interval

        current_cmd_l = 0.0
        current_cmd_r = 0.0

        while self._running:
            t0 = time.monotonic()

            with self._serial_lock:
                emergency = self._emergency_stop
                vl = self._target_v_left
                vr = self._target_v_right
                is_kicking = (t0 < self._forward_kick_until)

            if emergency:
                # Immediate hard hardware stop: bypass slew-rate glide
                current_cmd_l = 0.0
                current_cmd_r = 0.0
            else:
                # Apply forward breakaway kick if active
                if is_kicking:
                    kick = self.config.forward_kick_speed
                    target_l = kick if vl > 0.05 else vl
                    target_r = kick if vr > 0.05 else vr
                else:
                    target_l = vl
                    target_r = vr

                # Smooth Acceleration and Deceleration Slew-Rate Limiting:
                if self.config.enable_smooth_stop:
                    # Left wheel
                    diff_l = target_l - current_cmd_l
                    if diff_l > 0:
                        current_cmd_l += min(diff_l, max_delta_accel)
                    else:
                        current_cmd_l -= min(-diff_l, max_delta_decel)

                    # Right wheel
                    diff_r = target_r - current_cmd_r
                    if diff_r > 0:
                        current_cmd_r += min(diff_r, max_delta_accel)
                    else:
                        current_cmd_r -= min(-diff_r, max_delta_decel)
                else:
                    current_cmd_l = target_l
                    current_cmd_r = target_r

            if self._ser is not None:
                try:
                    packet = f"{current_cmd_l:.2f},{current_cmd_r:.2f}\n".encode("ascii")
                    self._ser.write(packet)
                    self._ser.flush()
                except Exception:
                    pass

            elapsed = time.monotonic() - t0
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _set_wheel_velocities(
        self,
        vl: float,
        vr: float,
        is_forward_accel: bool = False,
        immediate_stop: bool = False,
    ) -> None:
        """Update commanded wheel velocities with forward kick trigger and emergency override."""
        now = time.monotonic()
        with self._serial_lock:
            if immediate_stop:
                self._emergency_stop = True
                self._target_v_left = 0.0
                self._target_v_right = 0.0
                self._forward_kick_until = 0.0
                return

            self._emergency_stop = False
            was_stopped = (abs(self._target_v_left) < 0.02 and abs(self._target_v_right) < 0.02)
            moving_forward = (vl > 0.05 and vr > 0.05)

            if was_stopped and moving_forward and is_forward_accel and self.config.enable_forward_kick:
                self._forward_kick_until = now + self.config.forward_kick_duration_s

            self._target_v_left = vl
            self._target_v_right = vr

    def _estimate_human_distance(
        self,
        obs: TargetObservation,
        left_dist: float = 9.99,
        mid_dist: float = 9.99,
        right_dist: float = 9.99,
    ) -> Tuple[float, str]:
        """
        Multi-Sensor Distance Fusion:
        1. If Center ToF sensor has a valid reading (< 1.85m), prioritize it (millimeter laser accuracy).
        2. If Center missed, check Left/Right ToF sensors matching person bearing.
        3. If all ToF sensors are out of range (> 1.85m) or missed, evaluate Vision Bounding Box:
           - If bounding box fills >= 65% of screen height, target is close (< 1.0m)!
           - Otherwise, compute calibrated distance proxy from normalized box height.
        """
        # Priority 1: Direct Center ToF laser measurement
        if 0.05 < mid_dist < 1.85:
            return mid_dist, "TOF_LASER"

        # Priority 2: Directional ToF laser measurement based on tracked lateral error
        if obs.target_valid:
            if obs.horizontal_error < -0.10 and 0.05 < left_dist < 1.70:
                return left_dist, "TOF_LEFT"
            if obs.horizontal_error > +0.10 and 0.05 < right_dist < 1.70:
                return right_dist, "TOF_RIGHT"

            # Check if either side ToF senses an obstacle in close proximity
            valid_sides = [d for d in (left_dist, right_dist) if 0.05 < d < 1.20]
            if valid_sides:
                return min(valid_sides), "TOF_NEAR"

        # Priority 3: Vision Bounding Box Height Proxy with Physical Sanity Clamps
        if obs.target_valid and obs.bbox is not None and len(obs.bbox) == 4:
            x1, y1, x2, y2 = obs.bbox
            box_h = max(1.0, float(y2 - y1))
            frame_h = float(self.config.camera_height)
            h_norm = box_h / frame_h

            # If person fills >= 65% of vertical frame or is clipped at top and bottom:
            # Human is very close to the camera (sweet spot or closer), NEVER charge forward!
            if h_norm >= 0.65 or (y1 <= 12 and y2 >= frame_h - 12):
                return 0.95, "VISION_CLOSE"

            if h_norm >= 0.45:
                # Inside or right at edge of sweet spot (1.0m - 1.35m)
                d_est = max(0.95, min(1.35, 0.65 / h_norm))
                return round(d_est, 2), "VISION_SWEET"

            # Farther away (> 1.4m)
            d_est = max(1.40, min(3.50, 0.65 / h_norm))
            return round(d_est, 2), "VISION_FAR"

        # Priority 4: Fallback ToF if available
        if 0.05 < mid_dist < 2.0:
            return mid_dist, "TOF_LASER"

        return 1.20, "DEFAULT"

    def step(
        self,
        obs: Optional[TargetObservation] = None,
        mock_mid_tof: Optional[float] = None,
        mock_tofs: Optional[Tuple[float, float, float]] = None,
    ) -> Tuple[float, float, str]:
        """
        Execute one control cycle:
        1. Hard hardware safety check across all 3 ToF lasers (< 0.45m -> immediate stop).
        2. Distance regulation: linear velocity (v_linear) with smooth ramp into sweet spot.
        3. Heading regulation:
           - If moving forward: continuous proportional curvature steering (anti-fish-tailing).
           - If stationary: precision turret pivot with anti-overshoot PD damping.
        4. Kinematics fusion -> (v_left, v_right).
        """
        if obs is None:
            obs = self.vision.get_latest_observation()

        self._last_observation = obs
        now = time.monotonic()

        # Check target freshness and validity
        is_fresh = (now - obs.timestamp) <= self.config.target_timeout_s
        if not obs.target_valid or not is_fresh or obs.confidence < self.config.confidence_threshold:
            self._is_steering = False
            self._last_error = 0.0

            # Smart target recovery: sweep gently in the direction the person was last seen
            elapsed_lost = now - self._last_seen_time
            if (
                self.config.enable_search_recovery
                and self._last_seen_direction is not None
                and elapsed_lost <= self.config.search_duration_s
            ):
                search_speed = self.config.search_turn_speed
                if self._last_seen_direction == "RIGHT":
                    vl = +search_speed
                    vr = -search_speed
                    self.state = "SEARCH_RECOVERY_RIGHT"
                else:
                    vl = -search_speed
                    vr = +search_speed
                    self.state = "SEARCH_RECOVERY_LEFT"
                self._set_wheel_velocities(vl, vr)
                return vl, vr, self.state

            # Search expired or no last direction: smooth halt
            self._set_wheel_velocities(0.0, 0.0)
            self.state = "SEARCHING"
            return 0.0, 0.0, self.state

        # Remember last seen direction for smart target recovery if target is lost
        if abs(obs.horizontal_error) > 0.05:
            self._last_seen_direction = "RIGHT" if obs.horizontal_error > 0 else "LEFT"
        self._last_seen_time = now

        # Get ToF distance readings [left, middle, right]
        if mock_tofs is not None:
            left_d, mid_d, right_d = mock_tofs
        elif mock_mid_tof is not None:
            left_d, mid_d, right_d = 9.99, mock_mid_tof, 9.99
        else:
            left_d, mid_d, right_d = self.sensors.get_distances()

        # ── 1. HARD HARDWARE SAFETY SHIELD ────────────────────────────────────
        # If ANY ToF sensor sees an obstacle or human closer than emergency stop threshold:
        # Slam on the physical brakes immediately!
        valid_tofs = [d for d in (left_d, mid_d, right_d) if 0.03 < d < 2.0]
        min_tof = min(valid_tofs) if valid_tofs else None

        if min_tof is not None and min_tof < self.config.emergency_stop_dist_m:
            self._set_wheel_velocities(0.0, 0.0, immediate_stop=True)
            self.state = "SAFETY_STOP"
            return 0.0, 0.0, self.state

        # Sensor fusion for human distance
        dist_m, dist_source = self._estimate_human_distance(
            obs, left_dist=left_d, mid_dist=mid_d, right_dist=right_d
        )

        if dist_m < self.config.emergency_stop_dist_m:
            self._set_wheel_velocities(0.0, 0.0, immediate_stop=True)
            self.state = "SAFETY_STOP"
            return 0.0, 0.0, self.state

        # ── 2. Distance Regulation (Linear Velocity) ──────────────────────────
        sweet_spot_min = self.config.target_distance_m - self.config.distance_deadband_m  # e.g. 1.00m
        sweet_spot_max = self.config.target_distance_m + self.config.distance_deadband_m  # e.g. 1.40m

        if dist_m > sweet_spot_max:
            # Person is walking away: drive forward with smooth progressive ramp
            excess_dist = dist_m - sweet_spot_max
            v_linear = self.config.min_forward_speed + (
                self.config.max_forward_speed - self.config.min_forward_speed
            ) * min(1.0, excess_dist / 0.80)
            is_forward_drive = True
        else:
            # Inside sweet spot (1.0m - 1.4m) or close: hold forward position
            v_linear = 0.0
            is_forward_drive = False

        # ── 3. Frame-Synchronized Heading Rate Calculation ────────────────────
        h_err = obs.horizontal_error  # -1.0 (left) to +1.0 (right)

        if obs.timestamp > self._last_obs_timestamp:
            dt = max(0.02, obs.timestamp - self._last_obs_timestamp)
            raw_d_err = (h_err - self._last_error) / dt
            # Low-pass filter to reject single-frame bounding-box jitter
            self._smoothed_d_err = 0.40 * raw_d_err + 0.60 * self._smoothed_d_err
            self._last_error = h_err
            self._last_obs_timestamp = obs.timestamp

        d_err = self._smoothed_d_err

        # ── 4. Heading Regulation (Curvature Steering vs Stationary Pivot) ────
        if is_forward_drive:
            # ── BRANCH A: Forward Curvature Steering (Anti-Fish-tailing) ──────
            # While driving forward, wheels are already in motion (zero stiction).
            # We use a wider deadband to ignore natural human walking sway (torso swaying +/- 10cm).
            active_deadband = (
                self.config.deadband_forward_enter
                if self._is_steering
                else self.config.deadband_forward_exit
            )

            if abs(h_err) <= active_deadband:
                v_turn = 0.0
                self._is_steering = False
            else:
                self._is_steering = True
                excess = abs(h_err) - self.config.deadband_forward_enter
                raw_turn = self.config.kp_turn_forward * excess

                # Rate damping: if target is closing in on center line, ease off turn
                is_closing_in = (h_err > 0 and d_err < -0.15) or (h_err < 0 and d_err > 0.15)
                if is_closing_in:
                    raw_turn *= max(0.40, 1.0 - 0.25 * abs(d_err))

                turn_speed = max(0.03, min(self.config.max_turn_forward_speed, raw_turn))
                v_turn = turn_speed if h_err > 0 else -turn_speed

            # If heading error is significant (> 0.20), scale down forward speed slightly
            if abs(h_err) > 0.20:
                scale_factor = max(0.35, math.cos(math.radians(obs.bearing_deg)))
                v_linear *= scale_factor

        else:
            # ── BRANCH B: Stationary Turret Pivot (Anti-Overshoot) ────────────
            # Robot is stopped (holding sweet spot or reorienting).
            # Uses Stage 1 PD anti-overshoot easing with min_turn_speed breakaway.
            active_deadband = (
                self.config.deadband_turn_enter
                if self._is_steering
                else self.config.deadband_turn_exit
            )

            if abs(h_err) <= active_deadband:
                v_turn = 0.0
                self._is_steering = False
            else:
                self._is_steering = True
                err_mag = abs(h_err)
                span = max(0.01, 1.0 - self.config.deadband_turn_enter)
                norm_mag = min(1.0, max(0.0, (err_mag - self.config.deadband_turn_enter) / span))
                base_turn_speed = self.config.min_turn_speed + (
                    self.config.max_turn_speed - self.config.min_turn_speed
                ) * (norm_mag ** 1.3)

                is_closing_in = (h_err > 0 and d_err < -0.20) or (h_err < 0 and d_err > 0.20)
                if is_closing_in:
                    damping_factor = max(0.45, 1.0 - 0.20 * abs(d_err))
                    turn_speed = base_turn_speed * damping_factor
                else:
                    turn_speed = base_turn_speed

                turn_speed = max(self.config.min_turn_speed, min(self.config.max_turn_speed, turn_speed))
                v_turn = turn_speed if h_err > 0 else -turn_speed

        # ── 5. Kinematics Fusion ──────────────────────────────────────────────
        # Left wheel = v_linear + v_turn
        # Right wheel = v_linear - v_turn
        vl = v_linear + v_turn
        vr = v_linear - v_turn

        # Clamp wheel speeds to safe hardware limits
        max_limit = self.config.max_forward_speed + 0.05
        vl = max(-max_limit, min(max_limit, vl))
        vr = max(-max_limit, min(max_limit, vr))

        # Determine human-readable state
        if v_linear > 0.05:
            if abs(v_turn) > 0.02:
                self.state = "FOLLOW_CURVE_RIGHT" if v_turn > 0 else "FOLLOW_CURVE_LEFT"
            else:
                self.state = "FOLLOW_STRAIGHT"
        else:
            if abs(v_turn) > 0.02:
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
            left_d, mid_d, right_d = self.sensors.get_distances()
            dist_m, source = self._estimate_human_distance(
                obs, left_dist=left_d, mid_dist=mid_d, right_dist=right_d
            )

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
                "bbox": list(obs.bbox) if (obs.target_valid and obs.bbox is not None) else None,
                "tof_left": round(left_d, 2),
                "tof_mid": round(mid_d, 2),
                "tof_right": round(right_d, 2),
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
        self._set_wheel_velocities(0.0, 0.0, immediate_stop=True)

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
