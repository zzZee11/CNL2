"""
human_follower/stage3_follower.py
---------------------------------
Stage 3 Human Follower: Exact Stage 2 Follower with Real-Time ToF Obstacle Avoidance.

Architecture:
1. Base Follower: Exact Stage 2 Architecture
   - Distance Regulation: Target distance tracking (~1.2m) with +/- 20cm sweet spot (1.0m - 1.4m).
   - Sensor Fusion: Center VL53L0X ToF laser for millimeter close-range precision (< 1.85m),
     switching smoothly to calibrated Vision Proxy for long-range walking pursuit (> 1.85m).
   - Heading Regulation: Stage 1 PD anti-overshoot steering with enter/exit hysteresis deadbands
     and derivative approach damping to eliminate hunting/oscillations.
   - Slew-Rate Ramping & Forward Kick: Soft acceleration/deceleration with stiction breakaway kick.
   - Smart Target Recovery: Sweeps in the last known direction if human leaves frame.
2. Layered Obstacle Avoidance (on top of Stage 2):
   - Continuous monitoring of 3x VL53L0X ToF sensors: Left (+45 deg), Center (0 deg), Right (-45 deg).
   - When obstacles enter obstacle_safe_dist_m (default 0.65m):
     - Left obstacle generates a smooth steering bias to the Right (Left wheel faster).
     - Right obstacle generates a smooth steering bias to the Left (Right wheel faster).
     - Center obstacle (when not the tracked person) slows forward speed to prevent collision.
   - Safety Shield: Instant halt if any obstacle or human gets closer than emergency_stop_dist_m (< 0.35m).
"""

import math
import os
import sys
import time
import threading
from dataclasses import dataclass
from typing import Dict, Any, Optional, Tuple, List

# Enable importing packages from project root
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from vision.config import VisionConfig
from vision.vision_service import VisionService
from vision.target_data import TargetObservation
from autonomous_navigation.sensors import DistanceSensorManager


@dataclass
class Stage3Config:
    # ── UART & Serial Communication ───────────────────────────────────────────
    serial_port: str = "/dev/serial0"
    serial_baud: int = 115200
    tx_rate_hz: float = 20.0           # 20 Hz transmission to satisfy ESP32 500ms watchdog
    watchdog_timeout_s: float = 0.45   # Serial command timeout
    
    # ── Stage 2 Distance Regulation Controls ──────────────────────────────────
    target_distance_m: float = 1.00    # Desired distance to maintain from the human (meters)
    distance_deadband_m: float = 0.20  # +/- 20 cm sweet spot: between (target - 0.2) and (target + 0.2), v_lin = 0
    kp_dist: float = 0.55              # Proportional gain for forward velocity: v = Kp * distance_error
    min_forward_speed: float = 0.20    # Minimum forward speed (m/s) to overcome DC motor gearbox stiction
    max_forward_speed: float = 0.45    # Maximum cruising forward speed (m/s)
    emergency_stop_dist_m: float = 0.25 # Distance threshold below which robot halts immediately
    
    # ── Obstacle Avoidance Parameters (Layered on top of Stage 2) ─────────────
    obstacle_safe_dist_m: float = 0.50 # Distance below which obstacles start repelling robot (meters)
    k_rep: float = 0.10                # Obstacle repulsion gain
    max_rep_turn_speed: float = 0.05   # Maximum differential turn bias from obstacles (m/s) - gentle steering
    swap_left_right_tofs: bool = False # Swap left and right ToF inputs if physical wiring is reversed
    # Sensor mount angles in radians relative to robot forward (+ is Left, - is Right)
    sensor_angles_rad: Tuple[float, float, float] = (
        math.radians(+45.0),  # Left ToF
        math.radians(0.0),    # Middle ToF
        math.radians(-45.0),  # Right ToF
    )

    # ── Stage 1 Heading Controls (PD Anti-Overshoot) ───────────────────────────
    kp_turn: float = 0.35              # Proportional turn gain
    kd_turn: float = 0.12              # Derivative damping gain (virtual brake on center approach)
    deadband_turn_exit: float = 0.10   # Trigger threshold to start steering (10% of frame center)
    deadband_turn_enter: float = 0.05  # Settle threshold to stop steering (5% of frame center)
    min_turn_speed: float = 0.12       # Minimum gentle turn speed (m/s)
    max_turn_speed: float = 0.15       # Maximum turn speed clamp (m/s)
    
    # ── Forward Startup Kick ──────────────────────────────────────────────────
    enable_forward_kick: bool = True
    forward_kick_speed: float = 0.50
    forward_kick_duration_s: float = 0.12

    # ── Smooth Stop & Acceleration Ramping ────────────────────────────────────
    enable_smooth_stop: bool = True     # Soft deceleration ramp when stopping or changing speeds
    max_accel_mps2: float = 0.80        # Slew-rate acceleration limit (m/s^2)
    max_decel_mps2: float = 0.70        # Slew-rate deceleration limit (m/s^2) for silky-smooth stops

    # ── Smart Target Recovery (Search in Last Moved Direction) ────────────────
    enable_search_recovery: bool = True # Turn in last known direction instead of stopping blind
    search_turn_speed: float = 0.15     # Gentle pivot speed while searching (m/s)
    search_duration_s: float = 2.0      # How long to sweep in the last direction before giving up (seconds)
    
    # ── Safety & Timeouts ─────────────────────────────────────────────────────
    target_timeout_s: float = 1.0      # Target lost timeout
    confidence_threshold: float = 0.40 # Minimum detection confidence
    
    # ── Hardware & Camera Settings ────────────────────────────────────────────
    wheel_base_m: float = 0.134        # 13.4 cm track width
    wheel_radius_m: float = 0.0215     # 2.15 cm wheel radius
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
        if self.obstacle_safe_dist_m <= self.emergency_stop_dist_m:
            raise ValueError("obstacle_safe_dist_m must be greater than emergency_stop_dist_m.")
        if self.min_forward_speed > self.max_forward_speed:
            raise ValueError("min_forward_speed cannot exceed max_forward_speed.")


class Stage3ObstacleFollower:
    """
    Stage 3 Controller: Built directly on Stage 2's proven distance and heading follower,
    with layered 3x ToF reactive obstacle avoidance.
    """

    def __init__(self, config: Optional[Stage3Config] = None) -> None:
        self.config = config or Stage3Config()
        self.config.validate()

        # Serial state
        self._ser: Any = None
        self._serial_lock = threading.Lock()
        self._target_v_left: float = 0.0
        self._target_v_right: float = 0.0
        self._forward_kick_until: float = 0.0
        self._emergency_stop: bool = False
        self._last_obs_bias: float = 0.0

        # State tracking
        self.state: str = "INITIALIZING"
        self._running: bool = False
        self._tx_thread: Optional[threading.Thread] = None
        self._loop_thread: Optional[threading.Thread] = None

        # Heading PD & Hysteresis state (Exact Stage 2)
        self._is_steering: bool = False
        self._last_h_err: float = 0.0
        self._last_h_err_time: float = 0.0

        # Smart target recovery tracking (Exact Stage 2)
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
            print(f"[Stage3Follower] Could not open serial port {self.config.serial_port} ({e}).")
            print("[Stage3Follower] Running in mock serial mode.")
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
                current_cmd_l = 0.0
                current_cmd_r = 0.0
            else:
                if is_kicking:
                    kick = self.config.forward_kick_speed
                    target_l = kick if vl > 0.05 else vl
                    target_r = kick if vr > 0.05 else vr
                else:
                    target_l = vl
                    target_r = vr

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

    def _estimate_human_distance(self, obs: TargetObservation, mid_dist: float = 9.99) -> Tuple[float, str]:
        """
        Exact Stage 2 Sensor Fusion for Human Distance:
        1. If Center ToF sensor has a valid reading (< 1.85m) and person is roughly centered -> Use ToF laser.
        2. Else -> Use calibrated vision height proxy.
        3. If person is centered but ToF reads > 1.85m -> returns 2.2m (TOF_OUT_OF_RANGE).
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

    def calculate_apf(
        self,
        obs: TargetObservation,
        dist_m: float,
        left_dist: float,
        mid_dist: float,
        right_dist: float,
    ) -> Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]]:
        """
        Computes vector representation of attraction (Vision/Target) and repulsion (3x ToF obstacles)
        for telemetry overlay compass display.
        """
        sweet_spot_max = self.config.target_distance_m + self.config.distance_deadband_m
        human_theta = -math.radians(obs.bearing_deg)

        if dist_m > sweet_spot_max:
            excess = dist_m - sweet_spot_max
            att_mag = min(1.0, excess / 0.80)
            f_att_x = att_mag * math.cos(human_theta)
            f_att_y = att_mag * math.sin(human_theta)
        else:
            f_att_x = 0.0
            f_att_y = 0.0

        f_rep_x = 0.0
        f_rep_y = 0.0
        safe_d = self.config.obstacle_safe_dist_m
        sensor_readings = [left_dist, mid_dist, right_dist]
        sensor_angles = self.config.sensor_angles_rad

        for i, (dist, angle) in enumerate(zip(sensor_readings, sensor_angles)):
            if 0.03 < dist < safe_d:
                # Disambiguate center sensor from human target
                if i == 1 and abs(obs.horizontal_error) < 0.25 and abs(dist - dist_m) < 0.25:
                    continue

                rep_mag = self.config.k_rep * ((1.0 / dist) - (1.0 / safe_d)) * (1.0 / (dist ** 2))
                rep_mag = min(2.0, rep_mag)
                push_angle = angle + math.pi
                f_rep_x += rep_mag * math.cos(push_angle)
                f_rep_y += rep_mag * math.sin(push_angle)

        f_net_x = f_att_x + f_rep_x
        f_net_y = f_att_y + f_rep_y
        return (f_att_x, f_att_y), (f_rep_x, f_rep_y), (f_net_x, f_net_y)

    def step(
        self,
        obs: Optional[TargetObservation] = None,
        mock_mid_tof: Optional[float] = None,
        mock_tofs: Optional[Tuple[float, float, float]] = None,
    ) -> Tuple[float, float, str]:
        """
        Execute one control cycle:
        1. Exact Stage 2 Target Distance & Heading Evaluation.
        2. Layered 3x ToF Obstacle Avoidance:
           - Hard safety brake if hazard < 0.35m.
           - Left obstacle (< 0.65m) deflects steering Right.
           - Right obstacle (< 0.65m) deflects steering Left.
        3. Kinematics Fusion -> (v_left, v_right).
        """
        if obs is None:
            obs = self.vision.get_latest_observation()

        self._last_observation = obs
        now = time.monotonic()

        # Check target freshness and validity (Exact Stage 2)
        is_fresh = (now - obs.timestamp) <= self.config.target_timeout_s
        if not obs.target_valid or not is_fresh or obs.confidence < self.config.confidence_threshold:
            self._is_steering = False
            self._last_h_err = 0.0
            self._last_h_err_time = now

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

        # Read 3x ToF distance readings [left, middle, right]
        if mock_tofs is not None:
            left_d, mid_d, right_d = mock_tofs
        elif mock_mid_tof is not None:
            left_d, mid_d, right_d = 9.99, mock_mid_tof, 9.99
        else:
            left_d, mid_d, right_d = self.sensors.get_distances()

        # Handle optional left/right sensor swap if wiring is physically inverted
        if self.config.swap_left_right_tofs:
            left_d, right_d = right_d, left_d

        # ── 1. HARD SAFETY SHIELD (< emergency_stop_dist_m immediate halt) ────
        valid_tofs = [d for d in (left_d, mid_d, right_d) if 0.03 < d < 2.0]
        min_tof = min(valid_tofs) if valid_tofs else 9.99

        if min_tof < self.config.emergency_stop_dist_m:
            self._set_wheel_velocities(0.0, 0.0, immediate_stop=True)
            self._last_obs_bias = 0.0
            self.state = "SAFETY_STOP"
            return 0.0, 0.0, self.state

        # Exact Stage 2 Human Distance Estimation
        dist_m, dist_source = self._estimate_human_distance(obs, mid_dist=mid_d)

        if dist_m < self.config.emergency_stop_dist_m:
            self._set_wheel_velocities(0.0, 0.0, immediate_stop=True)
            self._last_obs_bias = 0.0
            self.state = "SAFETY_STOP"
            return 0.0, 0.0, self.state

        # ── 2. Stage 2 Distance Regulation (Linear Velocity) ──────────────────
        sweet_spot_max = self.config.target_distance_m + self.config.distance_deadband_m

        if dist_m > sweet_spot_max:
            dist_error = dist_m - self.config.target_distance_m
            raw_v_lin = self.config.kp_dist * dist_error
            v_linear = max(self.config.min_forward_speed, min(self.config.max_forward_speed, raw_v_lin))
            is_forward_drive = True
        else:
            v_linear = 0.0
            is_forward_drive = False

        # ── 3. Stage 2 Heading Regulation (PD Anti-Overshoot) ─────────────────
        h_err = obs.horizontal_error
        active_turn_deadband = self.config.deadband_turn_enter if self._is_steering else self.config.deadband_turn_exit

        if abs(h_err) <= active_turn_deadband:
            v_turn_human = 0.0
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
            v_turn_human = turn_speed if h_err > 0 else -turn_speed

        # ── 4. LAYERED TOF OBSTACLE AVOIDANCE ─────────────────────────────────
        safe_d = self.config.obstacle_safe_dist_m
        em_d = self.config.emergency_stop_dist_m
        delta_turn_obs = 0.0
        avoiding_obstacle = False

        # Only apply obstacle avoidance turn bias when robot is actively driving forward!
        # When stopped or holding sweet spot, robot holds position facing the target.
        if is_forward_drive:
            # Left ToF Obstacle -> Smooth quadratic deflection to the Right (+ turn bias)
            if 0.05 < left_d < safe_d:
                ratio = max(0.0, min(1.0, (safe_d - left_d) / max(0.01, safe_d - em_d)))
                rep_turn_l = self.config.max_rep_turn_speed * (ratio ** 2)
                delta_turn_obs += rep_turn_l
                avoiding_obstacle = True

            # Right ToF Obstacle -> Smooth quadratic deflection to the Left (- turn bias)
            if 0.05 < right_d < safe_d:
                ratio = max(0.0, min(1.0, (safe_d - right_d) / max(0.01, safe_d - em_d)))
                rep_turn_r = self.config.max_rep_turn_speed * (ratio ** 2)
                delta_turn_obs -= rep_turn_r
                avoiding_obstacle = True

            # Middle ToF Obstacle:
            # When tracking a human, the human in front is handled by distance regulation.
            # Only treat center sensor as an obstacle if it is NOT the tracked human:
            if 0.05 < mid_d < safe_d:
                is_human_center = obs.target_valid and abs(obs.horizontal_error) < 0.30
                if not is_human_center:
                    clearance_ratio = max(0.0, (mid_d - em_d) / max(0.01, safe_d - em_d))
                    v_linear *= max(0.20, clearance_ratio)
                    # Deflect toward side with noticeably more clearance
                    if abs(delta_turn_obs) < 0.01:
                        if left_d > right_d + 0.10:
                            delta_turn_obs = -0.06  # Steer left towards open clearance
                            avoiding_obstacle = True
                        elif right_d > left_d + 0.10:
                            delta_turn_obs = +0.06  # Steer right towards open clearance
                            avoiding_obstacle = True

        self._last_obs_bias = delta_turn_obs

        # Combine Human Heading Turn and Obstacle Steering Bias
        v_turn = v_turn_human + delta_turn_obs
        v_turn = max(-self.config.max_turn_speed, min(self.config.max_turn_speed, v_turn))

        # ── 5. Kinematics Fusion ──────────────────────────────────────────────
        if is_forward_drive:
            # Forward drive with turn factor reduction for sharp turns
            if abs(h_err) > 0.20 or abs(delta_turn_obs) > 0.05:
                scale_factor = max(0.35, math.cos(math.radians(obs.bearing_deg)))
                v_linear *= scale_factor

            # Modulate wheel speeds while ensuring inner wheel doesn't drop into reverse during forward pursuit
            if v_turn > 0:  # Steering right: left wheel faster, right wheel slower
                vl = min(self.config.max_forward_speed, v_linear + v_turn)
                vr = max(0.06, v_linear - v_turn)
            else:           # Steering left: right wheel faster, left wheel slower
                vl = max(0.06, v_linear + v_turn)
                vr = min(self.config.max_forward_speed, v_linear - v_turn)
        else:
            # Pure pivoting in place (Stage 1 anti-overshoot heading adjustment)
            vl = v_turn
            vr = -v_turn

        max_limit = self.config.max_forward_speed + 0.05
        vl = max(-max_limit, min(max_limit, vl))
        vr = max(-max_limit, min(max_limit, vr))

        # Determine human-readable state
        if avoiding_obstacle:
            self.state = "AVOIDING_OBSTACLE_RIGHT" if delta_turn_obs > 0 else "AVOIDING_OBSTACLE_LEFT"
        elif v_linear > 0.05:
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
        """20 Hz control loop executing distance and heading tracking with obstacle avoidance."""
        interval = 1.0 / self.config.tx_rate_hz

        while self._running:
            t0 = time.monotonic()
            vl, vr, state = self.step()

            # Record telemetry
            obs = self._last_observation
            v_telem = self.vision.get_telemetry()
            left_d, mid_d, right_d = self.sensors.get_distances()
            dist_m, source = self._estimate_human_distance(obs, mid_dist=mid_d)
            f_att, f_rep, f_net = self.calculate_apf(obs, dist_m, left_d, mid_d, right_d)

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
                "obs_bias": round(self._last_obs_bias, 3),
                "f_att": [round(f_att[0], 2), round(f_att[1], 2)],
                "f_rep": [round(f_rep[0], 2), round(f_rep[1], 2)],
                "f_net": [round(f_net[0], 2), round(f_net[1], 2)],
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
