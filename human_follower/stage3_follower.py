"""
human_follower/stage3_follower.py
---------------------------------
Stage 3 Human Follower: Combined Vision-Only Human Following with Real-Time ToF APF Obstacle Avoidance.

Architecture:
1. Target Finding & Distance Estimation (100% VISION ONLY):
   - The human is detected, tracked, and measured strictly using the camera pipeline.
   - Bearing angle is calculated from horizontal pixel offset relative to the optical axis.
   - Distance to human is estimated via calibrated perspective geometry (Vision Proxy).
   - Distance sensors are NEVER used to estimate or track the human.
2. Obstacle Detection & Avoidance (3x VL53L0X ToF Distance Sensors):
   - Left (+45 deg), Center (0 deg), and Right (-45 deg) ToF lasers are dedicated exclusively
     to detecting surrounding physical obstacles (walls, furniture, trash cans, etc.).
   - When an obstacle is detected within obstacle_safe_dist_m (default 0.65m), an inverse-square
     repulsive force F_rep pushes the robot away from the obstacle.
3. Artificial Potential Field (APF) Blending:
   - F_net = F_att (Vision Attraction) + F_rep (Sensor Repulsion).
   - Deflects robot heading dynamically around obstacles while continuously pursuing the human.
4. Hard Hardware Safety Shield:
   - If any sensor detects an obstacle closer than emergency_stop_dist_m (< 0.35m),
     immediately commands an emergency stop.
5. Kinematics & Slew-Rate Ramping:
   - Converts F_net to differential wheel speeds (v_left, v_right) with acceleration/deceleration
     slew-rate limiting and motor stiction startup kick.
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
    
    # ── Vision Distance Regulation (Sweet Spot & Speeds) ──────────────────────
    target_distance_m: float = 1.20    # Desired distance to maintain from the human (meters)
    distance_deadband_m: float = 0.20  # +/- 20 cm sweet spot (1.0m - 1.4m), v_lin = 0
    kp_dist: float = 0.55              # Proportional gain for forward velocity
    min_forward_speed: float = 0.15    # Minimum forward speed (m/s) to overcome DC motor gearbox stiction
    max_forward_speed: float = 0.45    # Maximum cruising forward speed (m/s)
    emergency_stop_dist_m: float = 0.35 # Distance threshold below which robot halts immediately
    
    # ── Artificial Potential Field (APF) Parameters ───────────────────────────
    kp_att: float = 1.20               # Attractive gain towards human target
    k_rep: float = 0.25                # Repulsive gain away from obstacles
    obstacle_safe_dist_m: float = 0.65 # Distance below which obstacles start exerting repulsive force
    max_rep_force: float = 2.0         # Saturation clamp on repulsive force to avoid extreme spikes
    # Sensor mount angles in radians relative to robot forward (+ is Left, - is Right)
    sensor_angles_rad: Tuple[float, float, float] = (
        math.radians(+45.0),  # Left ToF
        math.radians(0.0),    # Middle ToF
        math.radians(-45.0),  # Right ToF
    )

    # ── Heading & Steering Controls (Curvature & Pivots) ──────────────────────
    deadband_forward_exit: float = 0.14   # Ignores human walking sway (< 14% of frame center)
    deadband_forward_enter: float = 0.07  # Settles back to straight drive
    kp_turn_forward: float = 0.28         # Curvature steering gain when moving forward
    kd_turn_forward: float = 0.08         # Derivative rate damping during forward motion
    max_turn_forward_speed: float = 0.18  # Maximum differential turn offset while cruising (m/s)
    
    min_turn_speed: float = 0.10          # Gentle pivot breakaway speed when stationary (m/s)
    max_turn_speed: float = 0.22          # Maximum pivot speed clamp when stationary (m/s)
    deadband_turn_exit: float = 0.10      # Trigger threshold for stationary pivot
    deadband_turn_enter: float = 0.05     # Settle threshold for stationary pivot
    
    # ── Forward Startup Kick ──────────────────────────────────────────────────
    enable_forward_kick: bool = True
    forward_kick_speed: float = 0.45
    forward_kick_duration_s: float = 0.12

    # ── Smooth Stop & Acceleration Ramping ────────────────────────────────────
    enable_smooth_stop: bool = True     # Soft acceleration and deceleration ramping
    max_accel_mps2: float = 0.70        # Slew-rate acceleration limit (m/s^2)
    max_decel_mps2: float = 1.00        # Slew-rate deceleration limit (m/s^2)

    # ── Smart Target Recovery (Search in Last Moved Direction) ────────────────
    enable_search_recovery: bool = True # Turn in last known direction instead of stopping blind
    search_turn_speed: float = 0.14     # Gentle pivot speed while searching (m/s)
    search_duration_s: float = 2.0      # How long to sweep in the last direction before giving up (seconds)
    
    # ── Safety & Timeouts ─────────────────────────────────────────────────────
    target_timeout_s: float = 1.0      # Target lost timeout
    confidence_threshold: float = 0.40 # Minimum detection confidence
    
    # ── Vision Camera Geometry (Calibrated Pinhole Model) ─────────────────────
    camera_index: int = 0
    camera_width: int = 640
    camera_height: int = 480
    hfov_deg: float = 62.2             # Pi Camera Module V2 horizontal FOV
    assumed_person_height_m: float = 1.70 # Physical reference height
    model_backend: str = "tflite"
    model_path: str = "vision/models/detect.tflite"
    enable_preview: bool = True
    headless: bool = False
    mock: bool = False

    # ── Physical Dimensions (Synced with RobotController/Config.h) ────────────
    wheel_base_m: float = 0.134        # 13.4 cm track width
    wheel_radius_m: float = 0.0215     # 2.15 cm wheel radius

    def validate(self) -> None:
        if self.target_distance_m <= self.emergency_stop_dist_m:
            raise ValueError("target_distance_m must be greater than emergency_stop_dist_m.")
        if self.min_forward_speed > self.max_forward_speed:
            raise ValueError("min_forward_speed cannot exceed max_forward_speed.")


class Stage3ObstacleFollower:
    """
    Stage 3 Controller: Follows human using pure vision and steers around obstacles
    detected by the 3x ToF distance sensors via Artificial Potential Fields (APF).
    """

    def __init__(self, config: Optional[Stage3Config] = None) -> None:
        self.config = config or Stage3Config()
        self.config.validate()

        # Serial communication state
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

        # Heading PD & frame-synchronized damping
        self._is_steering: bool = False
        self._last_error: float = 0.0
        self._last_obs_timestamp: float = 0.0
        self._smoothed_d_err: float = 0.0

        # Smart target recovery tracking
        self._last_seen_direction: Optional[str] = None
        self._last_seen_time: float = 0.0

        # Subsystems
        v_config = VisionConfig(
            camera_index=self.config.camera_index,
            camera_width=self.config.camera_width,
            camera_height=self.config.camera_height,
            model_backend="mock" if self.config.mock else self.config.model_backend,
            model_path=self.config.model_path,
            confidence_threshold=self.config.confidence_threshold,
            distance_calibration_enabled=True,
            assumed_person_height_m=self.config.assumed_person_height_m,
            hfov_deg=self.config.hfov_deg,
            enable_preview=self.config.enable_preview and not self.config.headless,
            headless=self.config.headless,
        )
        self.vision = VisionService(v_config)
        self.sensors = DistanceSensorManager(use_mock=self.config.mock)

        # Telemetry & Vector State
        self._last_observation = TargetObservation.invalid()
        self._telemetry: Dict[str, Any] = {}

    def start(self) -> bool:
        """Start vision, sensors, serial streaming worker, and 20 Hz control loop."""
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
        """Initialize serial connection to ESP32."""
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
            print(f"[Stage3Follower] Could not open serial port {self.config.serial_port} ({e}). Running in mock mode.")
            self._ser = None

    def _serial_tx_worker(self) -> None:
        """Continuously streams velocity packets at 20 Hz with smooth acceleration/deceleration ramping."""
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

    def _estimate_human_distance(self, obs: TargetObservation) -> Tuple[float, str]:
        """
        Estimates human distance purely via computer vision (camera and bounding box geometry).
        Distance sensors are NEVER used for human estimation or tracking.
        """
        if not obs.target_valid:
            return self.config.target_distance_m, "DEFAULT"

        # Priority 1: Calibrated pinhole distance from TargetEstimator
        if obs.distance is not None and 0.20 < obs.distance < 12.0:
            return round(obs.distance, 2), "VISION"

        # Priority 2: Direct geometric computation from bounding box height
        if obs.bbox is not None and len(obs.bbox) == 4:
            x1, y1, x2, y2 = obs.bbox
            box_h = max(1.0, float(y2 - y1))
            half_fov_rad = math.radians(self.config.hfov_deg / 2.0)
            fx = (self.config.camera_width / 2.0) / math.tan(half_fov_rad)
            est_d = (fx * self.config.assumed_person_height_m) / box_h
            est_d = max(0.30, min(10.0, est_d))
            return round(est_d, 2), "VISION_BBOX"

        return self.config.target_distance_m, "DEFAULT"

    def calculate_apf(
        self,
        obs: TargetObservation,
        dist_m: float,
        left_dist: float,
        mid_dist: float,
        right_dist: float,
    ) -> Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]]:
        """
        Calculates Artificial Potential Field vectors in Robot Frame:
        +X = Forward, +Y = Left, -Y = Right.

        Human Target Attraction (F_att): Derived 100% from Vision.
        Obstacle Repulsion (F_rep): Derived 100% from 3x VL53L0X Distance Sensors.

        Returns:
            f_att: (att_x, att_y) attractive vector towards human
            f_rep: (rep_x, rep_y) repulsive vector away from obstacles
            f_net: (net_x, net_y) resultant navigation vector
        """
        # ── 1. Target Attraction towards Human (VISION ONLY) ──────────────────
        sweet_spot_max = self.config.target_distance_m + self.config.distance_deadband_m
        
        # Bearing angle of human in robot frame:
        # obs.bearing_deg is positive to the right (clockwise).
        # In Cartesian robot frame (+X forward, +Y left), bearing angle theta = -bearing_rad.
        human_theta = -math.radians(obs.bearing_deg)

        if dist_m > sweet_spot_max:
            excess_dist = dist_m - sweet_spot_max
            att_mag = self.config.kp_att * min(1.0, excess_dist / 0.80)
            f_att_x = att_mag * math.cos(human_theta)
            f_att_y = att_mag * math.sin(human_theta)
        else:
            # Inside sweet spot: no forward attraction force
            f_att_x = 0.0
            f_att_y = 0.0

        # ── 2. Obstacle Repulsion (SENSORS ONLY) ──────────────────────────────
        f_rep_x = 0.0
        f_rep_y = 0.0
        sensor_readings = [left_dist, mid_dist, right_dist]
        sensor_angles = self.config.sensor_angles_rad  # (+45 deg, 0 deg, -45 deg)

        for dist, angle in zip(sensor_readings, sensor_angles):
            if 0.03 < dist < self.config.obstacle_safe_dist_m:
                # Repulsion magnitude inversely proportional to distance (APF law)
                rep_mag = self.config.k_rep * (
                    (1.0 / dist) - (1.0 / self.config.obstacle_safe_dist_m)
                ) * (1.0 / (dist ** 2))
                rep_mag = min(self.config.max_rep_force, rep_mag)

                # Push directly away from obstacle angle (angle + pi)
                push_angle = angle + math.pi
                f_rep_x += rep_mag * math.cos(push_angle)
                f_rep_y += rep_mag * math.sin(push_angle)

        # ── 3. Net Vector Fusion ──────────────────────────────────────────────
        f_net_x = f_att_x + f_rep_x
        f_net_y = f_att_y + f_rep_y

        return (f_att_x, f_att_y), (f_rep_x, f_rep_y), (f_net_x, f_net_y)

    def step(
        self,
        obs: Optional[TargetObservation] = None,
        mock_tofs: Optional[Tuple[float, float, float]] = None,
    ) -> Tuple[float, float, str]:
        """
        Execute one control cycle:
        1. Target validation & Smart Recovery (Vision).
        2. Hard hardware safety shield check across all 3 ToF lasers (< 0.35m -> immediate stop).
        3. Human distance estimation strictly via VISION ONLY.
        4. APF vector calculation (F_att from vision + F_rep from distance sensors).
        5. Kinematics generation for smooth curve avoidance while pursuing human.
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

            # Search expired: smooth halt
            self._set_wheel_velocities(0.0, 0.0)
            self.state = "SEARCHING"
            return 0.0, 0.0, self.state

        # Remember last seen direction for recovery
        if abs(obs.horizontal_error) > 0.05:
            self._last_seen_direction = "RIGHT" if obs.horizontal_error > 0 else "LEFT"
        self._last_seen_time = now

        # Get ToF distance readings [left, middle, right] (for obstacles only)
        if mock_tofs is not None:
            left_d, mid_d, right_d = mock_tofs
        else:
            left_d, mid_d, right_d = self.sensors.get_distances()

        # ── 1. HARD HARDWARE SAFETY SHIELD ────────────────────────────────────
        valid_tofs = [d for d in (left_d, mid_d, right_d) if 0.03 < d < 2.0]
        min_tof = min(valid_tofs) if valid_tofs else None

        if min_tof is not None and min_tof < self.config.emergency_stop_dist_m:
            self._set_wheel_velocities(0.0, 0.0, immediate_stop=True)
            self.state = "SAFETY_STOP"
            return 0.0, 0.0, self.state

        # ── 2. ESTIMATE HUMAN DISTANCE (VISION ONLY) ──────────────────────────
        dist_m, dist_source = self._estimate_human_distance(obs)

        # ── 3. Calculate APF Vectors ──────────────────────────────────────────
        f_att, f_rep, f_net = self.calculate_apf(obs, dist_m, left_d, mid_d, right_d)
        f_net_x, f_net_y = f_net
        f_rep_x, f_rep_y = f_rep
        rep_mag = math.hypot(f_rep_x, f_rep_y)
        net_mag = math.hypot(f_net_x, f_net_y)

        # ── 4. Frame-Synchronized Heading Rate Calculation ────────────────────
        h_err = obs.horizontal_error
        if obs.timestamp > self._last_obs_timestamp:
            dt = max(0.02, obs.timestamp - self._last_obs_timestamp)
            raw_d_err = (h_err - self._last_error) / dt
            self._smoothed_d_err = 0.40 * raw_d_err + 0.60 * self._smoothed_d_err
            self._last_error = h_err
            self._last_obs_timestamp = obs.timestamp
        d_err = self._smoothed_d_err

        # ── 5. Kinematics from APF Vectors ────────────────────────────────────
        sweet_spot_max = self.config.target_distance_m + self.config.distance_deadband_m

        # Check if the robot needs to move forward
        is_forward_drive = (dist_m > sweet_spot_max) or (rep_mag > 0.08 and f_net_x > 0.05)

        if is_forward_drive:
            # Desired steering angle from APF net vector (in radians, + is Left, - is Right)
            desired_steer_rad = math.atan2(f_net_y, max(0.01, f_net_x))
            # Normalized steering error (-1.0 to +1.0)
            norm_steer = -desired_steer_rad / (math.pi / 2.0)  # Negate so + is Right (matching h_err)

            # Deadband check for straight cruise
            active_deadband = (
                self.config.deadband_forward_enter
                if self._is_steering
                else self.config.deadband_forward_exit
            )

            if abs(norm_steer) <= active_deadband:
                v_turn = 0.0
                self._is_steering = False
            else:
                self._is_steering = True
                excess = abs(norm_steer) - self.config.deadband_forward_enter
                raw_turn = self.config.kp_turn_forward * excess

                # Rate damping when closing in on target center
                is_closing_in = (norm_steer > 0 and d_err < -0.15) or (norm_steer < 0 and d_err > 0.15)
                if is_closing_in:
                    raw_turn *= max(0.40, 1.0 - 0.25 * abs(d_err))

                turn_speed = max(0.03, min(self.config.max_turn_forward_speed, raw_turn))
                v_turn = turn_speed if norm_steer > 0 else -turn_speed

            # Linear forward speed scaled with net vector authority
            v_linear = self.config.min_forward_speed + (
                self.config.max_forward_speed - self.config.min_forward_speed
            ) * min(1.0, max(0.0, net_mag))

            # Turn-factor slowdown for sharp turns
            if abs(desired_steer_rad) > 0.35:
                v_linear *= max(0.35, math.cos(desired_steer_rad))

            # Determine human-readable state
            if rep_mag > 0.08:
                # Active obstacle evasion
                self.state = "AVOIDING_OBSTACLE_LEFT" if f_rep_y > 0 else "AVOIDING_OBSTACLE_RIGHT"
            elif abs(v_turn) > 0.02:
                self.state = "FOLLOW_CURVE_RIGHT" if v_turn > 0 else "FOLLOW_CURVE_LEFT"
            else:
                self.state = "FOLLOW_STRAIGHT"

        else:
            # Inside sweet spot: stationary holding with Stage 1 precision pivot
            v_linear = 0.0
            active_deadband = (
                self.config.deadband_turn_enter
                if self._is_steering
                else self.config.deadband_turn_exit
            )

            if abs(h_err) <= active_deadband:
                v_turn = 0.0
                self._is_steering = False
                self.state = "HOLD_SWEET_SPOT"
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
                self.state = "PIVOT_RIGHT" if v_turn > 0 else "PIVOT_LEFT"

        # ── 6. Kinematics Fusion ──────────────────────────────────────────────
        vl = v_linear + v_turn
        vr = v_linear - v_turn

        max_limit = self.config.max_forward_speed + 0.05
        vl = max(-max_limit, min(max_limit, vl))
        vr = max(-max_limit, min(max_limit, vr))

        self._set_wheel_velocities(vl, vr, is_forward_accel=is_forward_drive)
        return vl, vr, self.state

    def _control_loop(self) -> None:
        """20 Hz control loop executing APF obstacle avoidance and human following."""
        interval = 1.0 / self.config.tx_rate_hz

        while self._running:
            t0 = time.monotonic()
            vl, vr, state = self.step()

            # Record telemetry
            obs = self._last_observation
            v_telem = self.vision.get_telemetry()
            left_d, mid_d, right_d = self.sensors.get_distances()
            dist_m, source = self._estimate_human_distance(obs)
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
