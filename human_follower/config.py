"""
human_follower/config.py
------------------------
Configuration parameters for Stage 1 Turret Tracking (Stationary Pivoting).
Tuned with PD damping and hysteresis to eliminate overshoot and hunting oscillations.
"""

from dataclasses import dataclass


@dataclass
class FollowerConfig:
    # ── UART & Serial Communication ───────────────────────────────────────────
    serial_port: str = "/dev/serial0"
    serial_baud: int = 115200
    tx_rate_hz: float = 20.0           # 20 Hz transmission to satisfy ESP32 500ms watchdog
    watchdog_timeout_s: float = 0.45   # Serial command timeout
    
    # ── Stage 1 Turret Turning Controls ───────────────────────────────────────
    kp_turn: float = 0.30              # Proportional gain: determines base turning speed
    kd_turn: float = 0.08              # Derivative damping gain: acts as virtual brake against overshoot
    max_accel_mps2: float = 0.85       # Slew-rate acceleration limit (m/s^2): eliminates motor chatter/jerks
    
    # ── Hysteresis Deadband (Eliminates Hunting / Chattering) ──────────────────
    # Outside deadband_exit (e.g. |err| > 0.10): robot starts pivoting
    # Inside deadband_enter (e.g. |err| <= 0.05): robot comes to complete stop
    deadband_exit: float = 0.10        # Trigger threshold to start turning (10% of frame center)
    deadband_enter: float = 0.05       # Settle threshold to stop turning (5% of frame center)
    deadband: float = 0.08             # Backward compatibility default
    
    # ── Speed Boundaries for Gentle Pivoting ──────────────────────────────────
    min_turn_speed: float = 0.10       # Gentle minimum wheel speed (m/s) for small corrections
    max_turn_speed: float = 0.24       # Capped maximum wheel speed (m/s) to prevent overshooting
    
    # ── Startup Kick for Turns ────────────────────────────────────────────────
    # Disabled for in-place pivots: differential pivots have low inertia and high kick causes overshooting
    enable_turn_kick: bool = False
    turn_kick_speed: float = 0.35
    turn_kick_duration_s: float = 0.08

    # ── Smart Target Recovery (Search in Last Moved Direction) ────────────────
    enable_search_recovery: bool = True # Turn in last known direction instead of stopping blind
    search_turn_speed: float = 0.14     # Gentle pivot speed while searching (m/s)
    search_duration_s: float = 2.0      # How long to sweep in the last direction before giving up (seconds)
    
    # ── Safety & Timeouts ─────────────────────────────────────────────────────
    target_timeout_s: float = 1.0      # If target is lost for > 1.0s, begin search recovery or stop motors
    confidence_threshold: float = 0.40 # Minimum detection confidence
    
    # ── Robot Hardware Geometry ───────────────────────────────────────────────
    wheel_base_m: float = 0.134        # Distance between left and right wheel centers
    wheel_radius_m: float = 0.0215     # Radius of wheels in meters
    
    # ── Vision & Camera ───────────────────────────────────────────────────────
    camera_index: int = 0
    camera_width: int = 640
    camera_height: int = 480
    model_backend: str = "tflite"      # "tflite", "opencv_dnn", or "mock"
    model_path: str = "vision/models/detect.tflite"
    enable_preview: bool = True
    headless: bool = False
    mock: bool = False

    def __post_init__(self) -> None:
        if self.deadband_enter > self.deadband_exit:
            self.deadband_enter = self.deadband_exit * 0.6

    def validate(self) -> None:
        if self.tx_rate_hz <= 0:
            raise ValueError("tx_rate_hz must be positive.")
        if self.deadband_exit <= 0.0 or self.deadband_exit > 0.5:
            raise ValueError("deadband_exit must be in (0.0, 0.5].")
        if self.deadband_enter < 0.0 or self.deadband_enter >= self.deadband_exit:
            raise ValueError("deadband_enter must be in [0.0, deadband_exit).")
        if self.min_turn_speed > self.max_turn_speed:
            raise ValueError("min_turn_speed cannot exceed max_turn_speed.")
        if self.kp_turn <= 0:
            raise ValueError("kp_turn must be positive.")
