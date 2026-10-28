"""
human_follower/config.py
------------------------
Configuration parameters for Stage 1 Turret Tracking (Stationary Pivoting).
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
    kp_turn: float = 0.45              # Proportional gain: wheel_speed = Kp * horizontal_error
    deadband: float = 0.08             # Normalized center deadband (|err| <= 0.08 -> motors 0)
    min_turn_speed: float = 0.18       # Minimum wheel speed (m/s) to overcome DC motor gearbox stiction
    max_turn_speed: float = 0.40       # Clamped maximum wheel speed (m/s) for smooth turning
    
    # ── Startup Kick for Turns ────────────────────────────────────────────────
    enable_turn_kick: bool = True      # Quick breakaway kick when starting a pivot from stop
    turn_kick_speed: float = 0.55      # Breakaway speed (m/s)
    turn_kick_duration_s: float = 0.10 # 100 ms burst duration
    
    # ── Safety & Timeouts ─────────────────────────────────────────────────────
    target_timeout_s: float = 1.0      # If target is lost for > 1.0s, immediately stop motors
    confidence_threshold: float = 0.40 # Minimum detection confidence
    
    # ── Robot Hardware Geometry ───────────────────────────────────────────────
    wheel_base_m: float = 0.13         # Distance between left and right wheel centers
    wheel_radius_m: float = 0.02       # Radius of wheels in meters
    
    # ── Vision & Camera ───────────────────────────────────────────────────────
    camera_index: int = 0
    camera_width: int = 640
    camera_height: int = 480
    model_backend: str = "tflite"      # "tflite", "opencv_dnn", or "mock"
    model_path: str = "vision/models/detect.tflite"
    enable_preview: bool = True
    headless: bool = False
    mock: bool = False

    def validate(self) -> None:
        if self.tx_rate_hz <= 0:
            raise ValueError("tx_rate_hz must be positive.")
        if self.deadband < 0.0 or self.deadband > 0.5:
            raise ValueError("deadband must be in [0.0, 0.5].")
        if self.min_turn_speed > self.max_turn_speed:
            raise ValueError("min_turn_speed cannot exceed max_turn_speed.")
        if self.kp_turn <= 0:
            raise ValueError("kp_turn must be positive.")

