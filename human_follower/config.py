"""
human_follower/config.py
------------------------
Configuration parameters for Stage 1 Turret Tracking (Stationary Pivoting).
Configured for clean, steady pivot speed (0.20 m/s) with zero startup kicks.
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
    # Steady turning speed: when turning, wheels spin at exactly this velocity
    turn_speed: float = 0.20           # Wheel speed (m/s) during pivot (default: 0.20 m/s)
    
    # Backward compatibility aliases
    min_turn_speed: float = 0.20
    max_turn_speed: float = 0.20
    kp_turn: float = 0.35
    kd_turn: float = 0.12
    enable_turn_kick: bool = False
    
    # ── Hysteresis Deadband (Eliminates Hunting / Chattering) ──────────────────
    # Outside deadband_exit (e.g. |err| > 0.10): robot starts pivoting
    # Inside deadband_enter (e.g. |err| <= 0.05): robot comes to complete stop
    deadband_exit: float = 0.10        # Trigger threshold to start turning (10% of frame center)
    deadband_enter: float = 0.05       # Settle threshold to stop turning (5% of frame center)
    deadband: float = 0.08             # Default single deadband fallback
    
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

    def __post_init__(self) -> None:
        if self.deadband_enter > self.deadband_exit:
            self.deadband_enter = self.deadband_exit * 0.6

    def validate(self) -> None:
        if self.tx_rate_hz <= 0:
            raise ValueError("tx_rate_hz must be positive.")
        if self.turn_speed <= 0.0:
            raise ValueError("turn_speed must be positive.")
        if self.deadband_exit <= 0.0 or self.deadband_exit > 0.5:
            raise ValueError("deadband_exit must be in (0.0, 0.5].")
        if self.deadband_enter < 0.0 or self.deadband_enter >= self.deadband_exit:
            raise ValueError("deadband_enter must be in [0.0, deadband_exit).")
