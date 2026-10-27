"""
vision/config.py
----------------
Configuration dataclass and default parameters for human detection and tracking.
Tuned specifically for CPU-constrained Raspberry Pi 4 (ARM Cortex-A72).
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional


@dataclass
class VisionConfig:
    # ── Camera Capture Settings ───────────────────────────────────────────────
    camera_index: int = 0
    camera_width: int = 640
    camera_height: int = 480
    camera_fps: int = 30
    camera_backend: str = "opencv"  # "opencv", "picamera2", "mock"

    # ── Model & Inference Backend ─────────────────────────────────────────────
    # Backend choices: "tflite" (recommended for Pi 4), "opencv_dnn", "mock"
    model_backend: str = "tflite"
    
    # Path to the pretrained model (relative or absolute).
    # Defaults to detect.tflite if present in vision/models or rasberrypi_host/vision/models
    model_path: str = "vision/models/detect.tflite"
    
    # Model input tensor resolution (SSD-MobileNet is 300x300; YOLO is 320x320 or 640x640)
    input_width: int = 300
    input_height: int = 300
    
    # Confidence threshold to accept a person detection (0.0 to 1.0)
    confidence_threshold: float = 0.40
    
    # Number of CPU threads for inference (Pi 4 has 4 cores; 2-4 recommended)
    num_threads: int = 4

    # ── Tracking & Temporal Smoothing ─────────────────────────────────────────
    # Exponential moving average smoothing factor alpha in (0.0, 1.0].
    # Higher = more responsive (less lag), lower = smoother (less jitter).
    ema_alpha: float = 0.65
    
    # Seconds after which a lost target is declared invalid (target_valid = False).
    track_lost_timeout_s: float = 1.5
    
    # Maximum normalized distance (0.0 to 1.0) between consecutive frames to consider
    # a detection as the same target person.
    max_tracking_distance_norm: float = 0.40

    # ── Geometry & Calibration Settings ───────────────────────────────────────
    # Camera Horizontal Field of View (degrees).
    # Default is 62.2° for Raspberry Pi Camera Module V2 (53.5° for V1, ~65-75° for webcams).
    hfov_deg: float = 62.2
    
    # Calibrated focal length in pixels (fx). If None, calculated from hfov_deg and camera_width.
    focal_length_px: Optional[float] = None

    # Enable metric distance estimation proxy.
    # Note: Strictly requires calibration and ground truth reference.
    # Default is False to avoid inaccurate claims without physical calibration.
    distance_calibration_enabled: bool = False
    
    # Assumed physical height of a person in meters (used only if distance_calibration_enabled=True)
    assumed_person_height_m: float = 1.70

    # ── Display & Headless Mode ───────────────────────────────────────────────
    enable_preview: bool = True
    headless: bool = False

    def __post_init__(self) -> None:
        if self.focal_length_px is None and self.camera_width > 0 and self.hfov_deg > 0:
            import math
            half_fov_rad = math.radians(self.hfov_deg / 2.0)
            self.focal_length_px = (self.camera_width / 2.0) / math.tan(half_fov_rad)

    def validate(self) -> None:
        """Validate configuration bounds."""
        if self.camera_width <= 0 or self.camera_height <= 0:
            raise ValueError("Camera dimensions must be positive integers.")
        if not (0.0 < self.confidence_threshold <= 1.0):
            raise ValueError("confidence_threshold must be in the range (0.0, 1.0].")
        if not (0.0 < self.ema_alpha <= 1.0):
            raise ValueError("ema_alpha must be in the range (0.0, 1.0].")
        if self.track_lost_timeout_s <= 0:
            raise ValueError("track_lost_timeout_s must be positive.")
        if self.num_threads < 1:
            raise ValueError("num_threads must be at least 1.")

