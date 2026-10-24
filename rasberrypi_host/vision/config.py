from dataclasses import dataclass
from pathlib import Path


def _get_default_model_path() -> Path:
    models_dir = Path(__file__).resolve().parent / "models"
    tflite = models_dir / "detect.tflite"
    if tflite.is_file():
        return tflite
    yolo_pose = models_dir / "yolo26n-pose.pt"
    if yolo_pose.is_file():
        return yolo_pose
    return models_dir / "yolo26n.pt"


@dataclass(frozen=True)
class Config:
    model_path: Path | str = _get_default_model_path()

    camera_index: int = 0
    camera_width: int = 640
    camera_height: int = 480

    confidence_threshold: float = 0.35
    inference_threads: int = 2
    device: str | None = None
    imgsz: int = 640

    # Tracking and target retention are measured in seconds, not frames:
    # inference FPS on a Pi can vary substantially.

    track_lost_seconds: float = 2.0
    target_hold_seconds: float = 2.0
    initial_target_min_hits: int = 2

    # For visualization only. JSON is emitted in either mode.
    debug_view: bool = True

    # Optional calibrated metric estimate. Leave unset by default.
    focal_length_px: float | None = None
    assumed_person_height_m: float | None = None

    # ── Pose & Gait Stance Detection ──────────────────────────────────
    # Minimum keypoint confidence to consider a joint visible/usable.
    keypoint_confidence_threshold: float = 0.35

    # Rolling window duration (seconds) for stride variance and cadence.
    # ~10-14 frames at typical Pi inference FPS.
    gait_window_seconds: float = 0.7

    # Normalized stride variance below this → person classified as
    # "standing" (feet planted, no stepping).
    gait_stop_variance_threshold: float = 0.0020

    # Normalized stride variance above this → person classified as
    # "walking" (active gait cycle).
    gait_walk_variance_threshold: float = 0.0055

    # ── Vision-Only Car Stop Detection ────────────────────────────────
    # Fractional depth-proxy expansion rate per second above which the
    # camera is considered to be closing in on the target.
    looming_stop_threshold: float = 0.08

    # Time-to-collision (tau) in seconds; below this → emergency brake
    # regardless of gait state.
    critical_ttc_seconds: float = 1.2

    # Debounce: once a stop is commanded, hold it for at least this many
    # seconds to avoid brake flicker from noisy keypoints.
    stop_hold_seconds: float = 0.5

    # Exponential smoothing alpha for keypoint jitter filtering.
    # 0.0 = full smoothing (lag), 1.0 = no smoothing (raw detections).
    keypoint_smoothing_alpha: float = 0.55

    def validate(self) -> None:
        if self.camera_width < 1 or self.camera_height < 1:
            raise ValueError("Camera dimensions must be positive.")
        if not 0.0 < self.confidence_threshold < 1.0:
            raise ValueError("confidence_threshold must be between 0 and 1.")
        if self.inference_threads < 1:
            raise ValueError("inference_threads must be positive.")
        if self.imgsz < 1:
            raise ValueError("imgsz must be positive.")
        if self.track_lost_seconds <= 0 or self.target_hold_seconds <= 0:
            raise ValueError("Tracking timeouts must be positive.")
        if self.initial_target_min_hits < 1:
            raise ValueError("initial_target_min_hits must be positive.")
        if (self.focal_length_px is None) != (
            self.assumed_person_height_m is None):
            raise ValueError(
                "Set both focal_length_px and assumed_person_height_m, "
                "or neither."
            )
        if not 0.0 < self.keypoint_confidence_threshold < 1.0:
            raise ValueError(
                "keypoint_confidence_threshold must be between 0 and 1."
            )
        if self.gait_window_seconds <= 0:
            raise ValueError("gait_window_seconds must be positive.")
        if self.gait_stop_variance_threshold <= 0:
            raise ValueError(
                "gait_stop_variance_threshold must be positive."
            )
        if self.gait_walk_variance_threshold <= 0:
            raise ValueError(
                "gait_walk_variance_threshold must be positive."
            )
        if self.gait_stop_variance_threshold >= self.gait_walk_variance_threshold:
            raise ValueError(
                "gait_stop_variance_threshold must be less than "
                "gait_walk_variance_threshold."
            )
        if self.looming_stop_threshold <= 0:
            raise ValueError("looming_stop_threshold must be positive.")
        if self.critical_ttc_seconds <= 0:
            raise ValueError("critical_ttc_seconds must be positive.")
        if self.stop_hold_seconds < 0:
            raise ValueError("stop_hold_seconds must be non-negative.")
        if not 0.0 < self.keypoint_smoothing_alpha <= 1.0:
            raise ValueError(
                "keypoint_smoothing_alpha must be in (0.0, 1.0]."
            )
