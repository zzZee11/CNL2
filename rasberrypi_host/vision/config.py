from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Config:
    model_path: Path | str = (
        Path(__file__).resolve().parent / "models" / "yolo26n.pt"
    )

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
    debug_view: bool = False

    # Optional calibrated metric estimate. Leave unset by default.
    focal_length_px: float | None = None
    assumed_person_height_m: float | None = None

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
            self.assumed_person_height_m is None
        ):
            raise ValueError(
                "Set both focal_length_px and assumed_person_height_m, "
                "or neither."
            )
