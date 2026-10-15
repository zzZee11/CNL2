from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from config import Config

from ultralytics import YOLO
import torch

@dataclass(frozen=True)
class PersonDetection:
    bbox: tuple[float, float, float, float]
    confidence: float
    keypoints: NDArray[np.float32] | None = None  # Shape: (17, 3) -> [x, y, conf]


class PersonDetector:
    """Person detector using Ultralytics YOLO26 (detection or pose model)."""

    # In COCO dataset, class 0 is 'person'.
    PERSON_CLASS_ID = 0

    def __init__(self, config: Config) -> None:
        
        config.validate()
        self.threshold = config.confidence_threshold
        self.device = getattr(config, "device", None)
        self.imgsz = getattr(config, "imgsz", 640)

        # Configure CPU inference threads if specified
        if config.inference_threads > 0:
            torch.set_num_threads(config.inference_threads)

        model_target = config.model_path
        if isinstance(model_target, (str, Path)):
            target_path = Path(model_target)
            if target_path.is_file():
                model_str = str(target_path)
            else:
                fallback_path = (
                    Path(__file__).resolve().parent / "models" / target_path.name
                )
                if fallback_path.is_file():
                    model_str = str(fallback_path)
                else:
                    model_str = str(model_target)
        else:
            model_str = str(model_target)

        self.model = YOLO(model_str)

        # Detect whether the loaded model supports pose keypoints.
        # Ultralytics pose models have task == "pose".
        self._has_pose: bool = getattr(self.model, "task", None) == "pose"

    def detect(self, frame: NDArray[np.uint8]) -> list[PersonDetection]:
        frame_height, frame_width = frame.shape[:2]
        if frame_width < 1 or frame_height < 1:
            return []

        kwargs: dict[str, object] = {
            "conf": self.threshold,
            "classes": [self.PERSON_CLASS_ID],
            "verbose": False,
            "imgsz": self.imgsz,
        }
        if self.device is not None:
            kwargs["device"] = self.device

        results = self.model.predict(frame, **kwargs)
        if not results:
            return []

        result = results[0]
        boxes = result.boxes
        if boxes is None or len(boxes) == 0:
            return []

        # Extract keypoints if the model provides them.
        raw_keypoints: NDArray[np.float32] | None = None
        if self._has_pose and result.keypoints is not None:
            kp_data = result.keypoints.data
            if kp_data is not None and len(kp_data) > 0:
                raw_keypoints = kp_data.cpu().numpy().astype(np.float32)

        people: list[PersonDetection] = []
        xyxy_coords = boxes.xyxy.cpu().numpy()
        scores = boxes.conf.cpu().numpy()

        for idx, (box, score) in enumerate(zip(xyxy_coords, scores)):
            score_val = float(score)
            if not np.isfinite(score_val) or score_val < self.threshold:
                continue

            xmin, ymin, xmax, ymax = map(float, box)
            if not all(np.isfinite(v) for v in (xmin, ymin, xmax, ymax)):
                continue

            x1 = max(0.0, min(float(frame_width), xmin))
            y1 = max(0.0, min(float(frame_height), ymin))
            x2 = max(0.0, min(float(frame_width), xmax))
            y2 = max(0.0, min(float(frame_height), ymax))

            if x2 > x1 and y2 > y1:
                kp: NDArray[np.float32] | None = None
                if raw_keypoints is not None and idx < len(raw_keypoints):
                    kp = raw_keypoints[idx]  # shape (17, 3)
                people.append(PersonDetection((x1, y1, x2, y2), score_val, kp))

        return people
