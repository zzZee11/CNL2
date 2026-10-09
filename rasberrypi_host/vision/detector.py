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


class PersonDetector:
    """Person detector using Ultralytics YOLO26."""

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

        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            return []

        people: list[PersonDetection] = []
        xyxy_coords = boxes.xyxy.cpu().numpy()
        scores = boxes.conf.cpu().numpy()

        for box, score in zip(xyxy_coords, scores):
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
                people.append(PersonDetection((x1, y1, x2, y2), score_val))

        return people
