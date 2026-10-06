from typing import Any

import numpy as np
from numpy.typing import NDArray

from config import Config
from detector import PersonDetector
from target_selector import TargetSelector
from tracker import PersonTracker, Track, center


def person_record(track: Track) -> dict[str, Any]:
    x1, y1, x2, y2 = track.bbox
    cx, cy = center(track.bbox)
    return {
        "id": track.id,
        "confidence": round(track.confidence, 4),
        "bbox": [round(v, 1) for v in track.bbox],
        "center": [round(cx, 1), round(cy, 1)],
        "width": round(x2 - x1, 1),
        "height": round(y2 - y1, 1),
    }


class VisionSystem:
    def __init__(self, config: Config) -> None:
        config.validate()
        self.config = config
        self.detector = PersonDetector(config)
        self.tracker = PersonTracker(config.track_lost_seconds)
        self.selector = TargetSelector(
            config.target_hold_seconds, config.initial_target_min_hits
        )

    def process_frame(
        self, frame: NDArray[np.uint8], now: float
    ) -> dict[str, Any]:
        height, width = frame.shape[:2]
        if width < 1 or height < 1:
            raise ValueError("Empty camera frame.")

        detections = self.detector.detect(frame)
        tracks = self.tracker.update(detections, now)
        target = self.selector.select(tracks, width, height, now)

        result: dict[str, Any] = {
            "target_detected": target is not None,
            "target_id": (
                target.id if target is not None else self.selector.target_id
            ),
            "people": [person_record(track) for track in tracks],
            "frame_size": [width, height],
            "timestamp_monotonic": round(now, 4),
        }

        if target is None:
            # A non-null target_id means the ID is temporarily reserved;
            # it does NOT mean there is a current target observation.
            return result

        person = person_record(target)
        cx, cy = center(target.bbox)
        box_height = target.bbox[3] - target.bbox[1]
        normalized_height = box_height / height
        horizontal_error = 2.0 * cx / width - 1.0
        vertical_error = 2.0 * cy / height - 1.0

        reference = self.selector.reference_height
        relative_distance_ratio = (
            reference / box_height
            if reference is not None and box_height > 0
            else None
        )

        # Only available when BOTH values were deliberately calibrated.
        # An assumed average human height is not a safe range sensor.
        distance_m = None
        if (
            self.config.focal_length_px is not None
            and self.config.assumed_person_height_m is not None
            and box_height > 0
        ):
            distance_m = (
                self.config.focal_length_px
                * self.config.assumed_person_height_m
                / box_height
            )

        result.update(
            {
                "confidence": person["confidence"],
                "bbox": person["bbox"],
                "center": person["center"],
                "target_center": person["center"],
                "horizontal_error": round(horizontal_error, 4),
                "vertical_error": round(vertical_error, 4),
                "normalized_width": round(
                    (target.bbox[2] - target.bbox[0]) / width, 4
                ),
                "normalized_height": round(normalized_height, 4),
                "normalized_bbox_width": round(
                    (target.bbox[2] - target.bbox[0]) / width, 4
                ),
                "normalized_bbox_height": round(normalized_height, 4),
                "relative_distance_ratio": (
                    round(relative_distance_ratio, 4)
                    if relative_distance_ratio is not None
                    else None
                ),
                "distance_m": (
                    round(distance_m, 3) if distance_m is not None else None
                ),
            }
        )
        return result
