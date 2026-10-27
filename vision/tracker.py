"""
vision/tracker.py
-----------------
Single-person tracking and temporal jitter filtering for Raspberry Pi 4.

Key Features:
1. Target Selection: Because only one target person is expected, selects the best candidate
   based on proximity to last known position and detection confidence.
2. Temporal Jitter Filtering: Uses Exponential Moving Average (EMA) smoothing to eliminate
   bounding-box jitter without introducing heavy computational overhead.
3. Coasting & Timeout: Bridges temporary occlusion and detector drops without false alarms,
   then cleanly declares the target invalid after `track_lost_timeout_s`.
"""

import math
import time
from typing import List, Optional, Tuple

from .config import VisionConfig
from .detector import Detection
from .target_data import TargetObservation


class SinglePersonTracker:
    """
    Maintains a continuous single-person target state across successive camera frames.
    """

    def __init__(self, config: VisionConfig) -> None:
        self.config = config
        self.alpha = config.ema_alpha
        self.timeout = config.track_lost_timeout_s
        self.max_dist_norm = config.max_tracking_distance_norm
        self.frame_width = config.camera_width
        self.frame_height = config.camera_height

        # State variables
        self._last_bbox: Optional[Tuple[float, float, float, float]] = None
        self._last_confidence: float = 0.0
        self._last_detection_time: float = 0.0
        self._is_tracking: bool = False

    def update(
        self,
        detections: List[Detection],
        frame_width: Optional[int] = None,
        frame_height: Optional[int] = None,
        timestamp: Optional[float] = None,
    ) -> TargetObservation:
        """
        Process new detections and return the updated TargetObservation.
        
        Args:
            detections: List of Detection objects from detector.
            frame_width: Optional frame width override (defaults to config).
            frame_height: Optional frame height override (defaults to config).
            timestamp: Monotonic timestamp of the observation.
        """
        now = timestamp if timestamp is not None else time.monotonic()
        fw = frame_width if frame_width is not None else self.frame_width
        fh = frame_height if frame_height is not None else self.frame_height

        # Select the single best detection
        chosen = self._select_best_target(detections, fw, fh)

        if chosen is not None:
            # Active detection found
            x1, y1, x2, y2 = chosen.bbox

            if self._last_bbox is None or not self._is_tracking:
                # Initialize new track without smoothing
                smoothed_bbox = (x1, y1, x2, y2)
                smoothed_conf = chosen.confidence
            else:
                # Apply Exponential Moving Average (EMA) smoothing
                lx1, ly1, lx2, ly2 = self._last_bbox
                smoothed_bbox = (
                    self.alpha * x1 + (1.0 - self.alpha) * lx1,
                    self.alpha * y1 + (1.0 - self.alpha) * ly1,
                    self.alpha * x2 + (1.0 - self.alpha) * lx2,
                    self.alpha * y2 + (1.0 - self.alpha) * ly2,
                )
                smoothed_conf = self.alpha * chosen.confidence + (1.0 - self.alpha) * self._last_confidence

            self._last_bbox = smoothed_bbox
            self._last_confidence = smoothed_conf
            self._last_detection_time = now
            self._is_tracking = True

            return self._build_observation(
                target_valid=True,
                bbox=smoothed_bbox,
                confidence=smoothed_conf,
                fw=fw,
                fh=fh,
                timestamp=now,
            )

        # No detection in current frame: check for temporary occlusion / coasting
        if self._is_tracking and self._last_bbox is not None:
            elapsed = now - self._last_detection_time

            if elapsed <= self.timeout:
                # Coasting: brief occlusion or missed detection
                # Decay confidence linearly over the timeout window
                decayed_conf = max(0.05, self._last_confidence * (1.0 - (elapsed / self.timeout)))

                return self._build_observation(
                    target_valid=True,
                    bbox=self._last_bbox,
                    confidence=decayed_conf,
                    fw=fw,
                    fh=fh,
                    timestamp=now,
                )
            else:
                # Timeout expired: target is lost
                self.reset()
                return TargetObservation.invalid(timestamp=now)

        # No target ever tracked
        return TargetObservation.invalid(timestamp=now)

    def _select_best_target(
        self,
        detections: List[Detection],
        fw: int,
        fh: int,
    ) -> Optional[Detection]:
        """
        Picks the single most appropriate human detection.
        - If previously tracking: prefers the detection closest to last known center.
        - If not tracking: prefers highest confidence and larger bounding box.
        """
        if not detections:
            return None

        if len(detections) == 1:
            return detections[0]

        if self._is_tracking and self._last_bbox is not None:
            # Match closest detection to last known center
            lx1, ly1, lx2, ly2 = self._last_bbox
            last_cx = (lx1 + lx2) / 2.0
            last_cy = (ly1 + ly2) / 2.0

            best_det = None
            min_dist = float("inf")

            for det in detections:
                cx = (det.bbox[0] + det.bbox[2]) / 2.0
                cy = (det.bbox[1] + det.bbox[3]) / 2.0
                dist_norm = math.hypot((cx - last_cx) / fw, (cy - last_cy) / fh)

                if dist_norm < min_dist and dist_norm <= self.max_dist_norm:
                    min_dist = dist_norm
                    best_det = det

            if best_det is not None:
                return best_det

        # Fallback: rank by confidence * bounding box area
        def score(det: Detection) -> float:
            w = max(1.0, det.bbox[2] - det.bbox[0])
            h = max(1.0, det.bbox[3] - det.bbox[1])
            area_norm = (w * h) / (fw * fh)
            return det.confidence * (1.0 + 0.5 * area_norm)

        return max(detections, key=score)

    def _build_observation(
        self,
        target_valid: bool,
        bbox: Tuple[float, float, float, float],
        confidence: float,
        fw: int,
        fh: int,
        timestamp: float,
    ) -> TargetObservation:
        """Construct TargetObservation with center coordinates and horizontal error."""
        cx = (bbox[0] + bbox[2]) / 2.0
        cy = (bbox[1] + bbox[3]) / 2.0

        center_x_frame = fw / 2.0
        h_error_px = cx - center_x_frame
        # Normalized error from -1.0 (far left) to +1.0 (far right)
        h_error_norm = max(-1.0, min(1.0, h_error_px / center_x_frame))

        return TargetObservation(
            target_valid=target_valid,
            confidence=confidence,
            bbox=bbox,
            center_x=cx,
            center_y=cy,
            horizontal_error=h_error_norm,
            horizontal_error_px=h_error_px,
            bearing=0.0,  # Enriched by TargetEstimator
            bearing_deg=0.0,
            distance=None,  # Enriched by TargetEstimator
            timestamp=timestamp,
        )

    def is_tracking(self) -> bool:
        """Return whether an active target is currently tracked."""
        return self._is_tracking

    def reset(self) -> None:
        """Clear tracking history and reset tracker state."""
        self._last_bbox = None
        self._last_confidence = 0.0
        self._last_detection_time = 0.0
        self._is_tracking = False

