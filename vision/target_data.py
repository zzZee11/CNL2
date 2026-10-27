"""
vision/target_data.py
---------------------
Thread-safe target observation data structure.
Exposes the single-person target state for external path-planning integration.
"""

from dataclasses import dataclass, asdict
import json
import time
from typing import Optional, Tuple, Dict, Any


@dataclass
class TargetObservation:
    """
    State container for the single detected/tracked human target.
    
    Attributes:
        target_valid (bool): True if a target is actively tracked and within timeout.
        confidence (float): Detection confidence score in [0.0, 1.0].
        bbox (tuple): Bounding box in pixel coordinates (x1, y1, x2, y2).
        center_x (float): Bounding box center X in pixels.
        center_y (float): Bounding box center Y in pixels.
        horizontal_error (float): Normalized horizontal offset from frame center:
                                  -1.0 (extreme left) to +1.0 (extreme right). 0.0 is dead-center.
        horizontal_error_px (float): Raw pixel offset from frame center (center_x - frame_width / 2).
        bearing (float): Estimated horizontal bearing angle in radians relative to optical axis.
                         Positive = target is to the right; Negative = target is to the left.
        bearing_deg (float): Bearing angle in degrees.
        distance (Optional[float]): Calibrated distance in meters, or None if uncalibrated.
        timestamp (float): Monotonic timestamp of the latest observation update.
    """
    target_valid: bool = False
    confidence: float = 0.0
    bbox: Optional[Tuple[float, float, float, float]] = None
    center_x: float = 0.0
    center_y: float = 0.0
    horizontal_error: float = 0.0
    horizontal_error_px: float = 0.0
    bearing: float = 0.0
    bearing_deg: float = 0.0
    distance: Optional[float] = None
    timestamp: float = 0.0

    @classmethod
    def invalid(cls, timestamp: Optional[float] = None) -> "TargetObservation":
        """Factory for an empty / invalid observation."""
        return cls(
            target_valid=False,
            confidence=0.0,
            bbox=None,
            center_x=0.0,
            center_y=0.0,
            horizontal_error=0.0,
            horizontal_error_px=0.0,
            bearing=0.0,
            bearing_deg=0.0,
            distance=None,
            timestamp=timestamp if timestamp is not None else time.monotonic(),
        )

    def is_valid(self) -> bool:
        """Convenience method checking target validity."""
        return self.target_valid

    def to_dict(self) -> Dict[str, Any]:
        """Convert observation to a dictionary."""
        data = asdict(self)
        if self.bbox is not None:
            data["bbox"] = [round(c, 2) for c in self.bbox]
        data["confidence"] = round(self.confidence, 3)
        data["center_x"] = round(self.center_x, 1)
        data["center_y"] = round(self.center_y, 1)
        data["horizontal_error"] = round(self.horizontal_error, 3)
        data["horizontal_error_px"] = round(self.horizontal_error_px, 1)
        data["bearing"] = round(self.bearing, 4)
        data["bearing_deg"] = round(self.bearing_deg, 2)
        if self.distance is not None:
            data["distance"] = round(self.distance, 2)
        data["timestamp"] = round(self.timestamp, 4)
        return data

    def to_json(self) -> str:
        """Convert observation to a JSON string."""
        return json.dumps(self.to_dict())

    def summary(self) -> str:
        """One-line human-readable summary for terminal logging."""
        if not self.target_valid:
            return "[Target: LOST / INVALID]"
        dist_str = f"{self.distance:.2f}m" if self.distance is not None else "uncalibrated"
        return (
            f"[Target: VALID | Conf: {self.confidence:.0%} | "
            f"Center: ({self.center_x:.0f}, {self.center_y:.0f}) | "
            f"H-Err: {self.horizontal_error:+.2f} | "
            f"Bearing: {self.bearing_deg:+.1f}° | Dist: {dist_str}]"
        )

