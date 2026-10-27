"""
vision/target_estimator.py
--------------------------
Calculates spatial target parameters:
1. Bearing angle relative to camera optical axis.
2. Calibrated distance proxy (only when physical calibration is enabled).

Design Note on Metric Distance:
A 2D bounding box alone cannot provide precise metric distance without known reference
geometry, camera tilt angle, and ground plane assumptions. Distance is left as None unless
`distance_calibration_enabled` is explicitly configured with measured camera calibration.
"""

import math
from typing import Optional

from .config import VisionConfig
from .target_data import TargetObservation


class TargetEstimator:
    """
    Enriches TargetObservation with geometric angle (bearing) and calibrated distance.
    """

    def __init__(self, config: VisionConfig) -> None:
        self.config = config
        self.frame_width = config.camera_width
        self.frame_height = config.camera_height
        self.hfov_deg = config.hfov_deg

        # Compute or assign focal length in pixels
        if config.focal_length_px is not None:
            self.fx = config.focal_length_px
        else:
            half_fov_rad = math.radians(self.hfov_deg / 2.0)
            self.fx = (self.frame_width / 2.0) / math.tan(half_fov_rad)

        self.distance_enabled = config.distance_calibration_enabled
        self.assumed_height = config.assumed_person_height_m

    def estimate(
        self,
        observation: TargetObservation,
        frame_width: Optional[int] = None,
        frame_height: Optional[int] = None,
    ) -> TargetObservation:
        """
        Enrich observation in-place and return it.
        """
        if not observation.target_valid or observation.bbox is None:
            observation.bearing = 0.0
            observation.bearing_deg = 0.0
            observation.distance = None
            return observation

        fw = frame_width if frame_width is not None else self.frame_width
        fh = frame_height if frame_height is not None else self.frame_height

        # Calculate focal length for current frame width if resolution differs
        if fw == self.frame_width:
            fx = self.fx
        else:
            half_fov_rad = math.radians(self.hfov_deg / 2.0)
            fx = (fw / 2.0) / math.tan(half_fov_rad)

        # ── 1. Bearing Angle Calculation ──────────────────────────────────────
        # Offset from optical axis in pixels
        dx_px = observation.center_x - (fw / 2.0)
        bearing_rad = math.atan2(dx_px, fx)
        bearing_deg = math.degrees(bearing_rad)

        observation.bearing = bearing_rad
        observation.bearing_deg = bearing_deg

        # ── 2. Calibrated Distance Estimation ─────────────────────────────────
        if self.distance_enabled:
            x1, y1, x2, y2 = observation.bbox
            box_height = max(1.0, y2 - y1)
            # Pinhole perspective projection: Distance = (FocalLength * RealHeight) / ImageHeight
            estimated_dist = (fx * self.assumed_height) / box_height
            # Clamp to physical sensible bounds (0.3m to 15m)
            observation.distance = max(0.3, min(15.0, estimated_dist))
        else:
            observation.distance = None

        return observation

