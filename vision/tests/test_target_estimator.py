"""
Unit tests for vision.target_estimator (TargetEstimator).
"""

import math
import unittest
from vision.config import VisionConfig
from vision.target_data import TargetObservation
from vision.target_estimator import TargetEstimator


class TestTargetEstimator(unittest.TestCase):
    def test_bearing_dead_center(self):
        config = VisionConfig(
            camera_width=640,
            camera_height=480,
            hfov_deg=60.0,
            distance_calibration_enabled=False,
        )
        estimator = TargetEstimator(config)

        # Observation exactly at center (cx = 320)
        obs = TargetObservation(
            target_valid=True,
            confidence=0.85,
            bbox=(270.0, 100.0, 370.0, 380.0),
            center_x=320.0,
            center_y=240.0,
            horizontal_error=0.0,
            horizontal_error_px=0.0,
        )
        enriched = estimator.estimate(obs)
        self.assertAlmostEqual(enriched.bearing, 0.0, places=4)
        self.assertAlmostEqual(enriched.bearing_deg, 0.0, places=3)
        self.assertIsNone(enriched.distance)

    def test_bearing_lateral_offset(self):
        config = VisionConfig(
            camera_width=640,
            camera_height=480,
            hfov_deg=60.0,
            distance_calibration_enabled=False,
        )
        estimator = TargetEstimator(config)

        # Target shifted right of center (cx = 400 > 320)
        obs_right = TargetObservation(
            target_valid=True,
            confidence=0.85,
            bbox=(350.0, 100.0, 450.0, 380.0),
            center_x=400.0,
            center_y=240.0,
        )
        enriched_right = estimator.estimate(obs_right)
        self.assertGreater(enriched_right.bearing, 0.0)
        self.assertGreater(enriched_right.bearing_deg, 0.0)

        # Target shifted left of center (cx = 240 < 320)
        obs_left = TargetObservation(
            target_valid=True,
            confidence=0.85,
            bbox=(190.0, 100.0, 290.0, 380.0),
            center_x=240.0,
            center_y=240.0,
        )
        enriched_left = estimator.estimate(obs_left)
        self.assertLess(enriched_left.bearing, 0.0)
        self.assertLess(enriched_left.bearing_deg, 0.0)

    def test_distance_calibration_toggle(self):
        # 1. When disabled, distance must remain None
        config_disabled = VisionConfig(distance_calibration_enabled=False)
        est_disabled = TargetEstimator(config_disabled)
        obs = TargetObservation(
            target_valid=True,
            confidence=0.85,
            bbox=(200.0, 50.0, 400.0, 350.0),  # height = 300 px
            center_x=300.0,
            center_y=200.0,
        )
        self.assertIsNone(est_disabled.estimate(obs).distance)

        # 2. When enabled, distance is computed
        config_enabled = VisionConfig(
            distance_calibration_enabled=True,
            assumed_person_height_m=1.70,
        )
        est_enabled = TargetEstimator(config_enabled)
        enriched = est_enabled.estimate(obs)
        self.assertIsNotNone(enriched.distance)
        self.assertGreater(enriched.distance, 0.5)
        self.assertLess(enriched.distance, 10.0)


if __name__ == "__main__":
    unittest.main()

