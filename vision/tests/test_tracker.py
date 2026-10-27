"""
Unit tests for vision.tracker (SinglePersonTracker).
"""

import time
import unittest
from vision.config import VisionConfig
from vision.detector import Detection
from vision.tracker import SinglePersonTracker


class TestTracker(unittest.TestCase):
    def setUp(self):
        self.config = VisionConfig(
            camera_width=640,
            camera_height=480,
            ema_alpha=0.50,  # 50% current, 50% previous
            track_lost_timeout_s=1.0,
        )
        self.tracker = SinglePersonTracker(self.config)

    def test_single_detection_update(self):
        det = Detection(bbox=(200.0, 100.0, 400.0, 400.0), confidence=0.85)
        obs = self.tracker.update([det], timestamp=10.0)

        self.assertTrue(obs.target_valid)
        self.assertEqual(obs.confidence, 0.85)
        self.assertEqual(obs.bbox, (200.0, 100.0, 400.0, 400.0))

        # Center calculation: cx = (200+400)/2 = 300, cy = (100+400)/2 = 250
        self.assertEqual(obs.center_x, 300.0)
        self.assertEqual(obs.center_y, 250.0)

        # Horizontal error: (300 - 320) = -20 px
        self.assertEqual(obs.horizontal_error_px, -20.0)
        # Normalized: -20 / 320 = -0.0625
        self.assertAlmostEqual(obs.horizontal_error, -0.0625, places=4)

    def test_temporal_ema_smoothing(self):
        # Frame 1: bbox at (100, 100, 200, 200)
        det1 = Detection(bbox=(100.0, 100.0, 200.0, 200.0), confidence=0.80)
        obs1 = self.tracker.update([det1], timestamp=10.0)
        self.assertEqual(obs1.bbox, (100.0, 100.0, 200.0, 200.0))

        # Frame 2: sudden jump / jitter to (120, 120, 220, 220)
        # With alpha=0.5, smoothed bbox should be exactly midway: (110, 110, 210, 210)
        det2 = Detection(bbox=(120.0, 120.0, 220.0, 220.0), confidence=0.90)
        obs2 = self.tracker.update([det2], timestamp=10.05)

        self.assertTrue(obs2.target_valid)
        self.assertEqual(obs2.bbox, (110.0, 110.0, 210.0, 210.0))
        self.assertEqual(obs2.center_x, 160.0)
        self.assertEqual(obs2.center_y, 160.0)
        self.assertAlmostEqual(obs2.confidence, 0.85, places=3)

    def test_missed_detection_coasting(self):
        # Frame 1: Valid detection at t = 1.0s
        det = Detection(bbox=(200.0, 100.0, 400.0, 400.0), confidence=0.80)
        self.tracker.update([det], timestamp=1.0)

        # Frame 2: Empty detection at t = 1.4s (within 1.0s timeout) -> Should coast
        obs_coast = self.tracker.update([], timestamp=1.4)
        self.assertTrue(obs_coast.target_valid)
        # Bbox maintained
        self.assertEqual(obs_coast.bbox, (200.0, 100.0, 400.0, 400.0))
        # Confidence should be partially decayed
        self.assertLess(obs_coast.confidence, 0.80)
        self.assertGreater(obs_coast.confidence, 0.0)

    def test_target_invalidation_after_timeout(self):
        # Frame 1: Valid detection at t = 1.0s
        det = Detection(bbox=(200.0, 100.0, 400.0, 400.0), confidence=0.80)
        self.tracker.update([det], timestamp=1.0)

        # Frame 2: Empty detection at t = 2.5s (exceeds 1.0s timeout) -> Invalidated!
        obs_lost = self.tracker.update([], timestamp=2.5)
        self.assertFalse(obs_lost.target_valid)
        self.assertIsNone(obs_lost.bbox)
        self.assertEqual(obs_lost.confidence, 0.0)
        self.assertFalse(self.tracker.is_tracking())

    def test_multi_candidate_selection(self):
        # When two people appear, tracker selects the most prominent one
        det1 = Detection(bbox=(50.0, 50.0, 100.0, 150.0), confidence=0.60)   # Small, lower conf
        det2 = Detection(bbox=(200.0, 100.0, 400.0, 400.0), confidence=0.90) # Large, high conf

        obs = self.tracker.update([det1, det2], timestamp=1.0)
        self.assertTrue(obs.target_valid)
        # Should pick det2
        self.assertEqual(obs.center_x, 300.0)
        self.assertEqual(obs.confidence, 0.90)


if __name__ == "__main__":
    unittest.main()

