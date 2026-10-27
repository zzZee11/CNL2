"""
Unit tests for vision.target_data (TargetObservation).
"""

import json
import unittest
from vision.target_data import TargetObservation


class TestTargetData(unittest.TestCase):
    def test_default_invalid_observation(self):
        obs = TargetObservation.invalid(timestamp=100.0)
        self.assertFalse(obs.target_valid)
        self.assertFalse(obs.is_valid())
        self.assertEqual(obs.confidence, 0.0)
        self.assertIsNone(obs.bbox)
        self.assertIsNone(obs.distance)
        self.assertEqual(obs.timestamp, 100.0)

    def test_to_dict_and_json(self):
        obs = TargetObservation(
            target_valid=True,
            confidence=0.8765,
            bbox=(100.0, 50.0, 200.0, 300.0),
            center_x=150.0,
            center_y=175.0,
            horizontal_error=-0.25,
            horizontal_error_px=-80.0,
            bearing=-0.1234,
            bearing_deg=-7.07,
            distance=2.45,
            timestamp=123.456,
        )
        data = obs.to_dict()
        self.assertTrue(data["target_valid"])
        self.assertAlmostEqual(data["confidence"], 0.8765, places=3)
        self.assertEqual(data["bbox"], [100.0, 50.0, 200.0, 300.0])
        self.assertEqual(data["horizontal_error"], -0.25)
        self.assertEqual(data["bearing_deg"], -7.07)
        self.assertEqual(data["distance"], 2.45)

        json_str = obs.to_json()
        parsed = json.loads(json_str)
        self.assertEqual(parsed["center_x"], 150.0)

    def test_summary_formatting(self):
        obs_invalid = TargetObservation.invalid()
        self.assertIn("LOST", obs_invalid.summary())

        obs_valid = TargetObservation(
            target_valid=True,
            confidence=0.92,
            bbox=(50.0, 50.0, 150.0, 250.0),
            center_x=100.0,
            center_y=150.0,
            horizontal_error=+0.15,
            bearing_deg=+5.2,
            distance=3.10,
        )
        summary = obs_valid.summary()
        self.assertIn("VALID", summary)
        self.assertIn("92%", summary)
        self.assertIn("3.10m", summary)


if __name__ == "__main__":
    unittest.main()
