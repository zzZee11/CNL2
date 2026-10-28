"""
Unit tests for human_follower (Stage 1 Turret Follower).
"""

import time
import unittest
from human_follower.config import FollowerConfig
from human_follower.turret_follower import TurretFollower
from vision.target_data import TargetObservation


class TestTurretFollower(unittest.TestCase):
    def setUp(self):
        self.config = FollowerConfig(
            mock=True,
            headless=True,
            deadband=0.08,
            min_turn_speed=0.18,
            max_turn_speed=0.40,
            kp_turn=0.50,
            target_timeout_s=1.0,
        )
        self.follower = TurretFollower(self.config)

    def tearDown(self):
        self.follower.stop()

    def test_deadband_locked_center(self):
        # Target within deadband (|h_err| = 0.04 <= 0.08)
        obs = TargetObservation(
            target_valid=True,
            confidence=0.88,
            horizontal_error=+0.04,
            bearing_deg=+1.2,
            timestamp=time.monotonic(),
        )
        vl, vr, state = self.follower.step(obs)

        self.assertEqual(state, "LOCKED_CENTER")
        self.assertEqual(vl, 0.0)
        self.assertEqual(vr, 0.0)

    def test_pivot_right_when_target_on_right(self):
        # Target to the right (h_err = +0.35 > 0.08)
        obs = TargetObservation(
            target_valid=True,
            confidence=0.90,
            horizontal_error=+0.35,
            bearing_deg=+10.5,
            timestamp=time.monotonic(),
        )
        vl, vr, state = self.follower.step(obs)

        self.assertEqual(state, "PIVOT_RIGHT")
        self.assertGreater(vl, 0.0)  # Left wheel drives forward
        self.assertLess(vr, 0.0)     # Right wheel drives backward
        self.assertAlmostEqual(vl, -vr, places=4)
        # Verify clamped within min and max
        self.assertGreaterEqual(vl, self.config.min_turn_speed)
        self.assertLessEqual(vl, self.config.max_turn_speed)

    def test_pivot_left_when_target_on_left(self):
        # Target to the left (h_err = -0.45 < -0.08)
        obs = TargetObservation(
            target_valid=True,
            confidence=0.85,
            horizontal_error=-0.45,
            bearing_deg=-13.5,
            timestamp=time.monotonic(),
        )
        vl, vr, state = self.follower.step(obs)

        self.assertEqual(state, "PIVOT_LEFT")
        self.assertLess(vl, 0.0)     # Left wheel drives backward
        self.assertGreater(vr, 0.0)  # Right wheel drives forward
        self.assertAlmostEqual(-vl, vr, places=4)
        # Verify clamped
        self.assertGreaterEqual(vr, self.config.min_turn_speed)
        self.assertLessEqual(vr, self.config.max_turn_speed)

    def test_target_lost_safety_stop(self):
        # Invalid target -> safe stop
        obs_lost = TargetObservation.invalid(timestamp=time.monotonic())
        vl, vr, state = self.follower.step(obs_lost)

        self.assertEqual(state, "SEARCHING")
        self.assertEqual(vl, 0.0)
        self.assertEqual(vr, 0.0)

    def test_stale_target_safety_stop(self):
        # Valid observation, but 2.0s old (> 1.0s timeout)
        obs_stale = TargetObservation(
            target_valid=True,
            confidence=0.90,
            horizontal_error=+0.50,
            timestamp=time.monotonic() - 2.5,
        )
        vl, vr, state = self.follower.step(obs_stale)

        self.assertEqual(state, "SEARCHING")
        self.assertEqual(vl, 0.0)
        self.assertEqual(vr, 0.0)

    def test_full_service_lifecycle(self):
        self.assertTrue(self.follower.start())
        time.sleep(0.3)
        telem = self.follower.get_telemetry()
        self.assertIn("state", telem)
        self.assertIn("v_left", telem)
        self.assertIn("v_right", telem)
        self.follower.stop()
        self.assertEqual(self.follower.state, "STOPPED")


if __name__ == "__main__":
    unittest.main()

