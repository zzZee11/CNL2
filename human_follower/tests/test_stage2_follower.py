"""
Unit tests for human_follower/stage2_follower.py (Stage 2 Distance & Heading Follower).
"""

import time
import unittest
from human_follower.stage2_follower import Stage2Config, Stage2DistanceFollower
from vision.target_data import TargetObservation


class TestStage2Follower(unittest.TestCase):
    def setUp(self):
        self.config = Stage2Config(
            mock=True,
            headless=True,
            target_distance_m=1.20,
            distance_deadband_m=0.20,
            emergency_stop_dist_m=0.45,
            min_forward_speed=0.15,
            max_forward_speed=0.45,
            kp_dist=0.50,
            target_timeout_s=1.0,
        )
        self.follower = Stage2DistanceFollower(self.config)

    def tearDown(self):
        self.follower.stop()

    def test_sweet_spot_holding(self):
        # Target inside sweet spot (1.20m, centered) -> Motors completely stopped
        t0 = time.monotonic()
        obs = TargetObservation(
            target_valid=True,
            confidence=0.90,
            horizontal_error=0.02,
            distance=1.20,
            timestamp=t0,
        )
        vl, vr, state = self.follower.step(obs, mock_mid_tof=1.20)

        self.assertEqual(state, "HOLD_SWEET_SPOT")
        self.assertEqual(vl, 0.0)
        self.assertEqual(vr, 0.0)

    def test_person_walking_away_straight(self):
        # Target walking away (1.80m, centered) -> Drives forward straight
        t0 = time.monotonic()
        obs = TargetObservation(
            target_valid=True,
            confidence=0.90,
            horizontal_error=0.02,
            distance=1.80,
            timestamp=t0,
        )
        vl, vr, state = self.follower.step(obs, mock_mid_tof=1.80)

        self.assertEqual(state, "FOLLOW_STRAIGHT")
        self.assertGreater(vl, 0.0)
        self.assertGreater(vr, 0.0)
        self.assertAlmostEqual(vl, vr, places=3)
        self.assertGreaterEqual(vl, self.config.min_forward_speed)
        self.assertLessEqual(vl, self.config.max_forward_speed)

    def test_curving_while_following(self):
        # Target walking away and shifted right (1.70m, h_err = +0.25)
        # Left wheel should move faster than right wheel to turn right while driving forward
        t0 = time.monotonic()
        obs = TargetObservation(
            target_valid=True,
            confidence=0.90,
            horizontal_error=+0.25,
            bearing_deg=+8.0,
            distance=1.70,
            timestamp=t0,
        )
        vl, vr, state = self.follower.step(obs, mock_mid_tof=1.70)

        self.assertEqual(state, "FOLLOW_CURVE_RIGHT")
        self.assertGreater(vl, 0.0)
        self.assertGreater(vl, vr)  # Left wheel faster than right wheel

    def test_safety_stop_when_too_close(self):
        # Target is at 0.35m (< 0.45m emergency stop threshold) -> Immediate halt
        t0 = time.monotonic()
        obs = TargetObservation(
            target_valid=True,
            confidence=0.90,
            horizontal_error=0.0,
            distance=0.35,
            timestamp=t0,
        )
        vl, vr, state = self.follower.step(obs, mock_mid_tof=0.35)

        self.assertEqual(state, "SAFETY_STOP")
        self.assertEqual(vl, 0.0)
        self.assertEqual(vr, 0.0)

    def test_target_lost_safety_stop(self):
        # Target invalid -> Safe stop
        obs_lost = TargetObservation.invalid(timestamp=time.monotonic())
        vl, vr, state = self.follower.step(obs_lost, mock_mid_tof=1.5)

        self.assertEqual(state, "SEARCHING")
        self.assertEqual(vl, 0.0)
        self.assertEqual(vr, 0.0)

    def test_search_recovery_when_target_lost(self):
        t0 = time.monotonic()
        # 1. Target was previously seen moving to the right
        obs_seen = TargetObservation(
            target_valid=True,
            confidence=0.90,
            horizontal_error=+0.30,
            distance=1.60,
            timestamp=t0,
        )
        self.follower.step(obs_seen, mock_mid_tof=1.60)

        # 2. Target suddenly lost -> Robot sweeps right to find target
        obs_lost = TargetObservation.invalid(timestamp=t0 + 0.1)
        vl, vr, state = self.follower.step(obs_lost, mock_mid_tof=1.60)
        self.assertEqual(state, "SEARCH_RECOVERY_RIGHT")
        self.assertGreater(vl, 0.0)
        self.assertLess(vr, 0.0)

        # 3. After search_duration_s (2.0s) without reacquiring, robot halts
        self.follower._last_seen_time = time.monotonic() - 3.0
        obs_expired = TargetObservation.invalid(timestamp=time.monotonic())
        vl, vr, state = self.follower.step(obs_expired, mock_mid_tof=1.60)
        self.assertEqual(state, "SEARCHING")
        self.assertEqual(vl, 0.0)
        self.assertEqual(vr, 0.0)

    def test_full_lifecycle(self):
        self.assertTrue(self.follower.start())
        time.sleep(0.3)
        telem = self.follower.get_telemetry()
        self.assertIn("state", telem)
        self.assertIn("dist_m", telem)
        self.assertIn("v_left", telem)
        self.assertIn("v_right", telem)
        self.follower.stop()
        self.assertEqual(self.follower.state, "STOPPED")


if __name__ == "__main__":
    unittest.main()

