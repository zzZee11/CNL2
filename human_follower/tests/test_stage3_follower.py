"""
Unit tests for human_follower/stage3_follower.py (Stage 3 Human Follower with APF Obstacle Avoidance).
"""

import time
import unittest
from human_follower.stage3_follower import Stage3Config, Stage3ObstacleFollower
from vision.target_data import TargetObservation


class TestStage3Follower(unittest.TestCase):
    def setUp(self):
        self.config = Stage3Config(
            mock=True,
            headless=True,
            target_distance_m=1.00,
            distance_deadband_m=0.20,
            emergency_stop_dist_m=0.35,
            obstacle_safe_dist_m=0.65,
            min_forward_speed=0.15,
            max_forward_speed=0.40,
            k_rep=0.20,
            target_timeout_s=1.0,
        )
        self.follower = Stage3ObstacleFollower(self.config)

    def tearDown(self):
        self.follower.stop()

    def test_sweet_spot_holding(self):
        # Target inside sweet spot (1.20m, centered), no obstacles -> Motors completely stopped
        t0 = time.monotonic()
        obs = TargetObservation(
            target_valid=True,
            confidence=0.90,
            horizontal_error=0.02,
            distance=1.20,
            timestamp=t0,
        )
        vl, vr, state = self.follower.step(obs, mock_tofs=(9.99, 9.99, 9.99))

        self.assertEqual(state, "HOLD_SWEET_SPOT")
        self.assertEqual(vl, 0.0)
        self.assertEqual(vr, 0.0)

    def test_clear_path_following_straight(self):
        # Target walking away (1.80m, centered), no obstacles (9.99m) -> Drives straight forward
        t0 = time.monotonic()
        obs = TargetObservation(
            target_valid=True,
            confidence=0.90,
            horizontal_error=0.02,
            distance=1.80,
            timestamp=t0,
        )
        vl, vr, state = self.follower.step(obs, mock_tofs=(9.99, 9.99, 9.99))

        self.assertEqual(state, "FOLLOW_STRAIGHT")
        self.assertGreater(vl, 0.0)
        self.assertGreater(vr, 0.0)
        self.assertAlmostEqual(vl, vr, places=3)

    def test_person_walks_away_triggers_forward_drive(self):
        # Target at 2.50m (far vision), sensors report 9.99m (clear)
        t0 = time.monotonic()
        obs = TargetObservation(
            target_valid=True,
            confidence=0.90,
            horizontal_error=0.01,
            distance=2.50,
            timestamp=t0,
        )
        vl, vr, state = self.follower.step(obs, mock_tofs=(9.99, 9.99, 9.99))

        self.assertEqual(state, "FOLLOW_STRAIGHT")
        self.assertGreaterEqual(vl, self.config.min_forward_speed)
        self.assertLessEqual(vl, self.config.max_forward_speed)
        self.assertAlmostEqual(vl, vr, places=3)

    def test_human_distance_sensor_fusion(self):
        # 1. Close range centered person: ToF Laser takes precedence
        t0 = time.monotonic()
        obs_close = TargetObservation(
            target_valid=True,
            confidence=0.90,
            horizontal_error=0.05,
            distance=1.35,
            timestamp=t0,
        )
        dist_m, source = self.follower._estimate_human_distance(obs_close, mid_dist=1.25)
        self.assertEqual(dist_m, 1.25)
        self.assertEqual(source, "TOF_LASER")

        # 2. Long range: ToF is out of range (> 1.85m), falls back to Vision Proxy
        obs_far = TargetObservation(
            target_valid=True,
            confidence=0.90,
            horizontal_error=0.02,
            distance=2.40,
            timestamp=t0,
        )
        dist_m, source = self.follower._estimate_human_distance(obs_far, mid_dist=9.99)
        self.assertEqual(dist_m, 2.40)
        self.assertEqual(source, "VISION_PROXY")

        # 3. Target offset horizontally: ignores center ToF and uses Vision Proxy
        obs_off_center = TargetObservation(
            target_valid=True,
            confidence=0.90,
            horizontal_error=0.35,
            distance=1.60,
            timestamp=t0,
        )
        dist_m, source = self.follower._estimate_human_distance(obs_off_center, mid_dist=0.80)
        self.assertEqual(dist_m, 1.60)
        self.assertEqual(source, "VISION_PROXY")

    def test_left_obstacle_evasion(self):
        # Human straight ahead at 1.80m, obstacle on the left (left ToF = 0.45m < 0.65m safe dist)
        # Left obstacle should push robot to the right: Left wheel moves faster than right wheel
        t0 = time.monotonic()
        obs = TargetObservation(
            target_valid=True,
            confidence=0.90,
            horizontal_error=0.0,
            distance=1.80,
            timestamp=t0,
        )
        vl, vr, state = self.follower.step(obs, mock_tofs=(0.45, 9.99, 9.99))

        self.assertEqual(state, "AVOIDING_OBSTACLE_RIGHT")
        self.assertGreater(vl, 0.0)
        self.assertGreater(vl, vr)  # Curves right to avoid left obstacle

    def test_right_obstacle_evasion(self):
        # Human straight ahead at 1.80m, obstacle on the right (right ToF = 0.45m < 0.65m safe dist)
        # Right obstacle should push robot to the left: Right wheel moves faster than left wheel
        t0 = time.monotonic()
        obs = TargetObservation(
            target_valid=True,
            confidence=0.90,
            horizontal_error=0.0,
            distance=1.80,
            timestamp=t0,
        )
        vl, vr, state = self.follower.step(obs, mock_tofs=(9.99, 9.99, 0.45))

        self.assertEqual(state, "AVOIDING_OBSTACLE_LEFT")
        self.assertGreater(vr, 0.0)
        self.assertGreater(vr, vl)  # Curves left to avoid right obstacle

    def test_human_disambiguation_no_repulsion(self):
        # All sensors clear (> 0.65m safe dist) -> Repulsive force is zero
        t0 = time.monotonic()
        obs = TargetObservation(
            target_valid=True,
            confidence=0.90,
            horizontal_error=0.02,
            distance=1.20,
            timestamp=t0,
        )
        f_att, f_rep, f_net = self.follower.calculate_apf(
            obs, dist_m=1.20, left_dist=9.99, mid_dist=9.99, right_dist=9.99
        )
        self.assertEqual(f_rep, (0.0, 0.0))

    def test_emergency_stop_when_hazard_too_close(self):
        # An obstacle is dangerously close (0.25m < 0.35m emergency stop threshold) -> Immediate halt
        t0 = time.monotonic()
        obs = TargetObservation(
            target_valid=True,
            confidence=0.90,
            horizontal_error=0.0,
            distance=1.50,
            timestamp=t0,
        )
        vl, vr, state = self.follower.step(obs, mock_tofs=(0.25, 9.99, 9.99))

        self.assertEqual(state, "SAFETY_STOP")
        self.assertEqual(vl, 0.0)
        self.assertEqual(vr, 0.0)

    def test_full_lifecycle(self):
        self.assertTrue(self.follower.start())
        time.sleep(0.3)
        telem = self.follower.get_telemetry()
        self.assertIn("state", telem)
        self.assertIn("f_att", telem)
        self.assertIn("f_rep", telem)
        self.assertIn("f_net", telem)
        self.assertIn("v_left", telem)
        self.assertIn("v_right", telem)
        self.follower.stop()
        self.assertEqual(self.follower.state, "STOPPED")


if __name__ == "__main__":
    unittest.main()
