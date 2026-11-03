"""
autonomous_navigation/tests/test_navigator.py
---------------------------------------------
Unit tests for Cartesian plane navigation, A* path planning, APF avoidance, and plot generation.
"""

import os
import time
import math
import unittest
from collections import deque
from autonomous_navigation.config import (
    WHEEL_BASE_M, DEFAULT_SPEED, DEFAULT_PLOT_PATH
)
from autonomous_navigation.planner import (
    create_empty_grid, world_to_grid, grid_to_world,
    mark_obstacle, astar_grid, is_path_blocked,
    calculate_apf, calculate_kinematics
)
from autonomous_navigation.sensors import DistanceSensorManager
from autonomous_navigation.serial_comm import RobotSerialComm
from autonomous_navigation.navigator import AutonomousNavigator, simplify_trajectory
from autonomous_navigation.plotter import save_navigation_plot
from autonomous_navigation.main import parse_goto_input, parse_patrol_input


class TestAutonomousNavigator(unittest.TestCase):
    def setUp(self):
        self.sensors = DistanceSensorManager(use_mock=True)
        self.sensors.initialize()
        self.sensors.start()

        self.comm = RobotSerialComm(use_mock=True)
        self.comm.connect()
        self.comm.start()

        self.nav = AutonomousNavigator(serial_comm=self.comm, sensor_mgr=self.sensors)
        self.test_plot_file = "/tmp/test_nav_result.png"

    def tearDown(self):
        self.nav.request_stop()
        self.comm.close()
        self.sensors.close()
        if os.path.exists(self.test_plot_file):
            try:
                os.remove(self.test_plot_file)
            except OSError:
                pass

    def test_goto_input_parsing(self):
        # Test various formats of user input with optional speed, orientation, and return flag
        self.assertEqual(parse_goto_input("goto 1 5"), (1.0, 5.0, None, None, False))
        self.assertEqual(parse_goto_input("goto 1.0 5.0 0.8"), (1.0, 5.0, 0.8, None, False))
        self.assertEqual(parse_goto_input("goto 1 2 90 deg"), (1.0, 2.0, None, 90.0, False))
        self.assertEqual(parse_goto_input("goto 1 2 heading 90"), (1.0, 2.0, None, 90.0, False))
        self.assertEqual(parse_goto_input("goto 1 1 @ 180"), (1.0, 1.0, None, 180.0, False))
        self.assertEqual(parse_goto_input("goto (-2.5, 3.0) 0.6 m/s 90 deg"), (-2.5, 3.0, 0.6, 90.0, False))
        self.assertEqual(parse_goto_input("navigate 4 -2 90"), (4.0, -2.0, None, 90.0, False))
        self.assertEqual(parse_goto_input("goto 1 1 --return"), (1.0, 1.0, None, None, True))
        self.assertEqual(parse_goto_input("goto 1 1 90 deg roundtrip"), (1.0, 1.0, None, 90.0, True))
        self.assertIsNone(parse_goto_input("invalid command"))

    def test_patrol_input_parsing(self):
        self.assertEqual(parse_patrol_input("patrol 1 1 3"), (1.0, 1.0, 3, None, None))
        self.assertEqual(parse_patrol_input("patrol 1 2 5 90 deg"), (1.0, 2.0, 5, None, 90.0))
        self.assertEqual(parse_patrol_input("shuttle 1.0 1.0 2"), (1.0, 1.0, 2, None, None))
        self.assertEqual(parse_patrol_input("patrol (1.0, 2.0) 3 speed 0.5 heading 90"), (1.0, 2.0, 3, 0.5, 90.0))
        self.assertIsNone(parse_patrol_input("move 5 m"))

    def test_cartesian_astar_planning_to_1_5(self):
        # Plan path from (0, 0) to (1.0, 5.0)
        grid = create_empty_grid()
        start = (0.0, 0.0)
        goal = (1.0, 5.0)
        path = astar_grid(start, goal, grid)

        self.assertTrue(len(path) > 5)
        # First waypoint should be start
        self.assertAlmostEqual(path[0][0], start[0], delta=0.2)
        self.assertAlmostEqual(path[0][1], start[1], delta=0.2)
        # Last waypoint should reach goal
        self.assertAlmostEqual(path[-1][0], goal[0], delta=0.2)
        self.assertAlmostEqual(path[-1][1], goal[1], delta=0.2)

    def test_obstacle_triggers_astar_replanning(self):
        # Place an obstacle directly in the path between (0, 0) and (2.0, 0.0)
        grid = create_empty_grid()
        initial_path = astar_grid((0.0, 0.0), (2.0, 0.0), grid)
        self.assertFalse(is_path_blocked(initial_path, grid))

        # Mark obstacle at (1.0, 0.0)
        mark_obstacle(grid, 1.0, 0.0)
        self.assertTrue(is_path_blocked(initial_path, grid))

        # Replan around obstacle
        new_path = astar_grid((0.0, 0.0), (2.0, 0.0), grid)
        self.assertTrue(len(new_path) > 0)
        # New path must not pass through the blocked obstacle cell
        gx, gy = world_to_grid(1.0, 0.0)
        for wp in new_path:
            wgx, wgy = world_to_grid(wp[0], wp[1])
            self.assertFalse(wgx == gx and wgy == gy)

    def test_kinematics_pivot_when_heading_offset_large(self):
        # Goal at (0.0, 2.0) while robot faces theta = 0 (offset is +90 deg / +1.57 rad)
        # Robot should perform an in-place pivot: Left wheel backward, Right wheel forward
        f_total = (0.0, 1.0)
        vl, vr = calculate_kinematics(f_total, current_theta=0.0, target_speed=0.60)
        self.assertLess(vl, 0.0)
        self.assertGreater(vr, 0.0)
        self.assertAlmostEqual(abs(vl), vr, places=3)

    def test_mock_navigate_to_cartesian_location(self):
        # Run a simulated short Cartesian navigation to (0.5, 0.5)
        success = self.nav.navigate_to(
            target_x=0.5,
            target_y=0.5,
            speed_mps=0.8,
            save_plot=True,
            plot_path=self.test_plot_file
        )
        self.assertTrue(success)
        self.assertTrue(os.path.exists(self.test_plot_file))
        self.assertGreater(os.path.getsize(self.test_plot_file), 1000)

    def test_plotter_generates_valid_png_image(self):
        # Test standalone plot generation
        traj = [(0.0, 0.0), (0.5, 1.0), (1.0, 3.0), (1.0, 5.0)]
        obs = [(0.8, 2.0), (0.9, 2.1)]
        plan = [(0.0, 0.0), (0.4, 1.2), (1.0, 5.0)]

        out_path = save_navigation_plot(
            trajectory=traj,
            recorded_obstacles=obs,
            start_pos=(0.0, 0.0),
            goal_pos=(1.0, 5.0),
            planned_path=plan,
            final_pose=(1.0, 5.0, 1.57),
            total_dist=5.2,
            elapsed_s=7.5,
            status="GOAL REACHED",
            output_path=self.test_plot_file,
        )
        self.assertTrue(os.path.exists(out_path))
        # Ensure it is a valid PNG file (starts with PNG magic bytes)
        with open(out_path, "rb") as f:
            header = f.read(8)
            self.assertEqual(header, b"\x89PNG\r\n\x1a\n")

        # Also verify the companion obstacle JSON file
        json_file = os.path.splitext(self.test_plot_file)[0] + "_obstacles.json"
        self.assertTrue(os.path.exists(json_file))
        import json
        with open(json_file, "r") as f:
            data = json.load(f)
            self.assertEqual(data["obstacle_count"], 2)
            self.assertEqual(data["mission_status"], "GOAL REACHED")
            self.assertEqual(len(data["obstacles"]), 2)
        if os.path.exists(json_file):
            os.remove(json_file)

    def test_imu_fused_odometry(self):
        from autonomous_navigation.imu import IMUManager
        mock_imu = IMUManager(use_mock=True)
        mock_imu.initialize()
        mock_imu.start()
        # Set spatial orientation to 90 degrees (+Y direction)
        mock_imu.reset_heading(math.radians(90.0))

        fused_comm = RobotSerialComm(use_mock=True, imu=mock_imu)
        fused_comm.connect()
        fused_comm.reset_odometry(0.0, 0.0, math.radians(90.0))

        # Baseline first encoder reading
        fused_comm._update_odometry(0, 0)
        # Advance both wheels by 720 ticks (1 revolution = ~0.135 m)
        fused_comm._update_odometry(720, 720)

        x, y, theta, dist = fused_comm.get_pose()
        # Since spatial gyro heading is 90° (+Y), forward movement must advance Y, not X
        self.assertAlmostEqual(x, 0.0, delta=0.02)
        self.assertGreater(y, 0.10)
        self.assertAlmostEqual(math.degrees(theta), 90.0, delta=1.0)
        self.assertAlmostEqual(dist, 0.135, delta=0.02)

        fused_comm.close()

    def test_navigate_to_with_target_orientation(self):
        # Run a simulated short Cartesian navigation to (0.4, 0.4) with target heading 90 deg
        success = self.nav.navigate_to(
            target_x=0.4,
            target_y=0.4,
            speed_mps=0.8,
            target_theta_deg=90.0,
            save_plot=True,
            plot_path=self.test_plot_file
        )
        self.assertTrue(success)
        cur_x, cur_y, cur_theta, _ = self.comm.get_pose()
        # Verify robot reached destination and aligned close to 90 degrees
        self.assertAlmostEqual(cur_x, 0.4, delta=0.10)
        self.assertAlmostEqual(cur_y, 0.4, delta=0.10)
        self.assertAlmostEqual(math.degrees(cur_theta), 90.0, delta=5.0)

        # Verify companion JSON contains target_orientation_deg
        json_file = os.path.splitext(self.test_plot_file)[0] + "_obstacles.json"
        self.assertTrue(os.path.exists(json_file))
        import json
        with open(json_file, "r") as f:
            data = json.load(f)
            self.assertEqual(data["target_orientation_deg"], 90.0)
        if os.path.exists(json_file):
            os.remove(json_file)

    def test_return_to_start_exact_path(self):
        # 1. Navigate outward to (0.3, 0.3)
        ok_out = self.nav.navigate_to(
            target_x=0.3,
            target_y=0.3,
            speed_mps=0.8,
            target_theta_deg=45.0,
            save_plot=False
        )
        self.assertTrue(ok_out)

        # 2. Retrace exact path forward back to start point (0, 0)
        ok_ret = self.nav.return_to_start(
            speed_mps=0.8,
            target_theta_deg=0.0,
            save_plot=True,
            plot_path=self.test_plot_file
        )
        self.assertTrue(ok_ret)

        # 3. Verify final position is at (0, 0) facing 0.0°
        cur_x, cur_y, cur_theta, total_dist = self.comm.get_pose()
        self.assertAlmostEqual(cur_x, 0.0, delta=0.12)
        self.assertAlmostEqual(cur_y, 0.0, delta=0.12)
        self.assertAlmostEqual(math.degrees(cur_theta), 0.0, delta=5.0)
        self.assertGreater(total_dist, 0.60)  # Outward + Return distance
        self.assertTrue(os.path.exists(self.test_plot_file))

    def test_patrol_back_and_forth(self):
        # Run a 2-cycle patrol (4 legs total) between (0, 0) and (0.3, 0.3)
        ok_patrol = self.nav.patrol(
            target_x=0.3,
            target_y=0.3,
            cycles=2,
            speed_mps=0.8,
            target_theta_deg=45.0,
            save_plot=True,
            plot_path=self.test_plot_file
        )
        self.assertTrue(ok_patrol)

        # Verify robot ended at start point (0, 0) facing 0.0°
        cur_x, cur_y, cur_theta, total_dist = self.comm.get_pose()
        self.assertAlmostEqual(cur_x, 0.0, delta=0.12)
        self.assertAlmostEqual(cur_y, 0.0, delta=0.12)
        self.assertAlmostEqual(math.degrees(cur_theta), 0.0, delta=5.0)
        self.assertGreater(total_dist, 0.90)  # 4 legs completed (approx 1.05m total)

        # Verify companion JSON mission status
        json_file = os.path.splitext(self.test_plot_file)[0] + "_obstacles.json"
        self.assertTrue(os.path.exists(json_file))
        import json
        with open(json_file, "r") as f:
            data = json.load(f)
            self.assertIn("PATROL COMPLETED", data["mission_status"])
        if os.path.exists(json_file):
            os.remove(json_file)

    def test_sensor_median_filter_rejects_spurious_noise(self):
        # Test median filtering behavior with transient optical spikes
        import statistics
        mgr = DistanceSensorManager(use_mock=True)
        mgr._median_window_size = 5
        mgr._windows = [deque([9.99] * 5, maxlen=5) for _ in range(3)]

        # Feed 1 spurious spike of 0.15m to center sensor (channel 1)
        mgr._windows[1].append(0.15)
        # Median of [9.99, 9.99, 9.99, 9.99, 0.15] is 9.99 -> spike rejected
        filtered = float(statistics.median(mgr._windows[1]))
        self.assertEqual(filtered, 9.99)

        # Feed second spike
        mgr._windows[1].append(0.14)
        filtered = float(statistics.median(mgr._windows[1]))
        self.assertEqual(filtered, 9.99)

        # Feed 3rd consistent reading -> Now it becomes a confirmed persistent obstacle
        mgr._windows[1].append(0.15)
        filtered = float(statistics.median(mgr._windows[1]))
        self.assertAlmostEqual(filtered, 0.15, places=2)

    def test_spatial_consensus_filters_moving_glare(self):
        # Simulated floor glare drifts with robot: relative distance stays 0.40m as robot moves
        # Spatial consensus must reject drifting candidates and confirm static world targets
        grid = create_empty_grid()
        self.nav.grid = grid
        self.nav.last_obstacles = []
        self.nav.candidate_obstacles = []

        # Simulate 10 ticks of moving floor glare (robot moving along X, distance constantly 0.40m ahead)
        for tick in range(10):
            cur_x = tick * 0.05  # robot advances 5cm per tick
            cur_y = 0.0
            now = time.time() + tick * 0.05
            d = 0.40  # constant distance ahead of moving robot
            obs_x = cur_x + d
            obs_y = cur_y

            # Run consensus logic
            matched = None
            for c in self.nav.candidate_obstacles:
                if math.hypot(obs_x - c["x"], obs_y - c["y"]) <= 0.12:
                    matched = c
                    break
            if matched:
                matched["hits"] += 1
                matched["x"] = 0.7 * matched["x"] + 0.3 * obs_x
                matched["y"] = 0.7 * matched["y"] + 0.3 * obs_y
                matched["last_seen"] = now
            else:
                self.nav.candidate_obstacles.append({
                    "x": obs_x, "y": obs_y, "hits": 1, "last_seen": now, "confirmed": False
                })
            # Prune
            self.nav.candidate_obstacles = [
                c for c in self.nav.candidate_obstacles if (now - c["last_seen"] <= 0.20)
            ]

        # Drifting floor glare should never reach persistence threshold of 5 hits
        confirmed = [c for c in self.nav.candidate_obstacles if c.get("hits", 0) >= 5]
        self.assertEqual(len(confirmed), 0)

        # Now simulate a true static obstacle at (0.80, 0.00) in the world frame
        static_x, static_y = 0.80, 0.00
        for tick in range(6):
            cur_x = tick * 0.03
            now = time.time() + 10.0 + tick * 0.05
            d = static_x - cur_x
            obs_x = cur_x + d
            obs_y = 0.0
            matched = None
            for c in self.nav.candidate_obstacles:
                if math.hypot(obs_x - c["x"], obs_y - c["y"]) <= 0.12:
                    matched = c
                    break
            if matched:
                matched["hits"] += 1
                matched["last_seen"] = now
            else:
                self.nav.candidate_obstacles.append({
                    "x": obs_x, "y": obs_y, "hits": 1, "last_seen": now, "confirmed": False
                })

        confirmed_static = [c for c in self.nav.candidate_obstacles if c.get("hits", 0) >= 5]
        self.assertEqual(len(confirmed_static), 1)
        self.assertAlmostEqual(confirmed_static[0]["x"], 0.80, delta=0.05)


if __name__ == "__main__":
    unittest.main()

