"""
autonomous_navigation/tests/test_navigator.py
---------------------------------------------
Unit tests for Cartesian plane navigation, A* path planning, APF avoidance, and plot generation.
"""

import os
import time
import math
import unittest
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
from autonomous_navigation.navigator import AutonomousNavigator
from autonomous_navigation.plotter import save_navigation_plot
from autonomous_navigation.main import parse_goto_input


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
        # Test various formats of user input with optional speed and orientation
        self.assertEqual(parse_goto_input("goto 1 5"), (1.0, 5.0, None, None))
        self.assertEqual(parse_goto_input("goto 1.0 5.0 0.8"), (1.0, 5.0, 0.8, None))
        self.assertEqual(parse_goto_input("goto 1 2 90 deg"), (1.0, 2.0, None, 90.0))
        self.assertEqual(parse_goto_input("goto 1 2 heading 90"), (1.0, 2.0, None, 90.0))
        self.assertEqual(parse_goto_input("goto 1 1 @ 180"), (1.0, 1.0, None, 180.0))
        self.assertEqual(parse_goto_input("goto (-2.5, 3.0) 0.6 m/s 90 deg"), (-2.5, 3.0, 0.6, 90.0))
        self.assertEqual(parse_goto_input("navigate 4 -2 90"), (4.0, -2.0, None, 90.0))
        self.assertIsNone(parse_goto_input("invalid command"))

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

    def test_continuous_obstacles_clustering_and_swept_path(self):
        from autonomous_navigation.plotter import cluster_obstacles, compute_swept_path, get_robot_box_corners

        # 1. Test clustering continuous points into ordered chains
        wall = [(0.0, 1.0), (0.4, 1.0), (0.2, 1.0), (0.6, 1.0)]
        isolated = [(3.0, 3.0)]
        clusters = cluster_obstacles(wall + isolated, max_link_dist=0.35)
        self.assertEqual(len(clusters), 2)
        # Wall cluster has 4 points in monotonic order
        wall_c = next(c for c in clusters if len(c) == 4)
        wall_xs = [p[0] for p in wall_c]
        self.assertTrue(wall_xs == [0.0, 0.2, 0.4, 0.6] or wall_xs == [0.6, 0.4, 0.2, 0.0])

        # 2. Test swept-path polygon corridor
        traj = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
        poly, left_t, right_t = compute_swept_path(traj, width=0.134)
        self.assertEqual(len(poly), 6)
        self.assertEqual(len(left_t), 3)
        self.assertEqual(len(right_t), 3)
        # Left track should be at +W/2 (+0.067m lateral) and right at -W/2 (-0.067m)
        self.assertAlmostEqual(left_t[0][1], 0.067, delta=0.005)
        self.assertAlmostEqual(right_t[0][1], -0.067, delta=0.005)

        # 3. Test robot box corners
        corners = get_robot_box_corners(0.0, 0.0, theta=0.0, length=0.18, width=0.134)
        self.assertEqual(len(corners), 4)

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
            self.assertIn("robot_dimensions", data)
        if os.path.exists(json_file):
            os.remove(json_file)


if __name__ == "__main__":
    unittest.main()

