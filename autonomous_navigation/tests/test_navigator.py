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
        # Test various formats of user input
        self.assertEqual(parse_goto_input("goto 1 5"), (1.0, 5.0, None))
        self.assertEqual(parse_goto_input("goto 1.0 5.0 0.8"), (1.0, 5.0, 0.8))
        self.assertEqual(parse_goto_input("goto (1, 5)"), (1.0, 5.0, None))
        self.assertEqual(parse_goto_input("goto (-2.5, 3.0) 0.6"), (-2.5, 3.0, 0.6))
        self.assertEqual(parse_goto_input("navigate 4 -2"), (4.0, -2.0, None))
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


if __name__ == "__main__":
    unittest.main()

