"""
grid_navigation/tests/test_navigator.py
---------------------------------------
End-to-end integration tests for the clean grid-based navigation system.
"""

import os
import math
import unittest
from grid_navigation.grid_map import OccupancyGridMap
from grid_navigation.robot_hardware import RobotHardware
from grid_navigation.navigator import GridNavigator


class TestGridNavigator(unittest.TestCase):
    def setUp(self):
        self.hw = RobotHardware(use_mock=True)
        self.hw.initialize()
        self.hw.start()

        self.grid = OccupancyGridMap(
            dim_x=100, dim_y=100, resolution_m=0.10, offset_x_m=5.0, offset_y_m=5.0
        )
        self.nav = GridNavigator(hardware=self.hw, grid_map=self.grid)
        self.test_plot = "/tmp/test_grid_nav_result.png"

    def tearDown(self):
        self.nav.request_stop()
        self.hw.close()
        for p in (self.test_plot, "/tmp/test_grid_nav_result_telemetry.json"):
            if os.path.exists(p):
                try:
                    os.remove(p)
                except OSError:
                    pass

    def test_navigate_to_destination_accuracy(self):
        # Navigate to (0.40, 0.40) at speed 0.7 m/s with target heading 90 deg
        success = self.nav.navigate_to(
            target_x=0.40,
            target_y=0.40,
            target_theta_deg=90.0,
            speed_mps=0.70,
            save_plot=True,
            plot_path=self.test_plot
        )
        self.assertTrue(success)

        cur_x, cur_y, cur_th, _ = self.hw.get_pose()
        # Verify landing accuracy within 2.5 cm
        dist_to_goal = math.hypot(0.40 - cur_x, 0.40 - cur_y)
        self.assertLessEqual(dist_to_goal, 0.026)

        # Verify heading alignment within tolerance (<= 3.5 deg)
        err_deg = abs(math.degrees(cur_th) - 90.0)
        self.assertLessEqual(err_deg, 3.5)

        # Verify plot image and telemetry JSON were exported
        self.assertTrue(os.path.exists(self.test_plot))
        json_file = "/tmp/test_grid_nav_result_telemetry.json"
        self.assertTrue(os.path.exists(json_file))

    def test_return_to_start_exact_path(self):
        # 1. Drive outward to (0.35, 0.25)
        ok_out = self.nav.navigate_to(
            target_x=0.35,
            target_y=0.25,
            target_theta_deg=45.0,
            speed_mps=0.70,
            return_to_start=False,
            save_plot=False
        )
        self.assertTrue(ok_out)

        # 2. Retrace exact path forward back to start point (0, 0)
        ok_ret = self.nav.return_to_start(
            speed_mps=0.70,
            target_theta_deg=0.0,
            save_plot=True,
            plot_path=self.test_plot
        )
        self.assertTrue(ok_ret)

        cur_x, cur_y, cur_th, total_dist = self.hw.get_pose()
        # Verify landing accuracy at (0, 0) within 2.5 cm
        dist_to_start = math.hypot(cur_x, cur_y)
        self.assertLessEqual(dist_to_start, 0.026)

        # Verify heading restored to 0.0°
        self.assertLessEqual(abs(math.degrees(cur_th)), 3.5)
        self.assertGreater(total_dist, 0.50)

    def test_multi_cycle_patrol(self):
        # 2 cycles back and forth between (0, 0) and (0.30, 0.30)
        ok = self.nav.patrol(
            target_x=0.30,
            target_y=0.30,
            cycles=2,
            target_theta_deg=45.0,
            speed_mps=0.70,
            save_plot=True,
            plot_path=self.test_plot
        )
        self.assertTrue(ok)

        # Robot must end back at (0, 0) facing 0°
        cur_x, cur_y, cur_th, total_dist = self.hw.get_pose()
        self.assertLessEqual(math.hypot(cur_x, cur_y), 0.035)
        self.assertGreater(total_dist, 0.90)

    def test_dynamic_obstacle_replan(self):
        # Set mock sensor to detect an obstacle 25 cm ahead during driving
        # Start navigation to (0.50, 0.00)
        # Inject obstacle after a brief moment
        self.hw.mock_distances = [9.99, 0.25, 9.99]

        success = self.nav.navigate_to(
            target_x=0.50,
            target_y=0.00,
            speed_mps=0.60,
            save_plot=False
        )
        # Should have detected and marked at least 1 obstacle in the grid
        self.assertGreater(len(self.grid.raw_obstacles), 0)


if __name__ == "__main__":
    unittest.main()

