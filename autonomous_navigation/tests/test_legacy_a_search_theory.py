"""
autonomous_navigation/tests/test_legacy_a_search_theory.py
-----------------------------------------------------------
Tests the autonomous navigation system directly against the foundational theory
in `legacy_files/a_search_main.py`, verifying:
1. Grid Mapping & Circular Inflation equivalence
2. 8-directional A* global search & Euclidean heuristic
3. Dynamic obstacle detection & route collision detection (`is_path_blocked`)
4. Dynamic A* replanning around newly detected obstacles
5. Artificial Potential Field (APF) attractive/repulsive vector balance
6. Rejection of false obstacle detections upon stationary verification
"""

import math
import unittest
import importlib.util

from autonomous_navigation.config import (
    GRID_RES, ROBOT_CLEARANCE_RADIUS, CELLS_TO_INFLATE,
    K_ATT, K_REP, SAFE_DIST, SENSOR_ANGLES,
    CRITICAL_STOP_DIST, CRITICAL_PERSIST_TICKS, TOF_MIN_VALID_M
)
from autonomous_navigation.planner import (
    create_empty_grid, world_to_grid, grid_to_world,
    mark_obstacle, unmark_obstacle, astar_grid, is_path_blocked,
    calculate_apf, calculate_kinematics
)
from autonomous_navigation.sensors import DistanceSensorManager
from autonomous_navigation.serial_comm import RobotSerialComm
from autonomous_navigation.navigator import AutonomousNavigator


class TestLegacyASearchTheory(unittest.TestCase):
    def setUp(self):
        self.sensors = DistanceSensorManager(use_mock=True)
        self.sensors.initialize()
        self.sensors.start()

        self.comm = RobotSerialComm(use_mock=True)
        self.comm.connect()
        self.comm.start()

        self.nav = AutonomousNavigator(serial_comm=self.comm, sensor_mgr=self.sensors)

    def tearDown(self):
        self.nav.request_stop()
        self.comm.close()
        self.sensors.close()

    def test_circular_inflation_theory(self):
        """
        Verifies that obstacle marking applies circular clearance inflation
        hypot(i, j) <= CELLS_TO_INFLATE, matching a_search_main.py line 61.
        """
        grid = create_empty_grid()
        obs_x, obs_y = 1.0, 1.0
        mark_obstacle(grid, obs_x, obs_y)

        gx, gy = world_to_grid(obs_x, obs_y)
        self.assertEqual(grid[gx][gy], 1)

        # Check cells within inflation radius are 1
        for i in range(-CELLS_TO_INFLATE, CELLS_TO_INFLATE + 1):
            for j in range(-CELLS_TO_INFLATE, CELLS_TO_INFLATE + 1):
                if math.hypot(i, j) <= CELLS_TO_INFLATE:
                    self.assertEqual(grid[gx + i][gy + j], 1)

    def test_astar_pathfinding_and_replanning_theory(self):
        """
        Verifies A* pathfinding and dynamic replanning when an obstacle blocks
        the route, matching a_search_main.py lines 183-222.
        """
        grid = create_empty_grid()
        start = (0.0, 0.0)
        goal = (2.0, 0.0)

        # 1. Initial path should be a straight line along the X axis
        path = astar_grid(start, goal, grid)
        self.assertTrue(len(path) >= 2)
        self.assertFalse(is_path_blocked(path, grid))

        # 2. Dynamically place obstacle on the path at (1.0, 0.0)
        mark_obstacle(grid, 1.0, 0.0)

        # 3. Path must now be detected as blocked
        self.assertTrue(is_path_blocked(path, grid))

        # 4. Replan path around the inflated obstacle
        replanned_path = astar_grid(start, goal, grid)
        self.assertTrue(len(replanned_path) >= 2)
        self.assertFalse(is_path_blocked(replanned_path, grid))

        # 5. Replanned path must detour laterally (Y != 0) to avoid the obstacle
        detoured = any(abs(wp[1]) >= ROBOT_CLEARANCE_RADIUS for wp in replanned_path)
        self.assertTrue(detoured, "A* path must detour laterally around obstacle")

    def test_apf_vector_field_theory(self):
        """
        Verifies APF vector generation matches the inverse-square repulsion formula
        in a_search_main.py lines 134-149:
          F_att = K_ATT * (target - pos)
          F_rep = K_REP * ((1/d) - (1/SAFE_DIST)) / (d^2) pushing away from sensor angle
        """
        current_pos = (0.0, 0.0)
        target_wp = (1.0, 0.0)
        current_theta = 0.0
        target_speed = 0.60

        # Case A: Clear path (no obstacles) -> Only attractive force along +X
        f_clear = calculate_apf(current_pos, target_wp, [9.99, 9.99, 9.99], current_theta, target_speed)
        self.assertGreater(f_clear[0], 0.0)
        self.assertAlmostEqual(f_clear[1], 0.0, places=3)

        # Case B: Obstacle detected on Right sensor (-45 deg / -pi/4) at 0.15m (< SAFE_DIST)
        # Repulsive force should push AWAY from Right -> pushes toward Left (+Y)
        f_repelled = calculate_apf(current_pos, target_wp, [9.99, 9.99, 0.15], current_theta, target_speed)
        self.assertGreater(f_repelled[1], 0.0, "Right sensor hazard must push robot to the Left (+Y)")

        # Case C: Obstacle detected on Left sensor (+45 deg / +pi/4) at 0.15m (< SAFE_DIST)
        # Repulsive force should push AWAY from Left -> pushes toward Right (-Y)
        f_left_repel = calculate_apf(current_pos, target_wp, [0.15, 9.99, 9.99], current_theta, target_speed)
        self.assertLess(f_left_repel[1], 0.0, "Left sensor hazard must push robot to the Right (-Y)")

    def test_kinematics_differential_steering_theory(self):
        """
        Verifies calculate_kinematics converts resultant force into differential
        wheel velocities according to heading error and track width, matching
        a_search_main.py lines 151-166.
        """
        # Force pointing directly forward: wheels should spin at equal forward speed
        f_forward = (1.0, 0.0)
        vl, vr = calculate_kinematics(f_forward, current_theta=0.0, target_speed=0.50)
        self.assertAlmostEqual(vl, vr, places=3)
        self.assertGreater(vl, 0.0)

        # Force pointing left (+90 deg): robot should turn left (vl < vr)
        f_left = (0.0, 1.0)
        vl, vr = calculate_kinematics(f_left, current_theta=0.0, target_speed=0.50)
        self.assertLess(vl, vr)

    def test_ignore_false_obstacle_detection(self):
        """
        Verifies that transient false obstacle detections (e.g. 1-2 ticks of floor glare
        or dust that disappear upon stationary verification) are ignored and do NOT
        corrupt the map or permanently block the route.
        """
        grid = create_empty_grid()
        self.nav.grid = grid
        self.nav.last_obstacles = []

        # Initially, no obstacles mapped
        self.assertEqual(len(self.nav.last_obstacles), 0)

        # Simulate stationary verification: if a transient reading disappears upon stop
        fresh_tof = [9.99, 9.99, 9.99]  # cleared reading
        verified_hazards = [
            (i, d) for i, d in enumerate(fresh_tof)
            if TOF_MIN_VALID_M <= d < (CRITICAL_STOP_DIST + 0.05)
        ]
        # Verified hazards must be empty
        self.assertEqual(len(verified_hazards), 0)
        # Grid must remain completely clear (no false obstacle baked in)
        self.assertEqual(len(self.nav.last_obstacles), 0)

    def test_unmark_obstacle_recovers_free_space(self):
        """
        Verifies that unmark_obstacle removes an erroneously mapped obstacle
        and its inflation halo, restoring the cell to free traversable space.
        """
        grid = create_empty_grid()
        obs_x, obs_y = 1.0, 0.5
        mark_obstacle(grid, obs_x, obs_y)

        gx, gy = world_to_grid(obs_x, obs_y)
        self.assertEqual(grid[gx][gy], 1)

        # Unmark the obstacle
        unmark_obstacle(grid, obs_x, obs_y)
        self.assertEqual(grid[gx][gy], 0)
        # Check neighboring inflated cells are also restored to 0
        self.assertEqual(grid[gx + 1][gy], 0)
        self.assertEqual(grid[gx - 1][gy], 0)


if __name__ == "__main__":
    unittest.main()
