"""
Unit tests for grid_navigation/planner.py
"""

import unittest
from grid_navigation.grid_map import OccupancyGridMap
from grid_navigation.planner import astar_search, has_line_of_sight, smooth_path


class TestPlanner(unittest.TestCase):
    def setUp(self):
        # 10m x 10m grid, 0.1m resolution, origin (0,0) at (50, 50)
        self.gmap = OccupancyGridMap(
            dim_x=100,
            dim_y=100,
            resolution_m=0.10,
            offset_x_m=5.0,
            offset_y_m=5.0,
            robot_radius_m=0.14
        )

    def test_direct_path_on_empty_grid(self):
        start = (0.0, 0.0)
        goal = (2.0, 1.0)
        path = astar_search(start, goal, self.gmap, smooth=True)

        self.assertGreater(len(path), 0)
        # Smoothed path on an open field should directly connect start to goal
        self.assertEqual(len(path), 2)
        self.assertEqual(path[0], start)
        self.assertEqual(path[-1], goal)

    def test_astar_navigates_around_obstacle(self):
        start = (0.0, 0.0)
        goal = (2.0, 0.0)

        # Place an obstacle directly in the center of the line at (1.0, 0.0)
        self.gmap.add_obstacle(1.0, 0.0)

        path = astar_search(start, goal, self.gmap, smooth=True)
        self.assertGreater(len(path), 2)

        # None of the waypoints should collide with the obstacle or its inflation
        obs_gx, obs_gy = self.gmap.world_to_grid(1.0, 0.0)
        for pt in path:
            pgx, pgy = self.gmap.world_to_grid(pt[0], pt[1])
            self.assertTrue(self.gmap.is_free(pgx, pgy))

    def test_unreachable_goal_returns_empty(self):
        # Enclose goal (1.0, 1.0) with walls
        for dx in [-0.3, 0.0, 0.3]:
            for dy in [-0.3, 0.0, 0.3]:
                if dx == 0.0 and dy == 0.0:
                    continue
                self.gmap.add_obstacle(1.0 + dx, 1.0 + dy)

        path = astar_search((0.0, 0.0), (1.0, 1.0), self.gmap)
        self.assertEqual(path, [])

    def test_line_of_sight(self):
        # Clear line between (10, 10) and (20, 20)
        self.assertTrue(has_line_of_sight((10, 10), (20, 20), self.gmap))

        # Block an intermediate cell (15, 15)
        self.gmap.grid[15][15] = 1
        self.assertFalse(has_line_of_sight((10, 10), (20, 20), self.gmap))


if __name__ == "__main__":
    unittest.main()

