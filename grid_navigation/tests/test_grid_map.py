"""
Unit tests for grid_navigation/grid_map.py
"""

import unittest
from grid_navigation.grid_map import OccupancyGridMap


class TestOccupancyGridMap(unittest.TestCase):
    def setUp(self):
        # 10m x 10m grid, 0.1m resolution, (0,0) at (50, 50)
        self.gmap = OccupancyGridMap(
            dim_x=100,
            dim_y=100,
            resolution_m=0.10,
            offset_x_m=5.0,
            offset_y_m=5.0,
            robot_radius_m=0.14
        )

    def test_coordinate_transforms(self):
        # Origin (0.0, 0.0) -> (50, 50)
        gx, gy = self.gmap.world_to_grid(0.0, 0.0)
        self.assertEqual((gx, gy), (50, 50))

        # (50, 50) -> (0.0, 0.0)
        wx, wy = self.gmap.grid_to_world(50, 50)
        self.assertAlmostEqual(wx, 0.0, places=2)
        self.assertAlmostEqual(wy, 0.0, places=2)

        # Coordinate (1.5, -2.0)
        gx, gy = self.gmap.world_to_grid(1.5, -2.0)
        self.assertEqual((gx, gy), (65, 30))
        wx, wy = self.gmap.grid_to_world(gx, gy)
        self.assertAlmostEqual(wx, 1.5, places=2)
        self.assertAlmostEqual(wy, -2.0, places=2)

    def test_obstacle_addition_and_inflation(self):
        # Origin (0, 0) should initially be free
        self.assertTrue(self.gmap.is_world_free(0.0, 0.0))

        # Add obstacle at (1.0, 1.0)
        added = self.gmap.add_obstacle(1.0, 1.0)
        self.assertTrue(added)
        self.assertFalse(self.gmap.is_world_free(1.0, 1.0))

        # Check circular inflation radius (radius 0.14m / 0.10m = 2 cells)
        # Point 10 cm away should be blocked due to inflation
        self.assertFalse(self.gmap.is_world_free(1.10, 1.0))
        # Point 30 cm away should be free
        self.assertTrue(self.gmap.is_world_free(1.30, 1.0))

    def test_obstacle_removal(self):
        self.gmap.add_obstacle(0.5, 0.5)
        self.assertFalse(self.gmap.is_world_free(0.5, 0.5))

        # Remove obstacle
        removed = self.gmap.remove_obstacle(0.5, 0.5)
        self.assertTrue(removed)
        # Cell and surrounding inflation should be free again
        self.assertTrue(self.gmap.is_world_free(0.5, 0.5))
        self.assertTrue(self.gmap.is_world_free(0.55, 0.5))

    def test_clear_grid(self):
        self.gmap.add_obstacle(1.0, 1.0)
        self.gmap.add_obstacle(-1.0, -1.0)
        self.assertEqual(len(self.gmap.raw_obstacles), 2)

        self.gmap.clear()
        self.assertEqual(len(self.gmap.raw_obstacles), 0)
        self.assertTrue(self.gmap.is_world_free(1.0, 1.0))
        self.assertTrue(self.gmap.is_world_free(-1.0, -1.0))


if __name__ == "__main__":
    unittest.main()

