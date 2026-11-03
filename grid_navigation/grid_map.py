"""
grid_navigation/grid_map.py
---------------------------
2D Discrete Occupancy Grid Map with circular safety inflation.
"""

import math
from typing import Tuple, List, Set

try:
    from .config import (
        GRID_SIZE_X_M, GRID_SIZE_Y_M, GRID_RES_M,
        ORIGIN_OFFSET_X, ORIGIN_OFFSET_Y,
        GRID_DIM_X, GRID_DIM_Y,
        ROBOT_RADIUS_M, INFLATION_CELLS
    )
except ImportError:
    from config import (
        GRID_SIZE_X_M, GRID_SIZE_Y_M, GRID_RES_M,
        ORIGIN_OFFSET_X, ORIGIN_OFFSET_Y,
        GRID_DIM_X, GRID_DIM_Y,
        ROBOT_RADIUS_M, INFLATION_CELLS
    )


class OccupancyGridMap:
    """
    2D Discrete Occupancy Grid centered at (0, 0) with circular obstacle inflation.
    Grid cells are 0 (free) or 1 (occupied / inflated buffer).
    """

    def __init__(
        self,
        dim_x: int = GRID_DIM_X,
        dim_y: int = GRID_DIM_Y,
        resolution_m: float = GRID_RES_M,
        offset_x_m: float = ORIGIN_OFFSET_X,
        offset_y_m: float = ORIGIN_OFFSET_Y,
        robot_radius_m: float = ROBOT_RADIUS_M,
    ):
        self.dim_x = dim_x
        self.dim_y = dim_y
        self.resolution = resolution_m
        self.offset_x = offset_x_m
        self.offset_y = offset_y_m
        self.robot_radius = robot_radius_m
        self.inflation_radius_cells = int(math.ceil(robot_radius_m / resolution_m))

        # 2D grid: 0 = free, 1 = occupied/inflated
        self.grid = [[0 for _ in range(self.dim_y)] for _ in range(self.dim_x)]
        # Track raw uninflated obstacle centers as set of (gx, gy)
        self.raw_obstacles: Set[Tuple[int, int]] = set()

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Converts metric world coordinates (x, y) to discrete grid cell (gx, gy)."""
        gx = int(round((x + self.offset_x) / self.resolution))
        gy = int(round((y + self.offset_y) / self.resolution))
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        """Converts discrete grid cell (gx, gy) to metric world coordinates (x, y)."""
        x = round((gx * self.resolution) - self.offset_x, 3)
        y = round((gy * self.resolution) - self.offset_y, 3)
        return x, y

    def is_in_bounds(self, gx: int, gy: int) -> bool:
        """Returns True if (gx, gy) is within the grid boundaries."""
        return 0 <= gx < self.dim_x and 0 <= gy < self.dim_y

    def is_free(self, gx: int, gy: int) -> bool:
        """Returns True if the cell is within bounds and free of obstacles/inflation."""
        return self.is_in_bounds(gx, gy) and self.grid[gx][gy] == 0

    def is_world_free(self, x: float, y: float) -> bool:
        """Returns True if the metric world coordinate is in bounds and free."""
        gx, gy = self.world_to_grid(x, y)
        return self.is_free(gx, gy)

    def add_obstacle(self, world_x: float, world_y: float) -> bool:
        """
        Marks an obstacle at (world_x, world_y) and inflates surrounding cells
        within the robot clearance radius.
        """
        gx, gy = self.world_to_grid(world_x, world_y)
        if not self.is_in_bounds(gx, gy):
            return False

        if (gx, gy) in self.raw_obstacles:
            return False  # Already marked

        self.raw_obstacles.add((gx, gy))
        self._inflate_cell(gx, gy, value=1)
        return True

    def remove_obstacle(self, world_x: float, world_y: float) -> bool:
        """Removes an obstacle and re-computes the inflation grid."""
        gx, gy = self.world_to_grid(world_x, world_y)
        if (gx, gy) not in self.raw_obstacles:
            return False

        self.raw_obstacles.remove((gx, gy))
        self._rebuild_grid()
        return True

    def clear(self) -> None:
        """Clears all obstacles and resets the grid to completely free."""
        self.raw_obstacles.clear()
        self.grid = [[0 for _ in range(self.dim_y)] for _ in range(self.dim_x)]

    def get_raw_obstacles_world(self) -> List[Tuple[float, float]]:
        """Returns list of metric (x, y) coordinates of all confirmed obstacles."""
        return [self.grid_to_world(gx, gy) for gx, gy in self.raw_obstacles]

    def _inflate_cell(self, center_gx: int, center_gy: int, value: int = 1) -> None:
        """Circular Euclidean inflation around (center_gx, center_gy)."""
        r = self.inflation_radius_cells
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                if (dx * dx + dy * dy) <= (r * r):
                    nx = center_gx + dx
                    ny = center_gy + dy
                    if self.is_in_bounds(nx, ny):
                        self.grid[nx][ny] = value

    def _rebuild_grid(self) -> None:
        """Reconstructs the full inflation grid from active raw obstacles."""
        self.grid = [[0 for _ in range(self.dim_y)] for _ in range(self.dim_x)]
        for gx, gy in self.raw_obstacles:
            self._inflate_cell(gx, gy, value=1)

