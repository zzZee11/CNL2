"""
grid_navigation/planner.py
--------------------------
A* 8-connected grid path planning with line-of-sight path smoothing.
"""

import math
import heapq
from typing import List, Tuple, Optional

try:
    from .grid_map import OccupancyGridMap
except ImportError:
    from grid_map import OccupancyGridMap


# 8-Directional Motion Neighbors with exact step costs
# (dx, dy, cost)
MOTIONS = [
    (1, 0, 1.0),
    (0, 1, 1.0),
    (-1, 0, 1.0),
    (0, -1, 1.0),
    (1, 1, math.sqrt(2)),
    (1, -1, math.sqrt(2)),
    (-1, 1, math.sqrt(2)),
    (-1, -1, math.sqrt(2)),
]


def astar_search(
    start_world: Tuple[float, float],
    goal_world: Tuple[float, float],
    grid_map: OccupancyGridMap,
    smooth: bool = True
) -> List[Tuple[float, float]]:
    """
    Computes an optimal collision-free path from start_world to goal_world on grid_map.
    Optionally applies line-of-sight smoothing to produce clean straight-line waypoints.

    Returns:
        List of (x, y) coordinates in meters from start to goal. Returns empty list if no path exists.
    """
    start_gx, start_gy = grid_map.world_to_grid(start_world[0], start_world[1])
    goal_gx, goal_gy = grid_map.world_to_grid(goal_world[0], goal_world[1])

    # Ensure start and goal are in bounds
    if not grid_map.is_in_bounds(start_gx, start_gy) or not grid_map.is_in_bounds(goal_gx, goal_gy):
        return []

    # If start or goal is blocked, return empty
    if not grid_map.is_free(start_gx, start_gy) or not grid_map.is_free(goal_gx, goal_gy):
        return []

    start_node = (start_gx, start_gy)
    goal_node = (goal_gx, goal_gy)

    if start_node == goal_node:
        return [start_world, goal_world]

    counter = 0
    # Priority queue stores (f_score, counter, node)
    open_set = []
    came_from = {}
    g_score = {start_node: 0.0}

    h_start = math.hypot(goal_gx - start_gx, goal_gy - start_gy)
    heapq.heappush(open_set, (h_start, counter, start_node))

    closed_set = set()

    while open_set:
        _, _, current = heapq.heappop(open_set)

        if current == goal_node:
            # Reconstruct path
            grid_path = [current]
            while current in came_from:
                current = came_from[current]
                grid_path.append(current)
            grid_path.reverse()

            if smooth:
                grid_path = smooth_path(grid_path, grid_map)

            # Convert to world coordinates
            world_path = [grid_map.grid_to_world(gx, gy) for gx, gy in grid_path]
            # Pin the exact start and goal floats
            world_path[0] = (start_world[0], start_world[1])
            world_path[-1] = (goal_world[0], goal_world[1])
            return world_path

        if current in closed_set:
            continue
        closed_set.add(current)

        cx, cy = current
        current_g = g_score[current]

        for dx, dy, step_cost in MOTIONS:
            nx = cx + dx
            ny = cy + dy
            neighbor = (nx, ny)

            if not grid_map.is_free(nx, ny):
                continue

            # Prevent cutting through diagonal obstacle cracks
            if dx != 0 and dy != 0:
                if not grid_map.is_free(cx + dx, cy) and not grid_map.is_free(cx, cy + dy):
                    continue

            tentative_g = current_g + step_cost

            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                h = math.hypot(goal_gx - nx, goal_gy - ny)
                f = tentative_g + h

                counter += 1
                heapq.heappush(open_set, (f, counter, neighbor))

    return []  # No path found


def has_line_of_sight(p1: Tuple[int, int], p2: Tuple[int, int], grid_map: OccupancyGridMap) -> bool:
    """
    Bresenham ray-casting line-of-sight check between two grid cells.
    Returns True if the direct line between p1 and p2 contains only free cells.
    """
    x0, y0 = p1
    x1, y1 = p2

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    x, y = x0, y0

    while True:
        if not grid_map.is_free(x, y):
            return False

        if x == x1 and y == y1:
            break

        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy

    return True


def smooth_path(grid_path: List[Tuple[int, int]], grid_map: OccupancyGridMap) -> List[Tuple[int, int]]:
    """
    Reduces dense grid steps into clean straight line segments using greedy line-of-sight string pulling.
    """
    if len(grid_path) <= 2:
        return grid_path

    smoothed = [grid_path[0]]
    current_idx = 0

    while current_idx < len(grid_path) - 1:
        # Check furthest visible waypoint along the path
        furthest_visible = current_idx + 1
        for candidate_idx in range(len(grid_path) - 1, current_idx + 1, -1):
            if has_line_of_sight(grid_path[current_idx], grid_path[candidate_idx], grid_map):
                furthest_visible = candidate_idx
                break

        smoothed.append(grid_path[furthest_visible])
        current_idx = furthest_visible

    return smoothed

