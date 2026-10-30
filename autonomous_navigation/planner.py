"""
autonomous_navigation/planner.py
---------------------------------
Global A* grid path planning and local Artificial Potential Field (APF)
reactive vector avoidance.
Implemented with zero external dependencies (pure Python math) for maximum portability.
"""

import math
import heapq

try:
    from .config import (
        GRID_SIZE_X, GRID_SIZE_Y, GRID_RES,
        ORIGIN_OFFSET_X, ORIGIN_OFFSET_Y,
        ROBOT_CLEARANCE_RADIUS, CELLS_TO_INFLATE,
        K_ATT, K_REP, MAX_REP_FORCE, SAFE_DIST, SENSOR_ANGLES,
        PIVOT_ENTER_RAD, PIVOT_EXIT_RAD,
        WHEEL_BASE_M, MAX_ALLOWED_SPEED, MIN_MANEUVER_SPEED
    )
except ImportError:
    from config import (
        GRID_SIZE_X, GRID_SIZE_Y, GRID_RES,
        ORIGIN_OFFSET_X, ORIGIN_OFFSET_Y,
        ROBOT_CLEARANCE_RADIUS, CELLS_TO_INFLATE,
        K_ATT, K_REP, MAX_REP_FORCE, SAFE_DIST, SENSOR_ANGLES,
        PIVOT_ENTER_RAD, PIVOT_EXIT_RAD,
        WHEEL_BASE_M, MAX_ALLOWED_SPEED, MIN_MANEUVER_SPEED
    )

GRID_DIM_X = int(GRID_SIZE_X / GRID_RES)
GRID_DIM_Y = int(GRID_SIZE_Y / GRID_RES)


def create_empty_grid():
    """Creates a 2D occupancy grid initialized with boundary walls (1 = occupied, 0 = free)."""
    grid = [[0] * GRID_DIM_Y for _ in range(GRID_DIM_X)]
    # Set outer boundaries as solid walls
    for x in range(GRID_DIM_X):
        grid[x][0] = 1
        grid[x][GRID_DIM_Y - 1] = 1
    for y in range(GRID_DIM_Y):
        grid[0][y] = 1
        grid[GRID_DIM_X - 1][y] = 1
    return grid


def world_to_grid(x, y):
    """
    Converts continuous robot frame coordinates (meters) to discrete grid indices.
    Applies ORIGIN_OFFSET so robot starts near (ORIGIN_OFFSET_X, ORIGIN_OFFSET_Y).
    """
    gx = int(round((x + ORIGIN_OFFSET_X) / GRID_RES))
    gy = int(round((y + ORIGIN_OFFSET_Y) / GRID_RES))
    gx = max(0, min(GRID_DIM_X - 1, gx))
    gy = max(0, min(GRID_DIM_Y - 1, gy))
    return (gx, gy)


def grid_to_world(gx, gy):
    """Converts discrete grid indices back to continuous world coordinates (meters)."""
    x = (gx * GRID_RES) - ORIGIN_OFFSET_X
    y = (gy * GRID_RES) - ORIGIN_OFFSET_Y
    return (x, y)


def mark_obstacle(grid, world_x, world_y):
    """Marks an obstacle at (world_x, world_y) and inflates it by the robot clearance radius."""
    gx, gy = world_to_grid(world_x, world_y)

    for i in range(-CELLS_TO_INFLATE, CELLS_TO_INFLATE + 1):
        for j in range(-CELLS_TO_INFLATE, CELLS_TO_INFLATE + 1):
            nx, ny = gx + i, gy + j
            if 0 <= nx < GRID_DIM_X and 0 <= ny < GRID_DIM_Y:
                if math.hypot(i, j) <= CELLS_TO_INFLATE:
                    grid[nx][ny] = 1


def astar_grid(start_world, goal_world, grid):
    """
    Finds the shortest collision-free path on the occupancy grid using A*.
    Returns a list of continuous waypoints [(x1, y1), (x2, y2), ...].
    """
    start = world_to_grid(*start_world)
    goal = world_to_grid(*goal_world)

    if grid[goal[0]][goal[1]] == 1:
        # If exact goal cell is marked or on boundary, find nearest free neighbor
        found = False
        for r in range(1, 15):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    cand = (goal[0] + dx, goal[1] + dy)
                    if 0 <= cand[0] < GRID_DIM_X and 0 <= cand[1] < GRID_DIM_Y:
                        if grid[cand[0]][cand[1]] == 0:
                            goal = cand
                            found = True
                            break
                if found:
                    break
            if found:
                break
        if not found:
            return []

    open_set = []
    counter = 0
    heapq.heappush(open_set, (0.0, counter, start))

    came_from = {}
    g_score = {start: 0.0}

    # 8-directional movement: (dx, dy, cost)
    directions = [
        (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
        (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)
    ]

    while open_set:
        current = heapq.heappop(open_set)[2]

        if current == goal:
            path = []
            while current in came_from:
                path.append(grid_to_world(*current))
                current = came_from[current]
            path.append(grid_to_world(*start))
            return path[::-1]

        for dx, dy, cost in directions:
            neighbor = (current[0] + dx, current[1] + dy)

            if 0 <= neighbor[0] < GRID_DIM_X and 0 <= neighbor[1] < GRID_DIM_Y:
                if grid[neighbor[0]][neighbor[1]] == 1:
                    continue

                tentative_g = g_score[current] + cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    h_score = math.hypot(goal[0] - neighbor[0], goal[1] - neighbor[1])
                    f_score = tentative_g + h_score

                    counter += 1
                    heapq.heappush(open_set, (f_score, counter, neighbor))

    return []  # No path found


def is_path_blocked(waypoints, grid):
    """Returns True if any waypoint along the current path is blocked by newly mapped obstacles."""
    for wp in waypoints:
        gx, gy = world_to_grid(wp[0], wp[1])
        if grid[gx][gy] == 1:
            return True
    return False


def calculate_apf(current_pos, waypoint, tof_readings, current_theta, target_speed):
    """
    Computes resultant force vector from Waypoint Attraction (F_att) and
    real-time Time-of-Flight Repulsion (F_rep).
    """
    # Attractive force toward current waypoint
    diff_x = waypoint[0] - current_pos[0]
    diff_y = waypoint[1] - current_pos[1]
    dist_to_wp = math.hypot(diff_x, diff_y)

    if dist_to_wp > 0.001:
        scale = (target_speed * K_ATT) / max(0.05, dist_to_wp)
        f_att_x = diff_x * scale
        f_att_y = diff_y * scale
    else:
        f_att_x = 0.0
        f_att_y = 0.0

    # Repulsive forces from active ToF sensor readings
    f_rep_x = 0.0
    f_rep_y = 0.0

    num_sensors = min(len(tof_readings), len(SENSOR_ANGLES))
    for i in range(num_sensors):
        distance = tof_readings[i]
        # Ignore floor glare / extreme close (< 0.05m) and out-of-range (> SAFE_DIST)
        if 0.05 < distance < SAFE_DIST:
            # Inverse-square repulsion magnitude with saturation cap to prevent spikes
            raw_rep = K_REP * ((1.0 / distance) - (1.0 / SAFE_DIST)) / (distance ** 2)
            rep_mag = min(MAX_REP_FORCE, raw_rep)

            # Push away from the obstacle angle
            push_angle = current_theta + SENSOR_ANGLES[i] + math.pi
            f_rep_x += rep_mag * math.cos(push_angle)
            f_rep_y += rep_mag * math.sin(push_angle)

    total_fx = f_att_x + f_rep_x
    total_fy = f_att_y + f_rep_y
    return (total_fx, total_fy)


def calculate_kinematics(f_total, current_theta, target_speed, is_pivoting=False):
    """
    Converts resultant navigation force vector into differential-drive
    wheel speeds (v_left, v_right) in meters per second.
    Includes in-place pivot with hysteresis:
      - Enter pivot when |heading_error| > PIVOT_ENTER_RAD (~49°)
      - Exit pivot only when |heading_error| < PIVOT_EXIT_RAD (~20°)
    """
    norm = math.hypot(f_total[0], f_total[1])
    if norm < 0.01:
        return 0.0, 0.0

    # Desired heading from vector
    desired_theta = math.atan2(f_total[1], f_total[0])
    heading_error = math.atan2(
        math.sin(desired_theta - current_theta),
        math.cos(desired_theta - current_theta)
    )

    # 1. Hysteresis pivot check: prevents hunting oscillation at boundary
    should_pivot = (abs(heading_error) > PIVOT_ENTER_RAD) or (is_pivoting and abs(heading_error) > PIVOT_EXIT_RAD)

    if should_pivot:
        # Smooth pivot speed easing: slows down gently as robot aligns to exit angle
        err_mag = abs(heading_error)
        norm_err = min(1.0, max(0.0, (err_mag - PIVOT_EXIT_RAD) / max(0.01, PIVOT_ENTER_RAD - PIVOT_EXIT_RAD)))
        pivot_speed = MIN_MANEUVER_SPEED + (0.26 - MIN_MANEUVER_SPEED) * (norm_err ** 1.2)
        if heading_error > 0:
            # Turn Left: Left wheel backward, Right wheel forward
            return -pivot_speed, +pivot_speed
        else:
            # Turn Right: Left wheel forward, Right wheel backward
            return +pivot_speed, -pivot_speed

    # 2. Smooth forward drive with differential curvature
    turn_factor = max(0.25, math.cos(heading_error))
    linear_v = min(target_speed, norm) * turn_factor
    linear_v = max(MIN_MANEUVER_SPEED, linear_v)

    # Angular velocity proportional to heading error
    omega = 2.0 * heading_error

    # Differential drive kinematics
    v_left = linear_v - (omega * WHEEL_BASE_M / 2.0)
    v_right = linear_v + (omega * WHEEL_BASE_M / 2.0)

    # Clamp to physical speed limits
    v_left = max(-MAX_ALLOWED_SPEED, min(MAX_ALLOWED_SPEED, v_left))
    v_right = max(-MAX_ALLOWED_SPEED, min(MAX_ALLOWED_SPEED, v_right))

    return v_left, v_right
