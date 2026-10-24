import json
import math
import heapq
import time
import numpy as np

# ==========================================
# CONFIGURATION CONSTANTS
# ==========================================
# Warehouse & Grid
WAREHOUSE_SIZE = 10.0      # 10x10 meters
GRID_RES = 0.1             # 10 cm per grid cell (100x100 matrix)
GRID_DIM = int(WAREHOUSE_SIZE / GRID_RES)

# Robot Physical Specs
ROBOT_RADIUS = 0.25        # 25 cm clearance radius
WHEEL_BASE = 0.2           # 20 cm between wheels
MAX_SPEED = 1.0            # Max linear velocity (m/s)
CELLS_TO_INFLATE = int(math.ceil(ROBOT_RADIUS / GRID_RES)) # Cells to pad around obstacles

# APF (Vector) Tuning
K_ATT = 1.2                # Pull strength to waypoint
K_REP = 0.8                # Push strength from obstacles
SAFE_DIST = 0.4            # Distance (meters) to start avoiding
SENSOR_ANGLES = [math.pi/4, 0.0, -math.pi/4] # Left (+45deg), Center (0), Right (-45deg)

# ==========================================
# 1. GRID MAPPING & CONVERSIONS
# ==========================================
def create_empty_grid():
    """Creates a grid initialized with 0 (free space) and 1 (boundary walls)."""
    grid = np.zeros((GRID_DIM, GRID_DIM), dtype=int)
    # Set outer boundaries as walls
    grid[0, :] = 1; grid[-1, :] = 1
    grid[:, 0] = 1; grid[:, -1] = 1
    return grid

def world_to_grid(x, y):
    """Converts continuous world coordinates (meters) to discrete grid indices."""
    gx = int(max(0, min(GRID_DIM - 1, x / GRID_RES)))
    gy = int(max(0, min(GRID_DIM - 1, y / GRID_RES)))
    return (gx, gy)

def grid_to_world(gx, gy):
    """Converts grid indices back to world coordinates (center of the cell)."""
    x = (gx * GRID_RES) + (GRID_RES / 2.0)
    y = (gy * GRID_RES) + (GRID_RES / 2.0)
    return (x, y)

def mark_obstacle(grid, world_x, world_y):
    """Marks a detected obstacle on the grid and inflates it for robot clearance."""
    gx, gy = world_to_grid(world_x, world_y)
    
    # Inflate the obstacle by the robot's radius
    for i in range(-CELLS_TO_INFLATE, CELLS_TO_INFLATE + 1):
        for j in range(-CELLS_TO_INFLATE, CELLS_TO_INFLATE + 1):
            nx, ny = gx + i, gy + j
            # Check bounds
            if 0 <= nx < GRID_DIM and 0 <= ny < GRID_DIM:
                # Circular inflation math
                if math.hypot(i, j) <= CELLS_TO_INFLATE:
                    grid[nx][ny] = 1

# ==========================================
# 2. GLOBAL PLANNER (Grid A*)
# ==========================================
def astar_grid(start_world, goal_world, grid):
    """Finds the shortest path on the occupancy grid."""
    start = world_to_grid(*start_world)
    goal = world_to_grid(*goal_world)
    
    open_set = []
    counter = 0
    heapq.heappush(open_set, (0, counter, start))
    
    came_from = {}
    g_score = {start: 0}
    
    # 8-directional movement (horizontal/vertical cost 1, diagonal cost 1.414)
    directions = [(0, 1, 1), (0, -1, 1), (1, 0, 1), (-1, 0, 1),
                  (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]
    
    while open_set:
        current = heapq.heappop(open_set)[2]
        
        if current == goal:
            # Reconstruct path and convert back to world coordinates
            path = []
            while current in came_from:
                path.append(grid_to_world(*current))
                current = came_from[current]
            path.append(grid_to_world(*start))
            return path[::-1]
            
        for dx, dy, cost in directions:
            neighbor = (current[0] + dx, current[1] + dy)
            
            # Check bounds and obstacles
            if 0 <= neighbor[0] < GRID_DIM and 0 <= neighbor[1] < GRID_DIM:
                if grid[neighbor[0]][neighbor[1]] == 1:
                    continue # Hit an obstacle
                    
                tentative_g = g_score[current] + cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    # Heuristic: Euclidean distance to goal in grid units
                    h_score = math.hypot(goal[0] - neighbor[0], goal[1] - neighbor[1])
                    f_score = tentative_g + h_score
                    
                    counter += 1
                    heapq.heappush(open_set, (f_score, counter, neighbor))
                    
    return [] # Path blocked

def is_path_blocked(waypoints, grid):
    """Checks if any upcoming waypoint has been mapped as an obstacle."""
    for wp in waypoints:
        gx, gy = world_to_grid(wp[0], wp[1])
        if grid[gx][gy] == 1:
            return True
    return False

# ==========================================
# 3. LOCAL PLANNER (APF Vectors & Kinematics)
# ==========================================
def calculate_apf(current_pos, waypoint, tof_readings, current_theta):
    """Calculates resultant vector from waypoint attraction and ToF repulsion."""
    pos = np.array(current_pos)
    target = np.array(waypoint)
    
    # Attractive Vector
    f_att = K_ATT * (target - pos)
    
    # Repulsive Vectors (Real-time ToF vectors)
    f_rep = np.array([0.0, 0.0])
    
    for i in range(3):
        distance = tof_readings[i]
        if distance < SAFE_DIST:
            magnitude = K_REP * ((1.0 / distance) - (1.0 / SAFE_DIST)) / (distance ** 2)
            push_angle = current_theta + SENSOR_ANGLES[i] + math.pi
            f_rep[0] += magnitude * math.cos(push_angle)
            f_rep[1] += magnitude * math.sin(push_angle)
            
    return f_att + f_rep

def calculate_kinematics(f_total, current_theta):
    """Translates vector into left/right wheel speeds."""
    desired_theta = math.atan2(f_total[1], f_total[0])
    heading_error = math.atan2(math.sin(desired_theta - current_theta), math.cos(desired_theta - current_theta))
    
    v = min(np.linalg.norm(f_total), MAX_SPEED)
    omega = 2.0 * heading_error 
    
    v_left = v - (omega * WHEEL_BASE / 2.0)
    v_right = v + (omega * WHEEL_BASE / 2.0)
    
    return v_left, v_right

# ==========================================
# 4. MAIN NAVIGATION LOOP
# ==========================================
def read_esp32_data():
    """Placeholder for actual serial/JSON reading."""
    # Example format from your ESP32 JSON payload
    payload = '{"x": 1.0, "y": 1.0, "theta": 0.0, "tof": [1.5, 0.3, 1.5]}'
    return json.loads(payload)

def main():
    grid = create_empty_grid()
    start_pos = (1.0, 1.0)
    goal_pos = (9.0, 9.0)
    
    print("Calculating initial blind path...")
    waypoints = astar_grid(start_pos, goal_pos, grid)
    if not waypoints:
        print("Goal is blocked or invalid.")
        return
        
    waypoints.pop(0) # Remove starting location

    while waypoints:
        # 1. Read Odometry and Sensors
        robot_data = read_esp32_data()
        current_pos = (robot_data['x'], robot_data['y'])
        current_theta = robot_data['theta']
        tof_readings = robot_data['tof']
        
        # 2. Map Unknown Obstacles dynamically
        map_updated = False
        for i in range(3):
            dist = tof_readings[i]
            # If ToF hits something within 2 meters, map it to the grid
            if dist < 2.0: 
                sensor_angle = current_theta + SENSOR_ANGLES[i]
                obs_x = current_pos[0] + (dist * math.cos(sensor_angle))
                obs_y = current_pos[1] + (dist * math.sin(sensor_angle))
                
                # Only mark if it wasn't previously marked to save processing
                gx, gy = world_to_grid(obs_x, obs_y)
                if grid[gx][gy] == 0:
                    mark_obstacle(grid, obs_x, obs_y)
                    map_updated = True
        
        # 3. Dynamic Replanning
        # If the map changed, check if our current path goes straight through the new obstacle
        if map_updated and is_path_blocked(waypoints, grid):
            print("Obstacle blocking path! Replanning A*...")
            waypoints = astar_grid(current_pos, goal_pos, grid)
            if not waypoints:
                print("Trapped! No valid path to goal.")
                break
            waypoints.pop(0) # Skip current position
            continue # Restart loop with new waypoints
            
        # 4. Check Waypoint Arrival
        target_waypoint = waypoints[0]
        if math.dist(current_pos, target_waypoint) < 0.15: # 15cm arrival threshold
            print(f"Arrived at intermediate waypoint: {target_waypoint}")
            waypoints.pop(0)
            continue
            
        # 5. Local Vector Avoidance & Motor Control
        # APF ensures we smoothly slide around the newly mapped obstacle while driving
        f_total = calculate_apf(current_pos, target_waypoint, tof_readings, current_theta)
        v_left, v_right = calculate_kinematics(f_total, current_theta)
        
        # Output commands back to ESP32 / Motors
        print(f"L_Motor: {v_left:.2f}, R_Motor: {v_right:.2f}")
        
        time.sleep(0.1) # 10Hz control loop
        break # Remove break in production

if __name__ == "__main__":
    main()