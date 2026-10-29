"""
autonomous_navigation/navigator.py
-----------------------------------
Autonomous Distance & Obstacle Avoidance Navigator.
Combines:
- A* Global Grid Search (for macro-level routing around detected obstacles)
- Artificial Potential Field (APF) local vector avoidance (for real-time micro-evasion)
- Dynamic ToF obstacle mapping with circular inflation
- Closed-loop wheel odometry with dead-reckoning
- Startup breakaway torque push
"""

import time
import math

try:
    from .config import (
        DEFAULT_SPEED, SENSOR_ANGLES,
        CRITICAL_STOP_DIST, TOF_MAX_RANGE_M
    )
    from .planner import (
        create_empty_grid, mark_obstacle,
        astar_grid, is_path_blocked,
        calculate_apf, calculate_kinematics,
        world_to_grid
    )
except ImportError:
    from config import (
        DEFAULT_SPEED, SENSOR_ANGLES,
        CRITICAL_STOP_DIST, TOF_MAX_RANGE_M
    )
    from planner import (
        create_empty_grid, mark_obstacle,
        astar_grid, is_path_blocked,
        calculate_apf, calculate_kinematics,
        world_to_grid
    )

class AutonomousNavigator:
    def __init__(self, serial_comm, sensor_mgr):
        self.comm = serial_comm
        self.sensors = sensor_mgr
        self.grid = create_empty_grid()
        self.stop_requested = False

    def move_distance(self, distance_m, speed_mps=DEFAULT_SPEED):
        """
        Drives the robot the commanded distance (e.g. 5.0m) at the commanded speed (e.g. 0.8 m/s),
        dynamically mapping obstacles with 3x ToF and avoiding them using A* and APF.
        """
        self.stop_requested = False
        distance_m = float(distance_m)
        speed_mps = max(0.1, min(1.0, float(speed_mps)))

        if abs(distance_m) < 0.05:
            print("[Nav] Distance is too small to travel.")
            return True

        # Wait until we have valid encoder telemetry from ESP32
        print("[Nav] Synchronizing with robot odometry...")
        t_wait = time.time()
        while not self.comm.has_first_encoder:
            if time.time() - t_wait > 5.0:
                print("[Nav] Error: No encoder telemetry received from ESP32.")
                return False
            time.sleep(0.05)

        # Reset starting pose to (0, 0) facing theta = 0 (along the +X axis)
        self.comm.reset_odometry(start_x=0.0, start_y=0.0, start_theta=0.0)
        self.grid = create_empty_grid()

        # Goal position is straight ahead along the X axis
        goal_pos = (distance_m, 0.0)
        start_pos = (0.0, 0.0)

        print(f"\n[Nav] ========================================================")
        print(f"[Nav] TARGET: Move {distance_m:.2f} m at {speed_mps:.2f} m/s")
        print(f"[Nav] Goal coordinates: ({goal_pos[0]:.2f}, {goal_pos[1]:.2f})")
        print(f"[Nav] Obstacle avoidance: ACTIVE (A* Global + APF Local)")
        print(f"[Nav] ========================================================\n")

        # Compute initial global path
        print("[Nav] Planning initial A* path...")
        waypoints = astar_grid(start_pos, goal_pos, self.grid)
        if not waypoints:
            print("[Nav] Error: Could not compute initial path.")
            return False

        if len(waypoints) > 1:
            waypoints.pop(0)  # Remove current starting point

        # Safety timeout: 3x expected travel time + 10s buffer
        timeout = (distance_m / speed_mps) * 3.0 + 10.0
        start_time = time.time()
        last_log_time = 0.0

        try:
            while not self.stop_requested and waypoints:
                now = time.time()
                if now - start_time > timeout:
                    print(f"\n[Nav] TIMEOUT: Mission took longer than {timeout:.1f}s.")
                    break

                # 1. Get current pose and odometry
                cur_x, cur_y, cur_theta, total_dist = self.comm.get_pose()
                current_pos = (cur_x, cur_y)

                # Distance remaining to the ultimate goal
                dist_to_goal = math.hypot(goal_pos[0] - cur_x, goal_pos[1] - cur_y)
                if dist_to_goal < 0.12 or total_dist >= (distance_m - 0.04):
                    print(f"\n[Nav] GOAL REACHED! Total distance travelled: {total_dist:.2f} m.")
                    break

                # 2. Read live ToF distance sensors [Left, Middle, Right]
                tof_readings = self.sensors.get_distances()

                # 3. Emergency brake if an obstacle suddenly appears critically close
                if tof_readings[1] < CRITICAL_STOP_DIST:
                    print(f"[Nav] EMERGENCY BRAKE: Obstacle detected at {tof_readings[1]:.2f} m!")
                    self.comm.stop()
                    time.sleep(0.15)
                    # Force mapping obstacle directly in front
                    obs_x = cur_x + (tof_readings[1] * math.cos(cur_theta))
                    obs_y = cur_y + (tof_readings[1] * math.sin(cur_theta))
                    mark_obstacle(self.grid, obs_x, obs_y)
                    # Replan path around it
                    waypoints = astar_grid(current_pos, goal_pos, self.grid)
                    if not waypoints:
                        print("[Nav] PATH BLOCKED: No route around obstacle!")
                        break
                    if len(waypoints) > 1:
                        waypoints.pop(0)
                    continue

                # 4. Map detected obstacles into the occupancy grid with circular inflation
                map_changed = False
                for i in range(3):
                    d = tof_readings[i]
                    if 0.05 < d < TOF_MAX_RANGE_M:
                        sensor_angle = cur_theta + SENSOR_ANGLES[i]
                        obs_x = cur_x + (d * math.cos(sensor_angle))
                        obs_y = cur_y + (d * math.sin(sensor_angle))

                        gx, gy = world_to_grid(obs_x, obs_y)
                        if self.grid[gx][gy] == 0:
                            mark_obstacle(self.grid, obs_x, obs_y)
                            map_changed = True

                # 5. Dynamic A* Replanning if current route is blocked
                if map_changed and is_path_blocked(waypoints, self.grid):
                    print("[Nav] Obstacle detected on route! Replanning A*...")
                    new_path = astar_grid(current_pos, goal_pos, self.grid)
                    if not new_path:
                        print("[Nav] Trapped! No clear route to goal.")
                        break
                    if len(new_path) > 1:
                        new_path.pop(0)
                    waypoints = new_path
                    print(f"[Nav] New path computed with {len(waypoints)} waypoints.")
                    continue

                # 6. Waypoint Tracking
                target_wp = waypoints[0]
                dist_to_wp = math.hypot(target_wp[0] - cur_x, target_wp[1] - cur_y)
                if dist_to_wp < 0.20:
                    waypoints.pop(0)
                    if not waypoints:
                        # If reached last intermediate waypoint, direct toward final goal
                        waypoints = [goal_pos]
                    target_wp = waypoints[0]

                # 7. Local APF Reactive Avoidance & Velocity Calculation
                # Adjust cruise speed: slow down slightly when nearing the final destination
                effective_speed = speed_mps
                if dist_to_goal < 0.50:
                    effective_speed = max(0.20, speed_mps * (dist_to_goal / 0.50))

                f_total = calculate_apf(current_pos, target_wp, tof_readings, cur_theta, effective_speed)
                v_left, v_right = calculate_kinematics(f_total, cur_theta, effective_speed)

                # Send commands to ESP32
                self.comm.set_speeds(v_left, v_right)

                # Status log at 2 Hz
                if now - last_log_time > 0.5:
                    print(
                        f"[Nav] Pos: ({cur_x:5.2f}, {cur_y:5.2f})m | "
                        f"Dist: {total_dist:5.2f}/{distance_m:.1f}m | "
                        f"ToF: L={tof_readings[0]:4.2f} M={tof_readings[1]:4.2f} R={tof_readings[2]:4.2f}m | "
                        f"Cmd: ({v_left:+.2f}, {v_right:+.2f}) m/s"
                    )
                    last_log_time = now

                time.sleep(0.05)  # 20 Hz navigation loop

        finally:
            self.comm.stop()

        final_dist = self.comm.get_pose()[3]
        print(f"[Nav] Navigation finished. Final distance travelled: {final_dist:.2f} m.")
        return True

    def request_stop(self):
        """Signals the navigation loop to stop immediately."""
        self.stop_requested = True
        self.comm.stop()
