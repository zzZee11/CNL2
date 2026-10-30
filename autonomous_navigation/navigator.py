"""
autonomous_navigation/navigator.py
-----------------------------------
Autonomous Cartesian Plane & Distance Navigator with Dynamic Obstacle Avoidance.

Capabilities:
- 2D Cartesian plane target navigation to any coordinate (X, Y) in meters (e.g. (1.0, 5.0)).
- Global A* grid path planning with dynamic replanning around obstacles.
- Artificial Potential Field (APF) reactive local micro-avoidance.
- Continuous 3x VL53L0X Time-of-Flight obstacle mapping with circular safety inflation.
- Dead-reckoning closed-loop wheel odometry integration.
- Automatic trajectory and obstacle map rendering to PNG image for analysis and testing.
"""

import time
import math
from typing import List, Tuple, Optional

try:
    from .config import (
        DEFAULT_SPEED, SENSOR_ANGLES,
        CRITICAL_STOP_DIST, TOF_MAX_RANGE_M,
        OBSTACLE_PERSISTENCE_COUNT, OBSTACLE_MAP_MAX_DIST_M,
        GOAL_TOLERANCE_M, LOOKAHEAD_DIST_M,
        DEFAULT_PLOT_PATH
    )
    from .planner import (
        create_empty_grid, mark_obstacle,
        astar_grid, is_path_blocked,
        calculate_apf, calculate_kinematics,
        world_to_grid
    )
    from .plotter import save_navigation_plot
except ImportError:
    from config import (
        DEFAULT_SPEED, SENSOR_ANGLES,
        CRITICAL_STOP_DIST, TOF_MAX_RANGE_M,
        OBSTACLE_PERSISTENCE_COUNT, OBSTACLE_MAP_MAX_DIST_M,
        GOAL_TOLERANCE_M, LOOKAHEAD_DIST_M,
        DEFAULT_PLOT_PATH
    )
    from planner import (
        create_empty_grid, mark_obstacle,
        astar_grid, is_path_blocked,
        calculate_apf, calculate_kinematics,
        world_to_grid
    )
    from plotter import save_navigation_plot


class AutonomousNavigator:
    def __init__(self, serial_comm, sensor_mgr):
        self.comm = serial_comm
        self.sensors = sensor_mgr
        self.grid = create_empty_grid()
        self.stop_requested = False

        # Obstacle persistence tracking to filter noise & floor glare
        self.sensor_hit_counts = [0, 0, 0]
        self.last_sensor_dists = [9.99, 9.99, 9.99]
        self.is_pivoting = False

        # Heading error tracking for PD course-correction rate damping
        self.last_heading_err = 0.0
        self.last_kinematics_time = 0.0

        # Mission logging & trajectory history
        self.last_trajectory: List[Tuple[float, float]] = []
        self.last_obstacles: List[Tuple[float, float]] = []
        self.last_planned_path: List[Tuple[float, float]] = []
        self.last_plot_path: Optional[str] = None

    def navigate_to(
        self,
        target_x: float,
        target_y: float,
        speed_mps: float = DEFAULT_SPEED,
        save_plot: bool = True,
        plot_path: str = DEFAULT_PLOT_PATH,
    ) -> bool:
        """
        Navigates the robot to the specified 2D Cartesian plane coordinate (target_x, target_y)
        in meters relative to the start position (0, 0), dynamically mapping obstacles and
        avoiding them using A* global planning and local APF.

        Args:
            target_x: Target position along the forward axis in meters (e.g. 1.0)
            target_y: Target position along the lateral axis in meters (e.g. 5.0, + is Left, - is Right)
            speed_mps: Target cruising speed in m/s (default: 0.60 m/s)
            save_plot: If True, saves a PNG image of the trajectory and obstacles after the run
            plot_path: Destination file path for the visualization image
        """
        self.stop_requested = False
        target_x = float(target_x)
        target_y = float(target_y)
        speed_mps = max(0.10, min(1.0, float(speed_mps)))

        goal_pos = (target_x, target_y)
        start_pos = (0.0, 0.0)
        direct_dist = math.hypot(target_x, target_y)

        if direct_dist < 0.05:
            print(f"[Nav] Target ({target_x:.2f}, {target_y:.2f}) is already at current position.")
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
        self.sensor_hit_counts = [0, 0, 0]
        self.last_sensor_dists = [9.99, 9.99, 9.99]
        self.is_pivoting = False
        self.last_heading_err = 0.0
        self.last_kinematics_time = time.time()

        # Telemetry history tracking
        self.last_trajectory = [(0.0, 0.0)]
        self.last_obstacles = []
        self.last_planned_path = []

        print(f"\n[Nav] ========================================================")
        print(f"[Nav] MISSION: Navigate to Cartesian Target ({target_x:.2f}, {target_y:.2f}) m")
        print(f"[Nav] Direct Distance: {direct_dist:.2f} m | Cruise Speed: {speed_mps:.2f} m/s")
        print(f"[Nav] Obstacle Avoidance: ACTIVE (A* Global Grid + APF Local Vector)")
        print(f"[Nav] ========================================================\n")

        # Compute initial global path
        print(f"[Nav] Planning initial A* path from (0.0, 0.0) to ({target_x:.2f}, {target_y:.2f})...")
        waypoints = astar_grid(start_pos, goal_pos, self.grid)
        if not waypoints:
            print("[Nav] Error: Could not compute initial path to goal.")
            return False

        self.last_planned_path = list(waypoints)
        print(f"[Nav] Initial route found with {len(waypoints)} waypoints.")

        if len(waypoints) > 1:
            waypoints.pop(0)  # Pop starting position

        # Safety timeout: 3x expected travel time + 15s buffer
        timeout = (direct_dist / speed_mps) * 3.5 + 15.0
        start_time = time.time()
        last_log_time = 0.0
        mission_status = "FAILED"

        try:
            while not self.stop_requested and waypoints:
                now = time.time()
                if now - start_time > timeout:
                    print(f"\n[Nav] TIMEOUT: Mission took longer than {timeout:.1f}s.")
                    mission_status = "TIMEOUT"
                    break

                # 1. Get current pose and odometry
                cur_x, cur_y, cur_theta, total_dist = self.comm.get_pose()
                current_pos = (cur_x, cur_y)

                # Append to trajectory history if moved significantly (> 2 cm)
                last_pt = self.last_trajectory[-1]
                if math.hypot(cur_x - last_pt[0], cur_y - last_pt[1]) >= 0.02:
                    self.last_trajectory.append((cur_x, cur_y))

                # Distance remaining to ultimate goal
                dist_to_goal = math.hypot(target_x - cur_x, target_y - cur_y)
                if dist_to_goal < GOAL_TOLERANCE_M:
                    print(f"\n[Nav] GOAL REACHED! Distance to target: {dist_to_goal:.2f} m (< {GOAL_TOLERANCE_M:.2f}m tolerance).")
                    mission_status = "GOAL REACHED"
                    break

                # 2. Read live ToF distance sensors [Left, Middle, Right]
                tof_readings = self.sensors.get_distances()

                # 3. Emergency brake if an obstacle suddenly appears critically close (< 0.18m)
                if any(0.02 < d < CRITICAL_STOP_DIST for d in tof_readings):
                    print(f"[Nav] EMERGENCY BRAKE: Critical hazard detected! Halting.")
                    self.comm.stop()
                    time.sleep(0.12)
                    # Mark all close obstacles into grid
                    for i in range(3):
                        d = tof_readings[i]
                        if 0.02 < d < CRITICAL_STOP_DIST:
                            sensor_angle = cur_theta + SENSOR_ANGLES[i]
                            obs_x = cur_x + (d * math.cos(sensor_angle))
                            obs_y = cur_y + (d * math.sin(sensor_angle))
                            mark_obstacle(self.grid, obs_x, obs_y)
                            self.last_obstacles.append((obs_x, obs_y))

                    # Replan path around hazard
                    new_path = astar_grid(current_pos, goal_pos, self.grid)
                    if not new_path:
                        print("[Nav] PATH BLOCKED: No route around obstacle!")
                        mission_status = "BLOCKED"
                        break
                    if len(new_path) > 1:
                        new_path.pop(0)
                    waypoints = new_path
                    continue

                # 4. Filter and map persistent obstacles into occupancy grid with circular inflation
                map_changed = False
                for i in range(3):
                    d = tof_readings[i]
                    # Filter out floor reflections (<0.06m) and distant noise (>OBSTACLE_MAP_MAX_DIST_M)
                    if 0.06 < d < OBSTACLE_MAP_MAX_DIST_M:
                        if abs(d - self.last_sensor_dists[i]) < 0.25:
                            self.sensor_hit_counts[i] += 1
                        else:
                            self.sensor_hit_counts[i] = 1
                        self.last_sensor_dists[i] = d

                        # Only map obstacle into A* grid if detected persistently (>= 2 cycles)
                        if self.sensor_hit_counts[i] >= OBSTACLE_PERSISTENCE_COUNT:
                            sensor_angle = cur_theta + SENSOR_ANGLES[i]
                            obs_x = cur_x + (d * math.cos(sensor_angle))
                            obs_y = cur_y + (d * math.sin(sensor_angle))

                            gx, gy = world_to_grid(obs_x, obs_y)
                            if self.grid[gx][gy] == 0:
                                mark_obstacle(self.grid, obs_x, obs_y)
                                self.last_obstacles.append((obs_x, obs_y))
                                map_changed = True
                    else:
                        self.sensor_hit_counts[i] = 0
                        self.last_sensor_dists[i] = d

                # 5. Dynamic A* Replanning if current route is blocked
                if map_changed and is_path_blocked(waypoints, self.grid):
                    print("[Nav] Obstacle detected on route! Replanning A* path...")
                    new_path = astar_grid(current_pos, goal_pos, self.grid)
                    if not new_path:
                        print("[Nav] Trapped! No clear route to goal.")
                        mission_status = "BLOCKED"
                        break
                    if len(new_path) > 1:
                        new_path.pop(0)
                    waypoints = new_path
                    print(f"[Nav] New path computed with {len(waypoints)} waypoints.")
                    continue

                # 6. Waypoint Tracking with Pure Pursuit Lookahead
                while len(waypoints) > 1:
                    wp_dist = math.hypot(waypoints[0][0] - cur_x, waypoints[0][1] - cur_y)
                    if wp_dist < 0.22:
                        waypoints.pop(0)
                    else:
                        break

                # Select lookahead target waypoint along path (approx 35 cm ahead) for smooth tracking
                target_wp = waypoints[-1]
                for wp in waypoints:
                    if math.hypot(wp[0] - cur_x, wp[1] - cur_y) >= LOOKAHEAD_DIST_M:
                        target_wp = wp
                        break

                # 7. Local APF Reactive Avoidance & Velocity Calculation
                effective_speed = speed_mps
                if dist_to_goal < 0.40:
                    # Smooth deceleration ramp for the final 40 cm to stop precisely at goal
                    effective_speed = max(0.10, speed_mps * (dist_to_goal / 0.40))

                f_total = calculate_apf(current_pos, target_wp, tof_readings, cur_theta, effective_speed)

                # Derivative rate of heading error for PD damping (Course Correction)
                desired_theta = math.atan2(f_total[1], f_total[0])
                current_heading_err = math.atan2(
                    math.sin(desired_theta - cur_theta),
                    math.cos(desired_theta - cur_theta)
                )
                dt_kin = max(0.005, now - self.last_kinematics_time)
                d_heading_err = (current_heading_err - self.last_heading_err) / dt_kin
                self.last_heading_err = current_heading_err
                self.last_kinematics_time = now

                v_left, v_right = calculate_kinematics(
                    f_total, cur_theta, effective_speed,
                    is_pivoting=self.is_pivoting,
                    d_heading_error=d_heading_err
                )
                self.is_pivoting = (v_left * v_right < -0.001)

                # Send commands to ESP32
                self.comm.set_speeds(v_left, v_right)

                # Status log at 3 Hz with live heading error
                if now - last_log_time > 0.3:
                    heading_deg = math.degrees(cur_theta)
                    heading_err_deg = math.degrees(current_heading_err)
                    print(
                        f"[Nav] Pos: ({cur_x:5.2f}, {cur_y:5.2f})m | "
                        f"Rem: {dist_to_goal:4.2f}m | "
                        f"Heading: {heading_deg:+5.1f}° (Err: {heading_err_deg:+5.1f}°) | "
                        f"Cmd: ({v_left:+.2f}, {v_right:+.2f}) m/s"
                    )
                    last_log_time = now

                time.sleep(0.05)  # 20 Hz navigation loop

            if self.stop_requested:
                mission_status = "STOPPED_BY_USER"

        finally:
            self.comm.stop()

        # Final mission recording
        final_x, final_y, final_theta, final_dist = self.comm.get_pose()
        self.last_trajectory.append((final_x, final_y))
        elapsed_total = time.time() - start_time

        print(f"\n[Nav] ========================================================")
        print(f"[Nav] Mission Finished: {mission_status}")
        print(f"[Nav] Final Pose: ({final_x:.2f}, {final_y:.2f}) m | Heading: {math.degrees(final_theta):.1f}°")
        print(f"[Nav] Distance Travelled: {final_dist:.2f} m | Time: {elapsed_total:.1f} s")
        print(f"[Nav] ========================================================\n")

        # Save trajectory & obstacle map plot
        if save_plot:
            self.last_plot_path = save_navigation_plot(
                trajectory=self.last_trajectory,
                recorded_obstacles=self.last_obstacles,
                start_pos=start_pos,
                goal_pos=goal_pos,
                planned_path=self.last_planned_path,
                final_pose=(final_x, final_y, final_theta),
                total_dist=final_dist,
                elapsed_s=elapsed_total,
                status=mission_status,
                output_path=plot_path,
                grid=self.grid,
            )

        return mission_status == "GOAL REACHED"

    def move_distance(
        self,
        distance_m: float,
        speed_mps: float = DEFAULT_SPEED,
        save_plot: bool = True,
        plot_path: str = DEFAULT_PLOT_PATH,
    ) -> bool:
        """
        Drives the robot a linear distance forward in meters (backwards-compatible wrapper).
        Equivalent to navigate_to(target_x=distance_m, target_y=0.0).
        """
        return self.navigate_to(
            target_x=float(distance_m),
            target_y=0.0,
            speed_mps=speed_mps,
            save_plot=save_plot,
            plot_path=plot_path,
        )

    def request_stop(self) -> None:
        """Signals the navigation loop to halt immediately."""
        self.stop_requested = True
        self.comm.stop()
