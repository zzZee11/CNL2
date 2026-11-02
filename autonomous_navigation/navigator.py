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
        CRITICAL_STOP_DIST, CRITICAL_PERSIST_TICKS,
        TOF_MIN_VALID_M, TOF_MAX_RANGE_M,
        OBSTACLE_PERSISTENCE_COUNT, OBSTACLE_CONFIRM_RADIUS_M,
        OBSTACLE_MAP_MAX_DIST_M, MIN_OBSTACLE_SPACING_M,
        GOAL_TOLERANCE_M, LOOKAHEAD_DIST_M,
        FINAL_HEADING_TOLERANCE_DEG, KP_ALIGN_TURN, KD_ALIGN_TURN,
        MIN_ALIGN_SPEED, MAX_ALIGN_SPEED,
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
        CRITICAL_STOP_DIST, CRITICAL_PERSIST_TICKS,
        TOF_MIN_VALID_M, TOF_MAX_RANGE_M,
        OBSTACLE_PERSISTENCE_COUNT, OBSTACLE_CONFIRM_RADIUS_M,
        OBSTACLE_MAP_MAX_DIST_M, MIN_OBSTACLE_SPACING_M,
        GOAL_TOLERANCE_M, LOOKAHEAD_DIST_M,
        FINAL_HEADING_TOLERANCE_DEG, KP_ALIGN_TURN, KD_ALIGN_TURN,
        MIN_ALIGN_SPEED, MAX_ALIGN_SPEED,
        DEFAULT_PLOT_PATH
    )
    from planner import (
        create_empty_grid, mark_obstacle,
        astar_grid, is_path_blocked,
        calculate_apf, calculate_kinematics,
        world_to_grid
    )
    from plotter import save_navigation_plot


def simplify_trajectory(trajectory: List[Tuple[float, float]], min_spacing: float = 0.06) -> List[Tuple[float, float]]:
    """
    Downsamples a dense trajectory to evenly spaced waypoints (min_spacing meters apart)
    while strictly preserving the start point, intermediate curves, and destination.
    """
    if len(trajectory) <= 2:
        return list(trajectory)
    simplified = [trajectory[0]]
    for p in trajectory[1:-1]:
        if math.hypot(p[0] - simplified[-1][0], p[1] - simplified[-1][1]) >= min_spacing:
            simplified.append(p)
    simplified.append(trajectory[-1])
    return simplified


class AutonomousNavigator:
    def __init__(self, serial_comm, sensor_mgr):
        self.comm = serial_comm
        self.sensors = sensor_mgr
        self.grid = create_empty_grid()
        self.stop_requested = False
        self.start_theta = 0.0

        # Obstacle spatial consensus tracking to eliminate floor glare & optical noise
        self.candidate_obstacles: List[dict] = []
        self.critical_hazard_ticks = 0
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
        target_theta_deg: Optional[float] = None,
        return_to_start: bool = False,
        save_plot: bool = True,
        plot_path: str = DEFAULT_PLOT_PATH,
    ) -> bool:
        """
        Navigates the robot to the specified 2D Cartesian plane coordinate (target_x, target_y)
        in meters relative to the start position (0, 0), dynamically mapping obstacles and
        avoiding them using A* global planning and local APF.
        Optionally aligns to a specific orientation (target_theta_deg) at the destination.

        Args:
            target_x: Target position along the forward axis in meters (e.g. 1.0)
            target_y: Target position along the lateral axis in meters (e.g. 5.0, + is Left, - is Right)
            speed_mps: Target cruising speed in m/s (default: 0.60 m/s)
            target_theta_deg: Optional target orientation (degrees) to align with at the goal
            save_plot: If True, saves a PNG image of the trajectory and obstacles after the run
            plot_path: Destination file path for the visualization image
        """
        self.stop_requested = False
        target_x = float(target_x)
        target_y = float(target_y)
        speed_mps = max(0.10, min(1.0, float(speed_mps)))
        target_theta_deg = float(target_theta_deg) if target_theta_deg is not None else None

        goal_pos = (target_x, target_y)
        start_pos = (0.0, 0.0)
        direct_dist = math.hypot(target_x, target_y)

        if direct_dist < 0.05:
            if target_theta_deg is not None:
                print(f"[Nav] Already at target position; aligning to heading {target_theta_deg:+.1f}°...")
                return self._align_to_orientation(target_theta_deg)
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
        self.candidate_obstacles = []
        self.critical_hazard_ticks = 0
        self.sensor_hit_counts = [0, 0, 0]
        self.last_sensor_dists = [9.99, 9.99, 9.99]
        self.is_pivoting = False
        self.last_heading_err = 0.0
        self.last_kinematics_time = time.time()

        # Telemetry history tracking
        self.last_trajectory = [(0.0, 0.0)]
        self.last_obstacles = []
        self.last_planned_path = []

        target_hdg_str = f"{target_theta_deg:+.1f}°" if target_theta_deg is not None else "Any"
        print(f"\n[Nav] ========================================================")
        print(f"[Nav] MISSION: Navigate to Cartesian Target ({target_x:.2f}, {target_y:.2f}) m | Final Heading: {target_hdg_str}")
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

                # 3. Critical hazard check & emergency brake
                # Requires confirmation over CRITICAL_PERSIST_TICKS (e.g. 2 cycles / 100ms) to reject optical spikes
                hazard_close = any(TOF_MIN_VALID_M <= d < CRITICAL_STOP_DIST for d in tof_readings)
                if hazard_close:
                    self.critical_hazard_ticks += 1
                else:
                    self.critical_hazard_ticks = 0

                if self.critical_hazard_ticks >= CRITICAL_PERSIST_TICKS:
                    print(f"\n[Nav] EMERGENCY BRAKE: Confirmed critical hazard ({min(tof_readings):.2f}m < {CRITICAL_STOP_DIST:.2f}m)! Halting.")
                    self.comm.stop()
                    time.sleep(0.10)
                    self.critical_hazard_ticks = 0

                    # Mark all close confirmed obstacles into grid
                    fresh_tof = self.sensors.get_distances()
                    for i in range(3):
                        d = fresh_tof[i]
                        if TOF_MIN_VALID_M <= d < (CRITICAL_STOP_DIST + 0.05):
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

                # 4. Spatial Consensus Obstacle Mapping (World-Frame Clustering)
                # Real stationary obstacles remain fixed in world coordinates as robot moves.
                # Transient floor glare / moving reflections move along with the robot and are rejected.
                map_changed = False
                for i in range(3):
                    d = tof_readings[i]
                    if TOF_MIN_VALID_M <= d <= OBSTACLE_MAP_MAX_DIST_M:
                        sensor_angle = cur_theta + SENSOR_ANGLES[i]
                        obs_x = cur_x + (d * math.cos(sensor_angle))
                        obs_y = cur_y + (d * math.sin(sensor_angle))

                        # Check if already mapped into permanent obstacle list
                        already_mapped = any(
                            math.hypot(obs_x - ox, obs_y - oy) < MIN_OBSTACLE_SPACING_M
                            for ox, oy in self.last_obstacles
                        )
                        if already_mapped:
                            continue

                        # Match against active candidate clusters in world coordinates
                        matched_candidate = None
                        for cand in self.candidate_obstacles:
                            if math.hypot(obs_x - cand["x"], obs_y - cand["y"]) <= OBSTACLE_CONFIRM_RADIUS_M:
                                matched_candidate = cand
                                break

                        if matched_candidate:
                            matched_candidate["hits"] += 1
                            # Exponential moving average to refine world centroid
                            matched_candidate["x"] = 0.70 * matched_candidate["x"] + 0.30 * obs_x
                            matched_candidate["y"] = 0.70 * matched_candidate["y"] + 0.30 * obs_y
                            matched_candidate["last_seen"] = now

                            if matched_candidate["hits"] >= OBSTACLE_PERSISTENCE_COUNT and not matched_candidate["confirmed"]:
                                matched_candidate["confirmed"] = True
                                gx, gy = world_to_grid(matched_candidate["x"], matched_candidate["y"])
                                if self.grid[gx][gy] == 0:
                                    mark_obstacle(self.grid, matched_candidate["x"], matched_candidate["y"])
                                    self.last_obstacles.append((matched_candidate["x"], matched_candidate["y"]))
                                    map_changed = True
                                    print(
                                        f"[Nav] Confirmed critical obstacle mapped at ({matched_candidate['x']:.2f}, {matched_candidate['y']:.2f}) m "
                                        f"[Sensor {i}, Hits: {matched_candidate['hits']}]"
                                    )
                        else:
                            self.candidate_obstacles.append({
                                "x": obs_x,
                                "y": obs_y,
                                "hits": 1,
                                "first_seen": now,
                                "last_seen": now,
                                "confirmed": False
                            })

                # Prune stale unconfirmed candidates older than 0.40s
                self.candidate_obstacles = [
                    c for c in self.candidate_obstacles
                    if c["confirmed"] or (now - c["last_seen"] <= 0.40)
                ]

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

                # Check for physical collision impact spike from IMU accelerometer
                if self.comm.imu and self.comm.imu.has_collision():
                    print("\n[Nav] EMERGENCY: Physical impact shock detected by IMU! Halting.")
                    mission_status = "COLLISION_HALT"
                    break

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

                # Use spatial gyro yaw rate if IMU is healthy, else fallback to numerical derivative
                yaw_rate = self.comm.imu.get_yaw_rate_rad_s() if (self.comm.imu and self.comm.imu.is_ready()) else None

                v_left, v_right = calculate_kinematics(
                    f_total, cur_theta, effective_speed,
                    is_pivoting=self.is_pivoting,
                    d_heading_error=d_heading_err,
                    yaw_rate=yaw_rate
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

        # Final orientation alignment at destination if requested and goal was reached
        if mission_status == "GOAL REACHED" and target_theta_deg is not None and not self.stop_requested:
            align_ok = self._align_to_orientation(target_theta_deg)
            if not align_ok:
                print(f"[Nav] Note: Alignment finished outside tolerance.")

        # If return-to-start was requested, immediately retrace path back to start
        if return_to_start and mission_status == "GOAL REACHED" and not self.stop_requested:
            print("\n[Nav] Outward destination reached! Commencing return trip along exact recorded path...")
            return self.return_to_start(
                speed_mps=speed_mps,
                target_theta_deg=0.0,
                save_plot=save_plot,
                plot_path=plot_path
            )

        # Final mission recording
        final_x, final_y, final_theta, final_dist = self.comm.get_pose()
        self.last_trajectory.append((final_x, final_y))
        elapsed_total = time.time() - start_time

        target_info = f" (Target: {target_theta_deg:+.1f}°)" if target_theta_deg is not None else ""
        print(f"\n[Nav] ========================================================")
        print(f"[Nav] Mission Finished: {mission_status}")
        print(f"[Nav] Final Pose: ({final_x:.2f}, {final_y:.2f}) m | Heading: {math.degrees(final_theta):.1f}°{target_info}")
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
                target_theta=math.radians(target_theta_deg) if target_theta_deg is not None else None,
                status=mission_status,
                output_path=plot_path,
                grid=self.grid,
            )

        return mission_status == "GOAL REACHED"

    def _align_to_orientation(self, target_deg: float, timeout_s: float = 6.0) -> bool:
        """
        Pivots the robot in-place at the destination to align with target_deg (degrees).
        Uses MPU-6050 spatial IMU (or encoder odometry fallback) with closed-loop PD control.
        """
        target_rad = math.radians(target_deg)
        target_rad = math.atan2(math.sin(target_rad), math.cos(target_rad))
        print(f"\n[Nav] Final Alignment: Rotating in-place to target heading {target_deg:+.1f}°...")

        self.comm.stop()
        time.sleep(0.08)

        timeout = max(timeout_s, abs(target_deg) / 30.0 + 3.0)
        t0 = time.time()
        last_report = 0.0

        try:
            while not self.stop_requested:
                now = time.time()
                if now - t0 > timeout:
                    print(f"[Nav] Alignment timeout after {timeout:.1f}s.")
                    return False

                cur_x, cur_y, cur_theta, _ = self.comm.get_pose()

                # Shortest angular error in [-pi, +pi]
                heading_err = math.atan2(
                    math.sin(target_rad - cur_theta),
                    math.cos(target_rad - cur_theta)
                )
                err_deg = math.degrees(heading_err)

                if abs(err_deg) <= FINAL_HEADING_TOLERANCE_DEG:
                    self.comm.stop()
                    print(f"[Nav] Final alignment achieved! Error: {err_deg:+.1f}° (<= {FINAL_HEADING_TOLERANCE_DEG:.1f}° tolerance).")
                    return True

                yaw_rate = self.comm.imu.get_yaw_rate_rad_s() if (self.comm.imu and self.comm.imu.is_ready()) else 0.0

                # PD pivot control
                omega = (KP_ALIGN_TURN * heading_err) - (KD_ALIGN_TURN * yaw_rate)
                omega_clamped = max(-MAX_ALIGN_SPEED, min(MAX_ALIGN_SPEED, omega))

                # Smoothly scale minimum speed down when error is small (< 6 deg)
                # to prevent discrete step quantization hunting
                effective_min = MIN_ALIGN_SPEED if abs(err_deg) > 6.0 else max(0.05, MIN_ALIGN_SPEED * (abs(err_deg) / 6.0))
                if abs(omega_clamped) < effective_min:
                    omega_clamped = math.copysign(effective_min, omega_clamped)

                # Differential pivot: counter-rotating wheels
                # heading_err > 0 means need CCW / left turn: Left wheel backward, Right wheel forward
                cmd_l = -omega_clamped
                cmd_r = +omega_clamped
                self.comm.set_speeds(cmd_l, cmd_r)

                if now - last_report > 0.25:
                    cur_deg = math.degrees(cur_theta)
                    sensor_tag = "IMU" if (self.comm.imu and self.comm.imu.is_ready()) else "ODOM"
                    print(
                        f"  [Align] Current: {cur_deg:+5.1f}° | "
                        f"Target: {target_deg:+5.1f}° | "
                        f"Err: {err_deg:+5.1f}° [{sensor_tag}] | "
                        f"Cmd: ({cmd_l:+.2f}, {cmd_r:+.2f}) m/s"
                    )
                    last_report = now

                time.sleep(0.04)

        finally:
            self.comm.stop()

        return False

    def follow_path(
        self,
        path: List[Tuple[float, float]],
        speed_mps: float = DEFAULT_SPEED,
        target_theta_deg: Optional[float] = None,
        label: str = "Follow Path",
    ) -> bool:
        """
        Follows an exact sequence of 2D waypoints forward using APF local micro-avoidance
        and pure pursuit lookahead, pivoting in-place initially to face the path heading.
        Aligns in-place to target_theta_deg (if specified) upon arrival at destination.
        """
        if not path or len(path) < 1:
            return True

        self.stop_requested = False
        speed_mps = max(0.10, min(1.0, float(speed_mps)))
        waypoints = list(path)
        goal_pos = waypoints[-1]

        cur_x, cur_y, cur_theta, _ = self.comm.get_pose()
        direct_dist = math.hypot(goal_pos[0] - cur_x, goal_pos[1] - cur_y)

        # 1. Pivot in-place to face the path heading initially (Forward motion, NOT reverse!)
        target_first_wp = waypoints[-1]
        for wp in waypoints:
            if math.hypot(wp[0] - cur_x, wp[1] - cur_y) >= 0.12:
                target_first_wp = wp
                break

        dx_init = target_first_wp[0] - cur_x
        dy_init = target_first_wp[1] - cur_y
        if math.hypot(dx_init, dy_init) >= 0.05:
            init_heading = math.atan2(dy_init, dx_init)
            heading_diff = math.atan2(
                math.sin(init_heading - cur_theta),
                math.cos(init_heading - cur_theta)
            )
            if abs(math.degrees(heading_diff)) > 15.0:
                print(f"[Nav] [{label}] Pivoting in-place to face path heading ({math.degrees(init_heading):+.1f}°)...")
                self._align_to_orientation(math.degrees(init_heading))

        # Reset kinematics state
        self.is_pivoting = False
        self.last_heading_err = 0.0
        self.last_kinematics_time = time.time()

        # Compute total path distance
        path_dist = 0.0
        for i in range(len(waypoints) - 1):
            path_dist += math.hypot(waypoints[i+1][0] - waypoints[i][0], waypoints[i+1][1] - waypoints[i][1])
        path_dist = max(path_dist, direct_dist)

        timeout = (path_dist / speed_mps) * 3.5 + 15.0
        start_time = time.time()
        last_log_time = 0.0
        critical_hazard_ticks = 0
        mission_status = "FAILED"

        try:
            while not self.stop_requested and waypoints:
                now = time.time()
                if now - start_time > timeout:
                    print(f"\n[Nav] [{label}] TIMEOUT: Path following took longer than {timeout:.1f}s.")
                    mission_status = "TIMEOUT"
                    break

                cur_x, cur_y, cur_theta, total_dist = self.comm.get_pose()
                current_pos = (cur_x, cur_y)

                # Record trajectory
                last_pt = self.last_trajectory[-1]
                if math.hypot(cur_x - last_pt[0], cur_y - last_pt[1]) >= 0.02:
                    self.last_trajectory.append((cur_x, cur_y))

                dist_to_goal = math.hypot(goal_pos[0] - cur_x, goal_pos[1] - cur_y)
                if dist_to_goal < GOAL_TOLERANCE_M:
                    print(f"\n[Nav] [{label}] DESTINATION REACHED! Distance to target: {dist_to_goal:.2f} m (< {GOAL_TOLERANCE_M:.2f}m).")
                    mission_status = "GOAL REACHED"
                    break

                # 2. Read live ToF distance sensors [Left, Middle, Right]
                tof_readings = self.sensors.get_distances()

                # Critical hazard check & emergency stop (< CRITICAL_STOP_DIST)
                hazard_close = any(TOF_MIN_VALID_M <= d < CRITICAL_STOP_DIST for d in tof_readings)
                if hazard_close:
                    critical_hazard_ticks += 1
                else:
                    critical_hazard_ticks = 0

                if critical_hazard_ticks >= CRITICAL_PERSIST_TICKS:
                    print(f"\n[Nav] [{label}] EMERGENCY BRAKE: Confirmed critical obstacle on path ({min(tof_readings):.2f}m < {CRITICAL_STOP_DIST:.2f}m)! Halting.")
                    self.comm.stop()
                    mission_status = "BLOCKED"
                    break

                # Advance waypoints
                while len(waypoints) > 1:
                    wp_dist = math.hypot(waypoints[0][0] - cur_x, waypoints[0][1] - cur_y)
                    if wp_dist < 0.22:
                        waypoints.pop(0)
                    else:
                        break

                target_wp = waypoints[-1]
                for wp in waypoints:
                    if math.hypot(wp[0] - cur_x, wp[1] - cur_y) >= LOOKAHEAD_DIST_M:
                        target_wp = wp
                        break

                effective_speed = speed_mps
                if dist_to_goal < 0.40:
                    effective_speed = max(0.10, speed_mps * (dist_to_goal / 0.40))

                if self.comm.imu and self.comm.imu.has_collision():
                    print(f"\n[Nav] [{label}] Physical impact shock detected by IMU! Halting.")
                    mission_status = "COLLISION_HALT"
                    break

                f_total = calculate_apf(current_pos, target_wp, tof_readings, cur_theta, effective_speed)

                desired_theta = math.atan2(f_total[1], f_total[0])
                current_heading_err = math.atan2(
                    math.sin(desired_theta - cur_theta),
                    math.cos(desired_theta - cur_theta)
                )
                dt_kin = max(0.005, now - self.last_kinematics_time)
                d_heading_err = (current_heading_err - self.last_heading_err) / dt_kin
                self.last_heading_err = current_heading_err
                self.last_kinematics_time = now

                yaw_rate = self.comm.imu.get_yaw_rate_rad_s() if (self.comm.imu and self.comm.imu.is_ready()) else None

                v_left, v_right = calculate_kinematics(
                    f_total, cur_theta, effective_speed,
                    is_pivoting=self.is_pivoting,
                    d_heading_error=d_heading_err,
                    yaw_rate=yaw_rate
                )
                self.is_pivoting = (v_left * v_right < -0.001)
                self.comm.set_speeds(v_left, v_right)

                if now - last_log_time > 0.3:
                    heading_deg = math.degrees(cur_theta)
                    heading_err_deg = math.degrees(current_heading_err)
                    print(
                        f"[Nav] [{label}] Pos: ({cur_x:5.2f}, {cur_y:5.2f})m | "
                        f"Rem: {dist_to_goal:4.2f}m | "
                        f"Heading: {heading_deg:+5.1f}° (Err: {heading_err_deg:+5.1f}°) | "
                        f"Cmd: ({v_left:+.2f}, {v_right:+.2f}) m/s"
                    )
                    last_log_time = now

                time.sleep(0.05)

            if self.stop_requested:
                mission_status = "STOPPED_BY_USER"

        finally:
            self.comm.stop()

        # Final orientation alignment at destination if requested and goal was reached
        if mission_status == "GOAL REACHED" and target_theta_deg is not None and not self.stop_requested:
            self._align_to_orientation(target_theta_deg)

        final_x, final_y, final_theta, _ = self.comm.get_pose()
        self.last_trajectory.append((final_x, final_y))
        return mission_status == "GOAL REACHED"

    def return_to_start(
        self,
        speed_mps: float = DEFAULT_SPEED,
        target_theta_deg: float = 0.0,
        save_plot: bool = True,
        plot_path: str = DEFAULT_PLOT_PATH,
    ) -> bool:
        """
        Retraces the exact recorded trajectory back to the last start point (0, 0)
        in FORWARD motion (pivoting in-place to face the return direction first), and
        aligns to the exact original start orientation (0.0°).
        """
        if len(self.last_trajectory) < 2:
            print("[Nav] Error: No recorded trajectory history to follow back. Drive somewhere first!")
            return False

        # Simplify recorded trajectory to clean waypoints and reverse it
        forward_path = simplify_trajectory(self.last_trajectory)
        return_path = list(reversed(forward_path))

        dest_pt = return_path[-1]  # (0.0, 0.0)
        print(f"\n[Nav] ========================================================")
        print(f"[Nav] RETRACING EXACT PATH BACK TO START POINT ({dest_pt[0]:.2f}, {dest_pt[1]:.2f})")
        print(f"[Nav] Return Waypoints: {len(return_path)} | Target Orientation: {target_theta_deg:+.1f}°")
        print(f"[Nav] Mode: FORWARD MOTION (Pivoting in-place at goal first)")
        print(f"[Nav] ========================================================\n")

        ok = self.follow_path(
            path=return_path,
            speed_mps=speed_mps,
            target_theta_deg=target_theta_deg,
            label="Return to Start"
        )

        final_x, final_y, final_theta, total_dist = self.comm.get_pose()
        mission_status = "RETURN COMPLETED" if ok else "RETURN FAILED"

        print(f"\n[Nav] ========================================================")
        print(f"[Nav] Return Mission Finished: {mission_status}")
        print(f"[Nav] Final Pose: ({final_x:.2f}, {final_y:.2f}) m | Heading: {math.degrees(final_theta):.1f}° (Target: {target_theta_deg:+.1f}°)")
        print(f"[Nav] Total Travelled: {total_dist:.2f} m")
        print(f"[Nav] ========================================================\n")

        if save_plot:
            self.last_plot_path = save_navigation_plot(
                trajectory=self.last_trajectory,
                recorded_obstacles=self.last_obstacles,
                start_pos=(0.0, 0.0),
                goal_pos=forward_path[-1],
                planned_path=self.last_planned_path,
                final_pose=(final_x, final_y, final_theta),
                total_dist=total_dist,
                elapsed_s=0.0,
                target_theta=math.radians(target_theta_deg),
                status=mission_status,
                output_path=plot_path,
                grid=self.grid,
            )

        return ok

    def patrol(
        self,
        target_x: float,
        target_y: float,
        cycles: int = 1,
        speed_mps: float = DEFAULT_SPEED,
        target_theta_deg: Optional[float] = None,
        save_plot: bool = True,
        plot_path: str = DEFAULT_PLOT_PATH,
    ) -> bool:
        """
        Shuttles the robot back and forth between the start point (0, 0) and (target_x, target_y)
        for a specified number of cycles, following the exact cleared path on all legs in FORWARD motion.
        On arrival at destination or start, the robot aligns in-place to the exact orientation.

        Args:
            target_x: Target position along forward axis in meters
            target_y: Target position along lateral axis in meters
            cycles: Number of full round-trips (back and forth) to perform (default: 1)
            speed_mps: Cruising speed in m/s
            target_theta_deg: Target orientation at destination (e.g. 90 deg)
            save_plot: If True, saves visualization map after all cycles
            plot_path: Destination file path for map plot
        """
        cycles = max(1, int(cycles))
        print(f"\n[Nav] ========================================================")
        print(f"[Nav] STARTING PATROL MISSION: {cycles} Cycles (Back & Forth)")
        print(f"[Nav] Destination: ({target_x:.2f}, {target_y:.2f}) m | Target Heading: {target_theta_deg if target_theta_deg is not None else 'Any'}")
        print(f"[Nav] Total Planned Legs: {cycles * 2}")
        print(f"[Nav] ========================================================\n")

        # Cycle 1 - Outward Leg: Explore, plan, and establish safe collision-free path
        print(f"[Nav] >>> CYCLE 1/{cycles} - Leg 1/{cycles*2}: Start (0,0) -> Goal ({target_x:.2f}, {target_y:.2f}) [A* Mapping]")
        ok = self.navigate_to(
            target_x=target_x,
            target_y=target_y,
            speed_mps=speed_mps,
            target_theta_deg=target_theta_deg,
            return_to_start=False,
            save_plot=False,
        )
        if not ok or self.stop_requested:
            print("[Nav] Patrol aborted: Failed outward leg to destination.")
            if save_plot:
                fx, fy, ft, fd = self.comm.get_pose()
                self.last_plot_path = save_navigation_plot(
                    trajectory=self.last_trajectory,
                    recorded_obstacles=self.last_obstacles,
                    start_pos=(0.0, 0.0),
                    goal_pos=(target_x, target_y),
                    planned_path=self.last_planned_path,
                    final_pose=(fx, fy, ft),
                    total_dist=fd,
                    elapsed_s=0.0,
                    target_theta=math.radians(target_theta_deg) if target_theta_deg is not None else None,
                    status="PATROL_ABORTED",
                    output_path=plot_path,
                    grid=self.grid,
                )
            return False

        # Extract established exact path
        established_outward = simplify_trajectory(self.last_trajectory)
        established_return = list(reversed(established_outward))

        # Cycle 1 - Return Leg
        print(f"\n[Nav] >>> CYCLE 1/{cycles} - Leg 2/{cycles*2}: Goal ({target_x:.2f}, {target_y:.2f}) -> Start (0,0) [Exact Path]")
        ok = self.follow_path(
            established_return,
            speed_mps=speed_mps,
            target_theta_deg=0.0,
            label="Patrol C1 Return"
        )
        if not ok or self.stop_requested:
            print("[Nav] Patrol aborted on cycle 1 return leg.")
            if save_plot:
                fx, fy, ft, fd = self.comm.get_pose()
                self.last_plot_path = save_navigation_plot(
                    trajectory=self.last_trajectory,
                    recorded_obstacles=self.last_obstacles,
                    start_pos=(0.0, 0.0),
                    goal_pos=(target_x, target_y),
                    planned_path=self.last_planned_path,
                    final_pose=(fx, fy, ft),
                    total_dist=fd,
                    elapsed_s=0.0,
                    target_theta=math.radians(target_theta_deg) if target_theta_deg is not None else None,
                    status="PATROL_ABORTED",
                    output_path=plot_path,
                    grid=self.grid,
                )
            return False

        # Cycles 2 through N
        for c in range(2, cycles + 1):
            if self.stop_requested:
                break

            leg_out = (c - 1) * 2 + 1
            leg_ret = (c - 1) * 2 + 2

            print(f"\n[Nav] >>> CYCLE {c}/{cycles} - Leg {leg_out}/{cycles*2}: Start (0,0) -> Goal ({target_x:.2f}, {target_y:.2f}) [Exact Path]")
            ok = self.follow_path(
                established_outward,
                speed_mps=speed_mps,
                target_theta_deg=target_theta_deg,
                label=f"Patrol C{c} Outward"
            )
            if not ok or self.stop_requested:
                print(f"[Nav] Patrol interrupted during cycle {c} outward leg.")
                break

            print(f"\n[Nav] >>> CYCLE {c}/{cycles} - Leg {leg_ret}/{cycles*2}: Goal ({target_x:.2f}, {target_y:.2f}) -> Start (0,0) [Exact Path]")
            ok = self.follow_path(
                established_return,
                speed_mps=speed_mps,
                target_theta_deg=0.0,
                label=f"Patrol C{c} Return"
            )
            if not ok or self.stop_requested:
                print(f"[Nav] Patrol interrupted during cycle {c} return leg.")
                break

        mission_status = f"PATROL COMPLETED ({cycles} Cycles)" if not self.stop_requested else "STOPPED_BY_USER"
        final_x, final_y, final_theta, total_dist = self.comm.get_pose()

        print(f"\n[Nav] ========================================================")
        print(f"[Nav] Patrol Mission Finished: {mission_status}")
        print(f"[Nav] Final Pose: ({final_x:.2f}, {final_y:.2f}) m | Heading: {math.degrees(final_theta):.1f}°")
        print(f"[Nav] Total Travelled: {total_dist:.2f} m")
        print(f"[Nav] ========================================================\n")

        if save_plot:
            self.last_plot_path = save_navigation_plot(
                trajectory=self.last_trajectory,
                recorded_obstacles=self.last_obstacles,
                start_pos=(0.0, 0.0),
                goal_pos=(target_x, target_y),
                planned_path=self.last_planned_path,
                final_pose=(final_x, final_y, final_theta),
                total_dist=total_dist,
                elapsed_s=0.0,
                target_theta=math.radians(target_theta_deg) if target_theta_deg is not None else None,
                status=mission_status,
                output_path=plot_path,
                grid=self.grid,
            )

        return mission_status.startswith("PATROL COMPLETED")

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
