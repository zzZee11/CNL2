"""
grid_navigation/navigator.py
----------------------------
Deterministic state-machine navigator for discrete 2D grid navigation.
Supports forward-path retracing to start, multi-cycle patrol, dynamic obstacle replanning,
and high-precision terminal creep docking (<= 2.5 cm).
"""

import time
import math
from typing import List, Tuple, Optional

try:
    from .config import (
        CRUISE_SPEED, MIN_CREEP_SPEED, PIVOT_SPEED,
        WAYPOINT_TOLERANCE_M, TERMINAL_GOAL_TOLERANCE_M,
        FINAL_HEADING_TOLERANCE_DEG, KP_HEADING, KD_HEADING,
        OBSTACLE_DETECT_DIST_M, CRITICAL_STOP_DIST_M,
        TOF_ANGLES, WHEEL_BASE_M, DEFAULT_PLOT_PATH
    )
    from .grid_map import OccupancyGridMap
    from .planner import astar_search
    from .robot_hardware import RobotHardware
except ImportError:
    from config import (
        CRUISE_SPEED, MIN_CREEP_SPEED, PIVOT_SPEED,
        WAYPOINT_TOLERANCE_M, TERMINAL_GOAL_TOLERANCE_M,
        FINAL_HEADING_TOLERANCE_DEG, KP_HEADING, KD_HEADING,
        OBSTACLE_DETECT_DIST_M, CRITICAL_STOP_DIST_M,
        TOF_ANGLES, WHEEL_BASE_M, DEFAULT_PLOT_PATH
    )
    from grid_map import OccupancyGridMap
    from planner import astar_search
    from robot_hardware import RobotHardware


class GridNavigator:
    """
    State-machine autonomous navigator operating over discrete 2D occupancy grid.
    """

    def __init__(self, hardware: RobotHardware, grid_map: Optional[OccupancyGridMap] = None):
        self.hw = hardware
        self.grid = grid_map if grid_map is not None else OccupancyGridMap()
        self.stop_requested = False

        # Mission telemetry history
        self.last_trajectory: List[Tuple[float, float]] = [(0.0, 0.0)]
        self.last_planned_path: List[Tuple[float, float]] = []
        self.last_obstacles: List[Tuple[float, float]] = []
        self.last_plot_path: Optional[str] = None

    def clear_map(self) -> None:
        """Clears all mapped obstacles from occupancy grid."""
        self.grid.clear()
        self.last_obstacles.clear()
        print("[GridNav] Occupancy grid map cleared.")

    def navigate_to(
        self,
        target_x: float,
        target_y: float,
        target_theta_deg: Optional[float] = None,
        speed_mps: float = CRUISE_SPEED,
        return_to_start: bool = False,
        reset_pose: bool = True,
        save_plot: bool = True,
        plot_path: str = DEFAULT_PLOT_PATH
    ) -> bool:
        """
        Navigates the robot to (target_x, target_y) in meters from (0, 0)
        using A* grid path planning, dynamic obstacle avoidance, and terminal docking.
        Optionally aligns to target_theta_deg upon arrival and retraces back to start.
        """
        self.stop_requested = False
        target_x = float(target_x)
        target_y = float(target_y)
        speed_mps = max(0.10, min(0.80, float(speed_mps)))

        if reset_pose:
            # Reset odometry to (0, 0) facing +X axis at mission start
            self.hw.reset_pose(0.0, 0.0, 0.0)
            self.last_trajectory = [(0.0, 0.0)]
        else:
            cur_x, cur_y, _, _ = self.hw.get_pose()
            self.last_trajectory = [(cur_x, cur_y)]

        self.last_obstacles = self.grid.get_raw_obstacles_world()

        print(f"\n[GridNav] ========================================================")
        print(f"[GridNav] MISSION: Navigate to ({target_x:.2f}, {target_y:.2f}) m")
        print(f"[GridNav] Target Heading: {target_theta_deg if target_theta_deg is not None else 'Any'} | Speed: {speed_mps:.2f} m/s")
        print(f"[GridNav] ========================================================\n")

        # Initial A* path planning
        cur_x, cur_y, cur_th, _ = self.hw.get_pose()
        waypoints = astar_search((cur_x, cur_y), (target_x, target_y), self.grid, smooth=True)
        if not waypoints:
            print("[GridNav] Error: Could not compute a collision-free path to goal.")
            return False

        self.last_planned_path = list(waypoints)
        print(f"[GridNav] Route planned with {len(waypoints)} waypoints.")

        # Follow the planned path
        success = self._follow_waypoint_path(
            waypoints=waypoints,
            speed_mps=speed_mps,
            label="Outward Mission"
        )

        if not success or self.stop_requested:
            self.hw.active_brake()
            return False

        # Terminal precision docking onto exact target coordinate
        self._fine_dock(target_x, target_y)

        # Final orientation alignment if specified
        if target_theta_deg is not None and not self.stop_requested:
            self._align_orientation(target_theta_deg)

        # Save visualization plot
        if save_plot:
            self._generate_plot(plot_path=plot_path, status="GOAL REACHED")

        # Retrace path back to start if requested
        if return_to_start and not self.stop_requested:
            print("\n[GridNav] Destination reached! Retracing exact path forward back to start...")
            return self.return_to_start(
                speed_mps=speed_mps,
                target_theta_deg=0.0,
                save_plot=save_plot,
                plot_path=plot_path
            )

        return True

    def return_to_start(
        self,
        speed_mps: float = CRUISE_SPEED,
        target_theta_deg: float = 0.0,
        save_plot: bool = True,
        plot_path: str = DEFAULT_PLOT_PATH
    ) -> bool:
        """
        Retraces the exact recorded trajectory forward back to (0.0, 0.0),
        pivoting 180° in place at the destination first, and restores initial heading (0.0°).
        """
        if len(self.last_trajectory) < 2:
            print("[GridNav] Error: No recorded trajectory history to follow back.")
            return False

        # Re-zero gyro drift bias during stationary pause at goal
        self.hw.recalibrate_gyro_bias(num_samples=25)

        # Reverse recorded trajectory points to form return route
        return_waypoints = list(reversed(self.last_trajectory))
        # Downsample waypoints to clean spacing
        return_waypoints = self._downsample_waypoints(return_waypoints, min_spacing_m=0.15)
        # Ensure exact destination is (0.0, 0.0)
        return_waypoints[-1] = (0.0, 0.0)

        print(f"\n[GridNav] ========================================================")
        print(f"[GridNav] RETRACING EXACT PATH BACK TO START POINT (0.00, 0.00)")
        print(f"[GridNav] Return Waypoints: {len(return_waypoints)} | Target Orientation: {target_theta_deg:+.1f}°")
        print(f"[GridNav] Mode: FORWARD MOTION (Pivoting in place at goal first)")
        print(f"[GridNav] ========================================================\n")

        success = self._follow_waypoint_path(
            waypoints=return_waypoints,
            speed_mps=speed_mps,
            label="Return to Start"
        )

        if not success or self.stop_requested:
            self.hw.active_brake()
            return False

        # Terminal precision docking onto exact (0.0, 0.0)
        self._fine_dock(0.0, 0.0)

        # Final orientation alignment back to exact start orientation (0.0°)
        self._align_orientation(target_theta_deg)

        if save_plot:
            self._generate_plot(plot_path=plot_path, status="RETURN COMPLETED")

        final_x, final_y, final_th, total_dist = self.hw.get_pose()
        print(f"\n[GridNav] Return Mission Finished: RETURN COMPLETED")
        print(f"[GridNav] Final Pose: ({final_x:.2f}, {final_y:.2f}) m | Heading: {math.degrees(final_th):.1f}°")
        print(f"[GridNav] Total Travelled: {total_dist:.2f} m\n")
        return True

    def patrol(
        self,
        target_x: float,
        target_y: float,
        cycles: int = 1,
        target_theta_deg: Optional[float] = None,
        speed_mps: float = CRUISE_SPEED,
        save_plot: bool = True,
        plot_path: str = DEFAULT_PLOT_PATH
    ) -> bool:
        """
        Shuttles the robot back and forth between (0, 0) and (target_x, target_y)
        for N full cycles along the cleared path.
        """
        cycles = max(1, int(cycles))
        print(f"\n[GridNav] ========================================================")
        print(f"[GridNav] STARTING PATROL MISSION: {cycles} Cycles (Back & Forth)")
        print(f"[GridNav] Destination: ({target_x:.2f}, {target_y:.2f}) m | Total Planned Legs: {cycles * 2}")
        print(f"[GridNav] ========================================================\n")

        # Leg 1: Outward navigation and mapping
        print(f"[GridNav] >>> CYCLE 1/{cycles} - Leg 1/{cycles*2}: Start (0,0) -> Goal ({target_x:.2f}, {target_y:.2f})")
        ok = self.navigate_to(
            target_x=target_x,
            target_y=target_y,
            target_theta_deg=target_theta_deg,
            speed_mps=speed_mps,
            return_to_start=False,
            save_plot=False
        )
        if not ok or self.stop_requested:
            return False

        # Leg 2: Return leg
        print(f"\n[GridNav] >>> CYCLE 1/{cycles} - Leg 2/{cycles*2}: Goal ({target_x:.2f}, {target_y:.2f}) -> Start (0,0)")
        ok = self.return_to_start(
            speed_mps=speed_mps,
            target_theta_deg=0.0,
            save_plot=False
        )
        if not ok or self.stop_requested:
            return False

        # Subsequent cycles
        for c in range(2, cycles + 1):
            if self.stop_requested:
                break

            leg_out = (c - 1) * 2 + 1
            leg_ret = (c - 1) * 2 + 2

            print(f"\n[GridNav] >>> CYCLE {c}/{cycles} - Leg {leg_out}/{cycles*2}: Start (0,0) -> Goal ({target_x:.2f}, {target_y:.2f})")
            ok = self.navigate_to(
                target_x=target_x,
                target_y=target_y,
                target_theta_deg=target_theta_deg,
                speed_mps=speed_mps,
                return_to_start=False,
                reset_pose=False,
                save_plot=False
            )
            if not ok or self.stop_requested:
                break

            print(f"\n[GridNav] >>> CYCLE {c}/{cycles} - Leg {leg_ret}/{cycles*2}: Goal ({target_x:.2f}, {target_y:.2f}) -> Start (0,0)")
            ok = self.return_to_start(
                speed_mps=speed_mps,
                target_theta_deg=0.0,
                save_plot=False
            )
            if not ok or self.stop_requested:
                break

        if save_plot:
            self._generate_plot(plot_path=plot_path, status=f"PATROL COMPLETED ({cycles} Cycles)")

        return not self.stop_requested

    def request_stop(self) -> None:
        """Signals active mission to halt immediately."""
        self.stop_requested = True
        self.hw.stop()

    # -------------------------------------------------------------
    # INTERNAL WAYPOINT TRACKING & DOCKING METHODS
    # -------------------------------------------------------------
    def _follow_waypoint_path(
        self,
        waypoints: List[Tuple[float, float]],
        speed_mps: float,
        label: str = "Nav"
    ) -> bool:
        """
        Follows a sequence of metric waypoints forward with in-place initial alignment,
        dynamic obstacle replanning, and multi-stage deceleration.
        """
        if not waypoints:
            return True

        wps = list(waypoints)
        goal_pos = wps[-1]

        # 1. Pivot in-place initially to face the path heading
        cur_x, cur_y, cur_th, _ = self.hw.get_pose()
        dx = wps[1][0] - cur_x if len(wps) > 1 else wps[0][0] - cur_x
        dy = wps[1][1] - cur_y if len(wps) > 1 else wps[0][1] - cur_y
        if math.hypot(dx, dy) > 0.05:
            init_heading_deg = math.degrees(math.atan2(dy, dx))
            self._align_orientation(init_heading_deg)

        last_log_time = 0.0

        while not self.stop_requested and wps:
            cur_x, cur_y, cur_th, _ = self.hw.get_pose()

            # Record trajectory history if moved > 2 cm
            last_pt = self.last_trajectory[-1]
            if math.hypot(cur_x - last_pt[0], cur_y - last_pt[1]) >= 0.02:
                self.last_trajectory.append((cur_x, cur_y))

            dist_to_goal = math.hypot(goal_pos[0] - cur_x, goal_pos[1] - cur_y)
            if dist_to_goal <= TERMINAL_GOAL_TOLERANCE_M:
                self.hw.active_brake()
                print(f"[GridNav] [{label}] Arrived at goal ({dist_to_goal*100.0:.1f} cm <= {TERMINAL_GOAL_TOLERANCE_M*100.0:.1f} cm tolerance).")
                return True

            # 2. Check live ToF sensors for obstacles
            tof_readings = self.hw.get_tof_distances()
            obstacle_detected = False
            for i, d in enumerate(tof_readings):
                if d <= OBSTACLE_DETECT_DIST_M:
                    # Stationary verification: Halt and verify reading to reject false optical spikes
                    self.hw.stop()
                    time.sleep(0.08)
                    fresh = self.hw.get_tof_distances()
                    if fresh[i] <= (OBSTACLE_DETECT_DIST_M + 0.05):
                        # Confirmed obstacle! Calculate world coordinate and mark into grid
                        obs_angle = cur_th + TOF_ANGLES[i]
                        obs_x = cur_x + fresh[i] * math.cos(obs_angle)
                        obs_y = cur_y + fresh[i] * math.sin(obs_angle)
                        self.grid.add_obstacle(obs_x, obs_y)
                        self.last_obstacles = self.grid.get_raw_obstacles_world()
                        print(f"\n[GridNav] [{label}] Obstacle detected at ({obs_x:.2f}, {obs_y:.2f}) m! Replanning A* route...")
                        obstacle_detected = True
                        break

            if obstacle_detected:
                # Replan from current position to ultimate goal
                new_path = astar_search((cur_x, cur_y), goal_pos, self.grid, smooth=True)
                if not new_path:
                    print(f"[GridNav] [{label}] BLOCKED: No route to goal around obstacles.")
                    self.hw.active_brake()
                    return False
                wps = new_path
                self.last_planned_path = list(wps)
                # Pivot to face the new replanned heading
                if len(wps) > 1:
                    ndx = wps[1][0] - cur_x
                    ndy = wps[1][1] - cur_y
                    self._align_orientation(math.degrees(math.atan2(ndy, ndx)))
                continue

            # 3. Advance waypoints
            while len(wps) > 1:
                wp_dist = math.hypot(wps[0][0] - cur_x, wps[0][1] - cur_y)
                pop_thresh = min(WAYPOINT_TOLERANCE_M, max(0.06, dist_to_goal * 0.45))
                if wp_dist < pop_thresh:
                    wps.pop(0)
                else:
                    break

            target_wp = wps[0]

            # 4. Multi-stage speed calculation
            if dist_to_goal >= 0.35:
                cmd_speed = speed_mps
            elif dist_to_goal >= 0.12:
                # Smooth deceleration
                ratio = (dist_to_goal - 0.12) / (0.35 - 0.12)
                cmd_speed = 0.10 + (speed_mps - 0.10) * ratio
            else:
                # Precision terminal creep
                ratio = max(0.0, (dist_to_goal - TERMINAL_GOAL_TOLERANCE_M) / (0.12 - TERMINAL_GOAL_TOLERANCE_M))
                cmd_speed = MIN_CREEP_SPEED + (0.10 - MIN_CREEP_SPEED) * ratio

            # 5. Course heading control
            target_bearing = math.atan2(target_wp[1] - cur_y, target_wp[0] - cur_x)
            heading_err = math.atan2(math.sin(target_bearing - cur_th), math.cos(target_bearing - cur_th))

            # If heading error is large (> 40 deg), pivot gently in place
            if abs(heading_err) > math.radians(40.0):
                turn_spd = PIVOT_SPEED
                if heading_err > 0:
                    self.hw.set_speeds(-turn_spd, +turn_spd)
                else:
                    self.hw.set_speeds(+turn_spd, -turn_spd)
                time.sleep(0.04)
                continue

            yaw_rate = self.hw.get_yaw_rate()
            omega = (KP_HEADING * heading_err) - (KD_HEADING * yaw_rate)
            vl = cmd_speed - (omega * WHEEL_BASE_M / 2.0)
            vr = cmd_speed + (omega * WHEEL_BASE_M / 2.0)
            self.hw.set_speeds(vl, vr)

            now = time.time()
            if now - last_log_time > 0.4:
                print(f"  [{label}] Pos: ({cur_x:5.2f}, {cur_y:5.2f})m | Rem: {dist_to_goal:4.2f}m | Hdg: {math.degrees(cur_th):+5.1f}° | Cmd: ({vl:+.2f}, {vr:+.2f}) m/s")
                last_log_time = now

            time.sleep(0.04)

        self.hw.active_brake()
        return not self.stop_requested

    def _fine_dock(self, target_x: float, target_y: float, timeout_s: float = 3.5) -> bool:
        """
        Nudges the robot forward at creeping speed (0.04 m/s) directly onto (target_x, target_y)
        until remaining distance <= TERMINAL_GOAL_TOLERANCE_M (2.5 cm).
        """
        cur_x, cur_y, _, _ = self.hw.get_pose()
        dist_init = math.hypot(target_x - cur_x, target_y - cur_y)
        if dist_init <= TERMINAL_GOAL_TOLERANCE_M:
            self.hw.active_brake()
            return True

        t0 = time.time()
        while not self.stop_requested and (time.time() - t0 < timeout_s):
            cur_x, cur_y, cur_th, _ = self.hw.get_pose()
            dx = target_x - cur_x
            dy = target_y - cur_y
            dist = math.hypot(dx, dy)

            if dist <= TERMINAL_GOAL_TOLERANCE_M:
                self.hw.active_brake()
                print(f"[GridNav] Fine docking achieved: remaining distance {dist*100.0:.1f} cm (<= {TERMINAL_GOAL_TOLERANCE_M*100.0:.1f} cm).")
                return True

            target_bearing = math.atan2(dy, dx)
            bearing_err = math.atan2(math.sin(target_bearing - cur_th), math.cos(target_bearing - cur_th))

            # Creep speed
            creep_spd = max(MIN_CREEP_SPEED, min(0.08, dist * 0.75))
            omega = 0.5 * bearing_err
            vl = creep_spd - (omega * WHEEL_BASE_M / 2.0)
            vr = creep_spd + (omega * WHEEL_BASE_M / 2.0)
            self.hw.set_speeds(vl, vr)
            time.sleep(0.04)

        self.hw.active_brake()
        return True

    def _align_orientation(self, target_deg: float, timeout_s: float = 5.0) -> bool:
        """Pivots the robot in-place to align with target_deg (degrees)."""
        target_rad = math.radians(target_deg)
        target_rad = math.atan2(math.sin(target_rad), math.cos(target_rad))

        self.hw.stop()
        time.sleep(0.05)
        t0 = time.time()

        while not self.stop_requested and (time.time() - t0 < timeout_s):
            _, _, cur_th, _ = self.hw.get_pose()
            err = math.atan2(math.sin(target_rad - cur_th), math.cos(target_rad - cur_th))
            err_deg = math.degrees(err)

            if abs(err_deg) <= FINAL_HEADING_TOLERANCE_DEG:
                self.hw.stop()
                print(f"[GridNav] Orientation aligned to {math.degrees(cur_th):+.1f}° (Target: {target_deg:+.1f}°, Error: {err_deg:+.1f}°).")
                return True

            spd = PIVOT_SPEED if abs(err_deg) > 10.0 else max(0.08, PIVOT_SPEED * (abs(err_deg) / 10.0))
            if err > 0:
                self.hw.set_speeds(-spd, +spd)
            else:
                self.hw.set_speeds(+spd, -spd)
            time.sleep(0.04)

        self.hw.stop()
        return False

    def _downsample_waypoints(self, pts: List[Tuple[float, float]], min_spacing_m: float = 0.15) -> List[Tuple[float, float]]:
        """Downsamples dense trajectory points to clean spaced waypoints."""
        if len(pts) <= 2:
            return list(pts)
        out = [pts[0]]
        for p in pts[1:-1]:
            if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) >= min_spacing_m:
                out.append(p)
        out.append(pts[-1])
        return out

    def _generate_plot(self, plot_path: str, status: str) -> None:
        """Internal helper to generate visualization map."""
        try:
            try:
                from .visualizer import export_navigation_map
            except ImportError:
                from visualizer import export_navigation_map
            fx, fy, fth, fdist = self.hw.get_pose()
            self.last_plot_path = export_navigation_map(
                trajectory=self.last_trajectory,
                planned_path=self.last_planned_path,
                obstacles=self.grid.get_raw_obstacles_world(),
                final_pose=(fx, fy, fth),
                total_dist=fdist,
                status=status,
                output_path=plot_path
            )
        except Exception as e:
            print(f"[GridNav] Note: Map visualization export skipped ({e}).")
