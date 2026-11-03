#!/usr/bin/env python3
"""
autonomous_navigation/main.py
------------------------------
Primary entry point for 2D Cartesian plane & distance navigation with dynamic obstacle avoidance.

Usage Examples:
  # Navigate to Cartesian coordinate (1.0, 5.0) in meters at 0.7 m/s:
  python3 autonomous_navigation/main.py --goal 1.0 5.0 --speed 0.7

  # Move straight forward 5 meters (legacy distance mode):
  python3 autonomous_navigation/main.py --distance 5.0 --speed 0.8

  # Interactive CLI shell (type 'goto 1 5' or 'move 3 m'):
  python3 autonomous_navigation/main.py

  # Test in simulation mode (runs on PC without hardware, generates PNG plot):
  python3 autonomous_navigation/main.py --mock --goal 1.0 5.0 --speed 0.6
"""

import os
import sys
import time
import math
import re
import argparse

# Ensure both parent directory and local directory are in sys.path for direct execution
pkg_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(pkg_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
if pkg_dir not in sys.path:
    sys.path.insert(0, pkg_dir)

try:
    from .config import SERIAL_PORT, SERIAL_BAUD, DEFAULT_SPEED, DEFAULT_PLOT_PATH, IMU_I2C_BUS, IMU_I2C_ADDR
    from .sensors import DistanceSensorManager
    from .serial_comm import RobotSerialComm
    from .navigator import AutonomousNavigator
    from .plotter import save_navigation_plot
    from .imu import IMUManager
    from .calibration import run_interactive_calibration
except ImportError:
    from config import SERIAL_PORT, SERIAL_BAUD, DEFAULT_SPEED, DEFAULT_PLOT_PATH, IMU_I2C_BUS, IMU_I2C_ADDR
    from sensors import DistanceSensorManager
    from serial_comm import RobotSerialComm
    from navigator import AutonomousNavigator
    from plotter import save_navigation_plot
    from calibration import run_interactive_calibration
    try:
        from imu import IMUManager
    except ImportError:
        IMUManager = None


def parse_args():
    parser = argparse.ArgumentParser(
        description="Autonomous 2D Cartesian & Distance Navigator with Obstacle Avoidance"
    )
    parser.add_argument(
        "--goal", "-g", nargs="+", type=float, default=None,
        help="Target Cartesian coordinate in meters, with optional target orientation in degrees (e.g. --goal 1.0 5.0 or --goal 1.0 5.0 90)"
    )
    parser.add_argument(
        "--heading", "--theta", type=float, default=None,
        help="Target orientation at destination in degrees (e.g. --heading 90, --heading -45)"
    )
    parser.add_argument(
        "--patrol", "--cycles", type=int, default=None, metavar="N",
        help="Patrol/shuttle back and forth between start (0, 0) and goal N times along exact path"
    )
    parser.add_argument(
        "--return", "--roundtrip", dest="return_to_start", action="store_true",
        help="Automatically retrace exact path forward back to start point after reaching goal"
    )
    parser.add_argument(
        "--distance", "-d", type=float, default=None,
        help="Linear forward distance to travel in meters (e.g. --distance 5.0)"
    )
    parser.add_argument(
        "--speed", "-s", type=float, default=DEFAULT_SPEED,
        help=f"Target cruise speed in m/s (default: {DEFAULT_SPEED})"
    )
    parser.add_argument(
        "--plot", type=str, default=DEFAULT_PLOT_PATH,
        help=f"Destination path for trajectory map plot (default: {DEFAULT_PLOT_PATH})"
    )
    parser.add_argument(
        "--no-plot", action="store_true",
        help="Disable automatic plot generation after mission"
    )
    parser.add_argument(
        "--port", "-p", default=SERIAL_PORT,
        help=f"Serial port for ESP32 (default: {SERIAL_PORT})"
    )
    parser.add_argument(
        "--baud", "-b", type=int, default=SERIAL_BAUD,
        help=f"Serial baud rate (default: {SERIAL_BAUD})"
    )
    parser.add_argument(
        "--calibrate", action="store_true",
        help="Run interactive physical odometry calibration wizard (wheel radius and track width)"
    )
    parser.add_argument(
        "--mock", action="store_true",
        help="Run in mock/simulation mode without physical hardware"
    )
    return parser.parse_args()


def parse_goto_input(text: str):
    """
    Parses various user coordinate and orientation inputs like:
      'goto 1 1'
      'goto 1.0 2.0 90 deg'
      'goto 1 1 heading 90'
      'goto 1 1 @ 90'
      'goto (1.0, 5.0) 0.6 m/s 90 deg'
      'goto 1 2 90'
      'goto 1 2 0.5'
      'goto 1 1 --return'
    Returns: (target_x, target_y, speed_or_none, target_theta_deg_or_none, return_to_start_bool) or None
    """
    cleaned = text.strip()
    if cleaned.lower().startswith("goto") or cleaned.lower().startswith("navigate"):
        cleaned = re.sub(r"^(goto|navigate)\s*", "", cleaned, flags=re.IGNORECASE).strip()

    target_theta = None
    speed = None
    return_to_start = False

    # Check for return / roundtrip flag
    if re.search(r"\b(return|roundtrip|round-trip|back)\b", cleaned, re.IGNORECASE):
        return_to_start = True
        cleaned = re.sub(r"\b(return|roundtrip|round-trip|back)\b", "", cleaned, flags=re.IGNORECASE).strip()

    # 1. Check for explicit heading/deg/theta: e.g. "heading 90", "theta -45", "90 deg", "@ 90"
    m_hdg = re.search(r"(?:heading|theta|@)\s*([+-]?\d*\.?\d+)", cleaned, re.IGNORECASE)
    if m_hdg:
        try:
            target_theta = float(m_hdg.group(1))
            cleaned = cleaned[:m_hdg.start()] + " " + cleaned[m_hdg.end():]
        except ValueError:
            pass
    else:
        m_deg = re.search(r"([+-]?\d*\.?\d+)\s*(?:deg|degrees|°)", cleaned, re.IGNORECASE)
        if m_deg:
            try:
                target_theta = float(m_deg.group(1))
                cleaned = cleaned[:m_deg.start()] + " " + cleaned[m_deg.end():]
            except ValueError:
                pass

    # 2. Check for explicit speed: e.g. "speed 0.6", "0.6 m/s"
    m_spd = re.search(r"(?:speed)\s*([+-]?\d*\.?\d+)", cleaned, re.IGNORECASE)
    if m_spd:
        try:
            speed = float(m_spd.group(1))
            cleaned = cleaned[:m_spd.start()] + " " + cleaned[m_spd.end():]
        except ValueError:
            pass
    else:
        m_mps = re.search(r"([+-]?\d*\.?\d+)\s*(?:m/s|mps)", cleaned, re.IGNORECASE)
        if m_mps:
            try:
                speed = float(m_mps.group(1))
                cleaned = cleaned[:m_mps.start()] + " " + cleaned[m_mps.end():]
            except ValueError:
                pass

    # 3. Extract remaining numeric arguments
    nums = re.findall(r"[-+]?\d*\.?\d+", cleaned)
    if len(nums) < 2:
        return None

    try:
        gx = float(nums[0])
        gy = float(nums[1])

        # Positional leftovers
        if len(nums) >= 3 and speed is None and target_theta is None:
            v = float(nums[2])
            # If magnitude > 2.0, it is an orientation in degrees (e.g. 90, 180, -45)
            # If magnitude <= 2.0, it is a cruising speed in m/s (e.g. 0.4, 0.7)
            if abs(v) > 2.0:
                target_theta = v
            else:
                speed = v

            if len(nums) >= 4:
                if target_theta is None:
                    target_theta = float(nums[3])
                elif speed is None:
                    speed = float(nums[3])

        return gx, gy, speed, target_theta, return_to_start
    except ValueError:
        return None


def parse_patrol_input(text: str):
    """
    Parses patrol / shuttle commands like:
      'patrol 1 1 3'
      'patrol 1 2 5 90 deg'
      'shuttle 1.0 1.0 2'
      'patrol (1.0, 2.0) 3 speed 0.5 heading 90'
    Returns: (target_x, target_y, cycles, speed_or_none, target_theta_or_none) or None
    """
    cleaned = text.strip()
    if not (cleaned.lower().startswith("patrol") or cleaned.lower().startswith("shuttle")):
        return None
    cleaned = re.sub(r"^(patrol|shuttle)\s*", "", cleaned, flags=re.IGNORECASE).strip()

    target_theta = None
    speed = None

    # Check for explicit heading/deg
    m_hdg = re.search(r"(?:heading|theta|@)\s*([+-]?\d*\.?\d+)", cleaned, re.IGNORECASE)
    if m_hdg:
        try:
            target_theta = float(m_hdg.group(1))
            cleaned = cleaned[:m_hdg.start()] + " " + cleaned[m_hdg.end():]
        except ValueError:
            pass
    else:
        m_deg = re.search(r"([+-]?\d*\.?\d+)\s*(?:deg|degrees|°)", cleaned, re.IGNORECASE)
        if m_deg:
            try:
                target_theta = float(m_deg.group(1))
                cleaned = cleaned[:m_deg.start()] + " " + cleaned[m_deg.end():]
            except ValueError:
                pass

    # Check for explicit speed
    m_spd = re.search(r"(?:speed)\s*([+-]?\d*\.?\d+)", cleaned, re.IGNORECASE)
    if m_spd:
        try:
            speed = float(m_spd.group(1))
            cleaned = cleaned[:m_spd.start()] + " " + cleaned[m_spd.end():]
        except ValueError:
            pass
    else:
        m_mps = re.search(r"([+-]?\d*\.?\d+)\s*(?:m/s|mps)", cleaned, re.IGNORECASE)
        if m_mps:
            try:
                speed = float(m_mps.group(1))
                cleaned = cleaned[:m_mps.start()] + " " + cleaned[m_mps.end():]
            except ValueError:
                pass

    nums = re.findall(r"[-+]?\d*\.?\d+", cleaned)
    if len(nums) < 2:
        return None

    try:
        gx = float(nums[0])
        gy = float(nums[1])
        cycles = int(float(nums[2])) if len(nums) >= 3 else 1

        if len(nums) >= 4 and speed is None and target_theta is None:
            v = float(nums[3])
            if abs(v) > 2.0:
                target_theta = v
            else:
                speed = v

        return gx, gy, cycles, speed, target_theta
    except ValueError:
        return None


def run_interactive(nav, sensors, comm, default_plot_path=DEFAULT_PLOT_PATH):
    print("\n" + "=" * 70)
    print("AUTONOMOUS CARTESIAN NAVIGATOR (A* Global + APF Vector Avoidance)")
    print("=" * 70)
    print("Navigation Commands:")
    print("  goto <X> <Y> [heading] [speed] [--return]   Navigate to coordinate (e.g. 'goto 1 1 90 deg', 'goto 1 1 --return')")
    print("  return / back / home                       Retrace exact path forward back to start & align to start orientation")
    print("  patrol <X> <Y> <cycles> [heading] [speed]  Move back & forth N times along exact path (e.g. 'patrol 1 1 3')")
    print("  move <dist> m [speed]                      Legacy forward drive (e.g. 'move 5 m 0.8' or 'move 2 m')")
    print("  plot [filepath]                            Save visualization map of the latest trajectory and obstacles")
    print("  sensors / pose / imu                       Display real-time telemetry")
    print("  stop / quit                                Emergency halt / Exit")
    print("=" * 70 + "\n")

    while True:
        try:
            line = input("nav> ").strip()
        except (KeyboardInterrupt, EOFError):
            break

        if not line:
            continue

        cmd_lower = line.lower()

        if cmd_lower in ("quit", "q", "exit"):
            break

        if cmd_lower in ("stop", "s", "brake"):
            nav.request_stop()
            print("[Nav] Robot halted.")
            continue

        if cmd_lower in ("clear", "reset-map", "clear-obstacles"):
            nav.clear_obstacles()
            continue

        if cmd_lower in ("sensors", "tof"):
            d = sensors.get_distances()
            print(f"[Sensors] Left: {d[0]:.2f}m | Middle: {d[1]:.2f}m | Right: {d[2]:.2f}m")
            continue

        if cmd_lower in ("pose", "odom"):
            x, y, theta, dist = comm.get_pose()
            print(f"[Pose] X: {x:.2f}m | Y: {y:.2f}m | Heading: {math.degrees(theta):.1f}° | Travelled: {dist:.2f}m")
            continue

        if cmd_lower in ("imu", "gyro"):
            if comm.imu and comm.imu.is_ready():
                hdg = comm.imu.get_heading_deg()
                rate = comm.imu.get_yaw_rate_deg_s()
                ax, ay, az = comm.imu.get_linear_accel()
                pitch, roll = comm.imu.get_tilt_deg()
                print(f"[IMU] Heading: {hdg:+5.1f}° | TurnRate: {rate:+5.1f}°/s | Accel: [{ax:+4.2f}, {ay:+4.2f}, {az:+4.2f}] m/s² | Tilt: Pitch={pitch:+.1f}°, Roll={roll:+.1f}°")
            else:
                print("[IMU] IMU not connected or in mock mode.")
            continue

        # Calibration wizard
        if cmd_lower in ("calibrate", "cal"):
            run_interactive_calibration(comm, nav)
            continue

        # Plot command
        if cmd_lower.startswith("plot"):
            parts = line.split(maxsplit=1)
            out_file = parts[1].strip() if len(parts) > 1 else default_plot_path
            final_x, final_y, final_theta, final_dist = comm.get_pose()
            saved = save_navigation_plot(
                trajectory=nav.last_trajectory,
                recorded_obstacles=nav.last_obstacles,
                start_pos=(0.0, 0.0),
                goal_pos=(final_x, final_y),
                planned_path=nav.last_planned_path,
                final_pose=(final_x, final_y, final_theta),
                total_dist=final_dist,
                elapsed_s=0.0,
                status="MANUAL_SAVE",
                output_path=out_file,
            )
            print(f"[Plot] Image saved to: {saved}")
            continue

        # Return to Start (Retrace path back to start point)
        if cmd_lower in ("return", "back", "home", "retrace"):
            nav.return_to_start(speed_mps=DEFAULT_SPEED, target_theta_deg=0.0, save_plot=True, plot_path=default_plot_path)
            continue

        # Patrol / Shuttle between Start and Goal N cycles
        patrol_match = parse_patrol_input(line)
        if (cmd_lower.startswith("patrol") or cmd_lower.startswith("shuttle")) and patrol_match:
            gx, gy, cycles, spd, target_th = patrol_match
            chosen_speed = spd if spd is not None else DEFAULT_SPEED
            nav.patrol(
                target_x=gx,
                target_y=gy,
                cycles=cycles,
                speed_mps=chosen_speed,
                target_theta_deg=target_th,
                save_plot=True,
                plot_path=default_plot_path
            )
            continue

        # Goto Cartesian coordinate (e.g. 'goto 1 1 90 deg', 'goto 1 5', 'goto 1.0 5.0 0.8')
        goto_match = parse_goto_input(line)
        if (cmd_lower.startswith("goto") or cmd_lower.startswith("navigate")) and goto_match:
            gx, gy, spd, target_th, ret_to_start = goto_match
            chosen_speed = spd if spd is not None else DEFAULT_SPEED
            nav.navigate_to(
                gx, gy,
                speed_mps=chosen_speed,
                target_theta_deg=target_th,
                return_to_start=ret_to_start,
                save_plot=True,
                plot_path=default_plot_path
            )
            continue

        # Legacy Move command (e.g. 'move 5 m 0.8' or 'move 2 m')
        parts = line.split()
        if len(parts) >= 2 and parts[0].lower() == "move":
            try:
                distance = float(parts[1])
                speed = DEFAULT_SPEED
                if len(parts) >= 3 and parts[2].lower() == "m":
                    if len(parts) >= 4:
                        speed = float(parts[3])
                elif len(parts) >= 3:
                    speed = float(parts[2])

                nav.move_distance(distance, speed_mps=speed, save_plot=True, plot_path=default_plot_path)
            except ValueError:
                print("Invalid format. Use: 'move 5 m 0.8' or 'goto 1 5'")
            continue

        print("Unknown command. Try: 'goto 1 5', 'move 5 m', 'plot', 'sensors', 'pose', or 'quit'.")


def main():
    args = parse_args()

    sensors = DistanceSensorManager(use_mock=args.mock)
    sensors.initialize()
    sensors.start()

    # Initialize MPU-6050 spatial IMU
    imu = None
    if IMUManager:
        try:
            candidate_imu = IMUManager(bus_num=IMU_I2C_BUS, address=IMU_I2C_ADDR, use_mock=args.mock)
            if candidate_imu.initialize():
                candidate_imu.start()
                imu = candidate_imu
                print("[Main] MPU-6050 spatial IMU fused into autonomous navigation!")
        except Exception as e:
            print(f"[Main] IMU initialization warning: {e}. Using pure encoder odometry.")
            imu = None

    comm = RobotSerialComm(port=args.port, baud=args.baud, use_mock=args.mock, imu=imu)
    if not comm.connect():
        if imu:
            imu.stop()
        sensors.close()
        sys.exit(1)
    comm.start()

    nav = AutonomousNavigator(serial_comm=comm, sensor_mgr=sensors)

    try:
        if getattr(args, "calibrate", False):
            run_interactive_calibration(comm, nav)
            return

        if args.goal is not None:
            # 2D Cartesian target navigation: --goal X Y [THETA]
            if len(args.goal) == 1:
                print("[Main] Error: --goal requires at least 2 arguments: X Y (e.g. --goal 1.0 1.0)")
                return
            gx = args.goal[0]
            gy = args.goal[1]
            target_th = args.goal[2] if len(args.goal) >= 3 else None
            if args.heading is not None:
                target_th = args.heading

            if args.patrol is not None:
                # Patrol mode between (0, 0) and (gx, gy) for N cycles
                nav.patrol(
                    target_x=gx,
                    target_y=gy,
                    cycles=args.patrol,
                    speed_mps=args.speed,
                    target_theta_deg=target_th,
                    save_plot=not args.no_plot,
                    plot_path=args.plot
                )
            else:
                nav.navigate_to(
                    target_x=gx,
                    target_y=gy,
                    speed_mps=args.speed,
                    target_theta_deg=target_th,
                    return_to_start=args.return_to_start,
                    save_plot=not args.no_plot,
                    plot_path=args.plot
                )
        elif args.patrol is not None:
            print("[Main] Error: --patrol requires a destination coordinate: --goal X Y (e.g. --goal 1.0 1.0 --patrol 3)")
            return
        elif args.distance is not None:
            # Linear forward distance navigation: --distance D
            nav.move_distance(
                distance_m=args.distance,
                speed_mps=args.speed,
                save_plot=not args.no_plot,
                plot_path=args.plot
            )
        else:
            # Interactive shell
            run_interactive(nav, sensors, comm, default_plot_path=args.plot)

    except KeyboardInterrupt:
        print("\n[Nav] Mission interrupted by user.")
    finally:
        nav.request_stop()
        comm.close()
        sensors.close()


if __name__ == "__main__":
    main()
