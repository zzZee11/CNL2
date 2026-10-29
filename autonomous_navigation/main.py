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
    from .config import SERIAL_PORT, SERIAL_BAUD, DEFAULT_SPEED, DEFAULT_PLOT_PATH
    from .sensors import DistanceSensorManager
    from .serial_comm import RobotSerialComm
    from .navigator import AutonomousNavigator
    from .plotter import save_navigation_plot
except ImportError:
    from config import SERIAL_PORT, SERIAL_BAUD, DEFAULT_SPEED, DEFAULT_PLOT_PATH
    from sensors import DistanceSensorManager
    from serial_comm import RobotSerialComm
    from navigator import AutonomousNavigator
    from plotter import save_navigation_plot


def parse_args():
    parser = argparse.ArgumentParser(
        description="Autonomous 2D Cartesian & Distance Navigator with Obstacle Avoidance"
    )
    parser.add_argument(
        "--goal", "-g", nargs=2, type=float, metavar=("X", "Y"), default=None,
        help="Target Cartesian coordinate in meters (e.g. --goal 1.0 5.0)"
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
        "--mock", action="store_true",
        help="Run in mock/simulation mode without physical hardware"
    )
    return parser.parse_args()


def parse_goto_input(text: str):
    """
    Parses various user coordinate inputs like:
      'goto 1 5'
      'goto 1.0 5.0 0.8'
      'goto (1, 5)'
      'goto (1.0, 5.0) 0.6'
      '1 5'
    Returns: (target_x, target_y, speed_or_none) or None
    """
    cleaned = text.strip()
    if cleaned.lower().startswith("goto"):
        cleaned = cleaned[4:].strip()

    # Extract all floating point numbers (including signs)
    nums = re.findall(r"[-+]?\d*\.?\d+", cleaned)
    if len(nums) >= 2:
        try:
            gx = float(nums[0])
            gy = float(nums[1])
            spd = float(nums[2]) if len(nums) >= 3 else None
            return gx, gy, spd
        except ValueError:
            return None
    return None


def run_interactive(nav, sensors, comm, default_plot_path=DEFAULT_PLOT_PATH):
    print("\n" + "=" * 70)
    print("AUTONOMOUS CARTESIAN NAVIGATOR (A* Global + APF Vector Avoidance)")
    print("=" * 70)
    print("Navigation Commands:")
    print("  goto <X> <Y> [speed]    Navigate to Cartesian coordinate (e.g. 'goto 1 5' or 'goto 1.0 5.0 0.7')")
    print("  move <dist> m [speed]   Legacy forward drive (e.g. 'move 5 m 0.8' or 'move 2 m')")
    print("  plot [filepath]         Save visualization map of the latest trajectory and obstacles")
    print("  sensors                 Display real-time 3x ToF distance sensor readings")
    print("  pose                    Display current dead-reckoning position (X, Y, Heading)")
    print("  stop                    Emergency halt")
    print("  quit  (or 'q')          Exit")
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

        if cmd_lower in ("sensors", "tof"):
            d = sensors.get_distances()
            print(f"[Sensors] Left: {d[0]:.2f}m | Middle: {d[1]:.2f}m | Right: {d[2]:.2f}m")
            continue

        if cmd_lower in ("pose", "odom"):
            x, y, theta, dist = comm.get_pose()
            print(f"[Pose] X: {x:.2f}m | Y: {y:.2f}m | Heading: {math.degrees(theta):.1f}° | Travelled: {dist:.2f}m")
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

        # Goto Cartesian coordinate (e.g. 'goto 1 5', 'goto 1.0 5.0 0.8', '(1, 5)')
        goto_match = parse_goto_input(line)
        if (cmd_lower.startswith("goto") or cmd_lower.startswith("navigate")) and goto_match:
            gx, gy, spd = goto_match
            chosen_speed = spd if spd is not None else DEFAULT_SPEED
            nav.navigate_to(gx, gy, speed_mps=chosen_speed, save_plot=True, plot_path=default_plot_path)
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

    comm = RobotSerialComm(port=args.port, baud=args.baud, use_mock=args.mock)
    if not comm.connect():
        sensors.close()
        sys.exit(1)
    comm.start()

    nav = AutonomousNavigator(serial_comm=comm, sensor_mgr=sensors)

    try:
        if args.goal is not None:
            # 2D Cartesian target navigation: --goal X Y
            gx, gy = args.goal
            nav.navigate_to(
                target_x=gx,
                target_y=gy,
                speed_mps=args.speed,
                save_plot=not args.no_plot,
                plot_path=args.plot
            )
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
