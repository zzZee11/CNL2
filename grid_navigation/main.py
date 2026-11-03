#!/usr/bin/env python3
"""
grid_navigation/main.py
-----------------------
Primary entry point for discrete 2D grid-based autonomous navigation.

Usage Examples:
  # Navigate to Cartesian coordinate (1.0, 1.0) with final heading 90 deg:
  python3 grid_navigation/main.py --goal 1.0 1.0 90

  # Navigate to (1.5, 0.5) and immediately retrace path back to (0, 0):
  python3 grid_navigation/main.py --goal 1.5 0.5 --return

  # Shuttle back and forth 3 times:
  python3 grid_navigation/main.py --goal 1.0 1.0 --patrol 3

  # Simulation / Mock test (runs on PC without hardware):
  python3 grid_navigation/main.py --mock --goal 1.0 1.0 90 --return

  # Interactive CLI shell:
  python3 grid_navigation/main.py
"""

import os
import sys
import math
import argparse

# Ensure local and parent package paths are available
pkg_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(pkg_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
if pkg_dir not in sys.path:
    sys.path.insert(0, pkg_dir)

try:
    from .config import SERIAL_PORT, SERIAL_BAUD, CRUISE_SPEED, DEFAULT_PLOT_PATH
    from .grid_map import OccupancyGridMap
    from .robot_hardware import RobotHardware
    from .navigator import GridNavigator
    from .visualizer import export_navigation_map
except ImportError:
    from config import SERIAL_PORT, SERIAL_BAUD, CRUISE_SPEED, DEFAULT_PLOT_PATH
    from grid_map import OccupancyGridMap
    from robot_hardware import RobotHardware
    from navigator import GridNavigator
    from visualizer import export_navigation_map


def parse_args():
    parser = argparse.ArgumentParser(
        description="Clean 2D Grid-Based Autonomous Navigation for CNL2 Robot"
    )
    parser.add_argument(
        "--goal", "-g", nargs="+", type=float, default=None,
        help="Target Cartesian coordinate in meters: X Y [THETA] (e.g. --goal 1.0 1.0 or --goal 1.0 1.0 90)"
    )
    parser.add_argument(
        "--heading", type=float, default=None,
        help="Target orientation at destination in degrees (e.g. --heading 90)"
    )
    parser.add_argument(
        "--return", "--roundtrip", dest="return_to_start", action="store_true",
        help="Retrace path forward back to start point (0, 0) after reaching goal"
    )
    parser.add_argument(
        "--patrol", type=int, default=None, metavar="N",
        help="Patrol/shuttle back and forth between start and goal N times"
    )
    parser.add_argument(
        "--speed", "-s", type=float, default=CRUISE_SPEED,
        help=f"Cruising speed in m/s (default: {CRUISE_SPEED})"
    )
    parser.add_argument(
        "--mock", action="store_true",
        help="Run in mock/simulation mode without hardware"
    )
    parser.add_argument(
        "--plot", type=str, default=DEFAULT_PLOT_PATH,
        help=f"Destination image file for map plot (default: {DEFAULT_PLOT_PATH})"
    )
    parser.add_argument(
        "--no-plot", action="store_true",
        help="Disable automatic plot generation"
    )
    parser.add_argument(
        "--port", default=SERIAL_PORT,
        help=f"Serial port for ESP32 (default: {SERIAL_PORT})"
    )
    parser.add_argument(
        "--baud", type=int, default=SERIAL_BAUD,
        help=f"Serial baud rate (default: {SERIAL_BAUD})"
    )
    return parser.parse_args()


def run_interactive_shell(nav: GridNavigator, default_plot: str):
    """Simple interactive CLI prompt for testing navigation commands."""
    print("\n========================================================")
    print("      CNL2 GRID NAVIGATION — INTERACTIVE SHELL          ")
    print("========================================================")
    print("Commands:")
    print("  goto X Y [THETA] [--return] : Navigate to target coordinate")
    print("  return / back              : Retrace exact path forward to (0,0)")
    print("  patrol X Y N               : Patrol N cycles back and forth")
    print("  sensors                    : Read live ToF distances")
    print("  pose                       : Display live robot pose (X, Y, Theta)")
    print("  clear                      : Reset occupancy grid map")
    print("  plot [filename]            : Export current navigation map PNG")
    print("  stop                       : Emergency halt")
    print("  quit / exit                : Exit shell")
    print("========================================================\n")

    while True:
        try:
            line = input("grid_nav> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting shell.")
            break

        if not line:
            continue

        parts = line.split()
        cmd = parts[0].lower()

        if cmd in ("quit", "exit", "q"):
            break

        if cmd in ("stop", "halt", "s"):
            nav.request_stop()
            print("[GridNav] Emergency halt signaled.")
            continue

        if cmd in ("clear", "reset"):
            nav.clear_map()
            continue

        if cmd in ("sensors", "tof"):
            d = nav.hw.get_tof_distances()
            print(f"[Sensors] Left: {d[0]:.2f}m | Middle: {d[1]:.2f}m | Right: {d[2]:.2f}m")
            continue

        if cmd in ("pose", "odom"):
            x, y, th, dist = nav.hw.get_pose()
            print(f"[Pose] X: {x:.3f}m | Y: {y:.3f}m | Heading: {math.degrees(th):+5.1f}° | Travelled: {dist:.2f}m")
            continue

        if cmd in ("return", "back", "home"):
            nav.return_to_start(save_plot=True, plot_path=default_plot)
            continue

        if cmd == "plot":
            out_file = parts[1] if len(parts) > 1 else default_plot
            fx, fy, fth, fdist = nav.hw.get_pose()
            export_navigation_map(
                trajectory=nav.last_trajectory,
                planned_path=nav.last_planned_path,
                obstacles=nav.grid.get_raw_obstacles_world(),
                final_pose=(fx, fy, fth),
                total_dist=fdist,
                status="MANUAL_SAVE",
                output_path=out_file
            )
            continue

        if cmd == "patrol" and len(parts) >= 4:
            try:
                gx, gy, cycles = float(parts[1]), float(parts[2]), int(parts[3])
                nav.patrol(target_x=gx, target_y=gy, cycles=cycles, save_plot=True, plot_path=default_plot)
            except ValueError:
                print("Usage: patrol <X> <Y> <cycles> (e.g. patrol 1.0 1.0 3)")
            continue

        if cmd in ("goto", "navigate") and len(parts) >= 3:
            try:
                gx = float(parts[1])
                gy = float(parts[2])
                target_th = float(parts[3]) if len(parts) >= 4 and not parts[3].startswith("-") else None
                ret = "--return" in parts or "-r" in parts
                nav.navigate_to(
                    target_x=gx,
                    target_y=gy,
                    target_theta_deg=target_th,
                    return_to_start=ret,
                    save_plot=True,
                    plot_path=default_plot
                )
            except ValueError:
                print("Usage: goto <X> <Y> [heading_deg] [--return]")
            continue

        print(f"Unknown command '{cmd}'. Try: goto 1 1, return, patrol 1 1 2, sensors, pose, plot, or quit.")


def main():
    args = parse_args()

    hw = RobotHardware(port=args.port, baud=args.baud, use_mock=args.mock)
    if not hw.initialize():
        sys.exit(1)
    hw.start()

    grid = OccupancyGridMap()
    nav = GridNavigator(hardware=hw, grid_map=grid)

    try:
        if args.goal is not None:
            if len(args.goal) < 2:
                print("[Main] Error: --goal requires at least 2 arguments: X Y (e.g. --goal 1.0 1.0)")
                return
            gx = args.goal[0]
            gy = args.goal[1]
            target_th = args.goal[2] if len(args.goal) >= 3 else args.heading

            if args.patrol is not None:
                nav.patrol(
                    target_x=gx,
                    target_y=gy,
                    cycles=args.patrol,
                    target_theta_deg=target_th,
                    speed_mps=args.speed,
                    save_plot=not args.no_plot,
                    plot_path=args.plot
                )
            else:
                nav.navigate_to(
                    target_x=gx,
                    target_y=gy,
                    target_theta_deg=target_th,
                    speed_mps=args.speed,
                    return_to_start=args.return_to_start,
                    save_plot=not args.no_plot,
                    plot_path=args.plot
                )
        else:
            # Interactive Shell Mode
            run_interactive_shell(nav, default_plot=args.plot)

    finally:
        nav.request_stop()
        hw.close()


if __name__ == "__main__":
    main()

