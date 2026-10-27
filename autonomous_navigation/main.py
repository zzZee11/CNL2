#!/usr/bin/env python3
"""
autonomous_navigation/main.py
------------------------------
Primary entry point for autonomous distance navigation with dynamic obstacle avoidance.

Usage Examples:
  # Move 5 meters at 0.8 m/s:
  python3 autonomous_navigation/main.py --distance 5.0 --speed 0.8

  # Interactive CLI shell:
  python3 autonomous_navigation/main.py

  # Test in simulation mode (on PC without hardware):
  python3 autonomous_navigation/main.py --mock --distance 5.0 --speed 0.8
"""

import os
import sys
import time
import math
import argparse

# Ensure both parent directory and local directory are in sys.path for direct execution
pkg_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(pkg_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
if pkg_dir not in sys.path:
    sys.path.insert(0, pkg_dir)

try:
    from .config import SERIAL_PORT, SERIAL_BAUD, DEFAULT_SPEED
    from .sensors import DistanceSensorManager
    from .serial_comm import RobotSerialComm
    from .navigator import AutonomousNavigator
except ImportError:
    from config import SERIAL_PORT, SERIAL_BAUD, DEFAULT_SPEED
    from sensors import DistanceSensorManager
    from serial_comm import RobotSerialComm
    from navigator import AutonomousNavigator

def parse_args():
    parser = argparse.ArgumentParser(description="Autonomous Distance & Obstacle Avoidance Navigator")
    parser.add_argument("--distance", "-d", type=float, default=None,
                        help="Distance to travel in meters (e.g. 5.0)")
    parser.add_argument("--speed", "-s", type=float, default=DEFAULT_SPEED,
                        help=f"Target cruise speed in m/s (default: {DEFAULT_SPEED})")
    parser.add_argument("--port", "-p", default=SERIAL_PORT,
                        help=f"Serial port for ESP32 (default: {SERIAL_PORT})")
    parser.add_argument("--baud", "-b", type=int, default=SERIAL_BAUD,
                        help=f"Serial baud rate (default: {SERIAL_BAUD})")
    parser.add_argument("--mock", action="store_true",
                        help="Run in mock/simulation mode without physical hardware")
    return parser.parse_args()


def run_interactive(nav, sensors, comm):
    print("\n" + "=" * 70)
    print("AUTONOMOUS DISTANCE NAVIGATOR (A* + APF + 3x ToF)")
    print("=" * 70)
    print("Commands:")
    print("  move <dist> m [speed]   e.g. 'move 5 m 0.8' or 'move 2 m'")
    print("  stop                    Emergency halt")
    print("  sensors                 Display live ToF distance sensor readings")
    print("  pose                    Display current dead-reckoning position")
    print("  quit  (or 'q')          Exit")
    print("=" * 70 + "\n")

    while True:
        try:
            line = input("nav> ").strip().lower()
        except (KeyboardInterrupt, EOFError):
            break

        if not line:
            continue

        if line in ("quit", "q", "exit"):
            break

        if line in ("stop", "s", "brake"):
            nav.request_stop()
            print("[Nav] Robot stopped.")
            continue

        if line in ("sensors", "tof"):
            d = sensors.get_distances()
            print(f"[Sensors] Left: {d[0]:.2f}m | Middle: {d[1]:.2f}m | Right: {d[2]:.2f}m")
            continue

        if line in ("pose", "odom"):
            x, y, theta, dist = comm.get_pose()
            print(f"[Pose] X: {x:.2f}m | Y: {y:.2f}m | Heading: {math.degrees(theta):.1f}° | Travelled: {dist:.2f}m")
            continue

        parts = line.split()
        if len(parts) >= 2 and parts[0] == "move":
            try:
                distance = float(parts[1])
                speed = DEFAULT_SPEED

                # Check if format has 'm' (e.g. 'move 5 m' or 'move 5 m 0.8')
                if len(parts) >= 3 and parts[2] == "m":
                    if len(parts) >= 4:
                        speed = float(parts[3])
                elif len(parts) >= 3:
                    speed = float(parts[2])

                nav.move_distance(distance, speed_mps=speed)

            except ValueError:
                print("Invalid input format. Use: move 5 m 0.8")
        else:
            print("Unknown command. Type 'move 5 m 0.8', 'sensors', 'pose', 'stop', or 'quit'.")


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
        if args.distance is not None:
            # Single-shot command line run
            nav.move_distance(args.distance, speed_mps=args.speed)
        else:
            # Interactive shell
            run_interactive(nav, sensors, comm)

    except KeyboardInterrupt:
        print("\n[Nav] Interrupted by user.")

    finally:
        nav.request_stop()
        comm.close()
        sensors.close()
        print("[Nav] All systems cleanly terminated.")


if __name__ == "__main__":
    main()
