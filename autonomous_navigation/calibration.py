"""
autonomous_navigation/calibration.py
-----------------------------------
Physical Odometry Calibration Utility (Solution 5).

Calibrates the robot's physical wheel radius (WHEEL_RADIUS_M) and track width (WHEEL_BASE_M)
to eliminate mechanical scaling errors and wheel-slip bias on specific floor surfaces.
Results are saved to autonomous_navigation/calibration.json and automatically loaded on startup.

Usage:
  # From command line:
  python3 autonomous_navigation/main.py --calibrate
  
  # Or inside interactive CLI shell:
  calibrate
"""

import os
import sys
import time
import math
from typing import Tuple, Optional

try:
    from . import config
except ImportError:
    import config


def calibrate_linear_distance(
    comm,
    nav=None,
    target_dist_m: float = 1.00,
    speed_mps: float = 0.25,
    user_measured_cm: Optional[float] = None
) -> Tuple[float, float]:
    """
    Drives the robot forward target_dist_m meters at slow speed, then prompts for the
    physical ruler measurement to calculate the exact effective WHEEL_RADIUS_M.

    Returns:
        (new_wheel_radius_m, correction_ratio)
    """
    print("\n========================================================")
    print(" STEP 1: LINEAR DISTANCE CALIBRATION (WHEEL RADIUS)")
    print("========================================================")
    print(f"Current Wheel Radius: {config.WHEEL_RADIUS_M * 100.0:.2f} cm")
    print(f"Commanded Distance:   {target_dist_m:.2f} m ({target_dist_m * 100.0:.0f} cm)")
    print(f"Cruise Speed:         {speed_mps:.2f} m/s")
    print("--------------------------------------------------------")
    print("Instructions:")
    print("1. Align the front edge or center wheel mark of the robot with a tape measure mark (0.00 cm).")
    print("2. Ensure a clear straight 1.5-meter path ahead.")

    if user_measured_cm is None:
        try:
            input("Press [Enter] to start forward driving...")
        except (EOFError, KeyboardInterrupt):
            print("\nCalibration canceled.")
            return config.WHEEL_RADIUS_M, 1.0

    # Reset odometry to 0,0,0
    comm.reset_odometry(0.0, 0.0, 0.0)
    time.sleep(0.1)

    print(f"\n[Calibrate] Driving forward {target_dist_m:.2f} m...")
    t0 = time.time()
    timeout = (target_dist_m / speed_mps) * 2.5 + 5.0

    while time.time() - t0 < timeout:
        cur_x, cur_y, cur_th, dist = comm.get_pose()
        rem = target_dist_m - dist
        if rem <= 0.01:
            comm.active_brake()
            break

        # Terminal deceleration ramp down to 0.04 m/s in the final 10 cm
        if rem > 0.15:
            spd = speed_mps
        else:
            spd = max(0.04, speed_mps * (rem / 0.15))

        # Mild straight-line heading hold
        omega = -1.2 * cur_th
        vl = spd - (omega * config.WHEEL_BASE_M / 2.0)
        vr = spd + (omega * config.WHEEL_BASE_M / 2.0)
        comm.set_speeds(vl, vr)
        time.sleep(0.04)

    comm.active_brake()
    time.sleep(0.2)

    cur_x, cur_y, cur_th, odom_dist = comm.get_pose()
    print(f"[Calibrate] Stopped. Odometry distance: {odom_dist*100.0:.1f} cm.")

    if user_measured_cm is None:
        while True:
            try:
                raw = input(f"Enter actual tape-measured distance in cm [default {target_dist_m * 100.0:.0f}]: ").strip()
                if not raw:
                    user_measured_cm = target_dist_m * 100.0
                    break
                user_measured_cm = float(raw)
                if user_measured_cm > 10.0:
                    break
                print("Please enter a realistic distance (> 10 cm).")
            except (ValueError, EOFError):
                user_measured_cm = target_dist_m * 100.0
                break

    actual_m = float(user_measured_cm) / 100.0
    ratio = actual_m / target_dist_m
    new_radius = round(config.WHEEL_RADIUS_M * ratio, 5)

    print(f"\n[Calibrate] Measured: {actual_m*100.0:.1f} cm | Commanded: {target_dist_m*100.0:.1f} cm")
    print(f"[Calibrate] Scaling Ratio: {ratio:.4f} ({(ratio - 1.0)*100.0:+.2f}%)")
    print(f"[Calibrate] Calibrated WHEEL_RADIUS_M: {new_radius * 100.0:.3f} cm (was {config.WHEEL_RADIUS_M * 100.0:.3f} cm)")
    return new_radius, ratio


def calibrate_wheel_base(
    comm,
    nav=None,
    num_turns: int = 3,
    speed_mps: float = 0.25,
    user_offset_deg: Optional[float] = None
) -> Tuple[float, float]:
    """
    Rotates in place 360° x num_turns using differential wheel odometry, then checks
    physical orientation (via IMU or user observation) to calculate the effective WHEEL_BASE_M.

    Returns:
        (new_wheel_base_m, correction_ratio)
    """
    num_turns = max(1, int(num_turns))
    target_deg = 360.0 * num_turns
    target_rad = math.radians(target_deg)

    print("\n========================================================")
    print(" STEP 2: TRACK WIDTH / WHEEL BASE CALIBRATION")
    print("========================================================")
    print(f"Current Wheel Base: {config.WHEEL_BASE_M * 100.0:.2f} cm")
    print(f"Test Rotation:      {num_turns} full rotations ({target_deg:.0f}°)")
    print(f"Rotation Speed:     {speed_mps:.2f} m/s")
    print("--------------------------------------------------------")
    print("Instructions:")
    print("1. Place robot on the test surface with enough clearance to spin (40x40 cm).")
    print("2. Note the starting heading (e.g. point the nose at a wall reference).")

    if user_offset_deg is None:
        try:
            input("Press [Enter] to start spin rotation...")
        except (EOFError, KeyboardInterrupt):
            print("\nCalibration canceled.")
            return config.WHEEL_BASE_M, 1.0

    comm.reset_odometry(0.0, 0.0, 0.0)
    time.sleep(0.1)
    with comm.state_lock:
        start_left = comm.left_ticks
        start_right = comm.right_ticks

    # Arc distance each wheel must travel: arc = (theta / 2) * wheel_base
    wheel_arc_target = (target_rad / 2.0) * config.WHEEL_BASE_M

    print(f"\n[Calibrate] Rotating in place {num_turns} turns...")
    t0 = time.time()
    timeout = (wheel_arc_target / speed_mps) * 2.5 + 8.0

    while time.time() - t0 < timeout:
        _, _, cur_th, _ = comm.get_pose()
        # Measure wheel ticks / arc progression
        with comm.state_lock:
            lt = abs(comm.left_ticks - start_left)
            rt = abs(comm.right_ticks - start_right)
        dist_l = (lt / config.TICKS_PER_REV) * (2.0 * math.pi * config.WHEEL_RADIUS_M)
        dist_r = (rt / config.TICKS_PER_REV) * (2.0 * math.pi * config.WHEEL_RADIUS_M)
        wheel_arc_done = (dist_l + dist_r) / 2.0

        rem = wheel_arc_target - wheel_arc_done
        if rem <= 0.002:
            comm.active_brake()
            break

        # Slow down during the last half-turn
        spd = speed_mps if rem > 0.06 else max(0.08, speed_mps * (rem / 0.06))
        # Turn CCW: Left backward, Right forward
        comm.set_speeds(-spd, +spd)
        time.sleep(0.03)

    comm.active_brake()
    time.sleep(0.3)

    # Read IMU heading if available
    imu_angle_deg = None
    if comm.imu and comm.imu.is_ready():
        imu_angle_deg = comm.imu.get_heading_deg()
        print(f"[Calibrate] IMU detected heading offset: {imu_angle_deg:+.1f}°")

    if user_offset_deg is None:
        default_offset = f"{imu_angle_deg:+.1f}" if imu_angle_deg is not None else "0"
        print("Did the robot stop facing the EXACT initial heading?")
        print("  - If it under-rotated (stopped short): enter negative degrees (e.g. -15)")
        print("  - If it over-rotated (spun too far):  enter positive degrees (e.g. +10)")
        print(f"  - If perfectly aligned:               press [Enter] (default {default_offset}°)")
        while True:
            try:
                raw = input(f"Enter offset degrees [default {default_offset}]: ").strip()
                if not raw:
                    user_offset_deg = float(default_offset)
                    break
                user_offset_deg = float(raw)
                break
            except (ValueError, EOFError):
                user_offset_deg = 0.0
                break

    actual_deg = target_deg + float(user_offset_deg)
    ratio = actual_deg / target_deg
    new_wheel_base = round(config.WHEEL_BASE_M * ratio, 5)

    print(f"\n[Calibrate] Commanded: {target_deg:.0f}° | Actual Turn: {actual_deg:.1f}°")
    print(f"[Calibrate] Scaling Ratio: {ratio:.4f} ({(ratio - 1.0)*100.0:+.2f}%)")
    print(f"[Calibrate] Calibrated WHEEL_BASE_M: {new_wheel_base * 100.0:.3f} cm (was {config.WHEEL_BASE_M * 100.0:.3f} cm)")
    return new_wheel_base, ratio


def run_interactive_calibration(comm, nav=None):
    """
    Full step-by-step interactive calibration wizard.
    Allows testing/calibrating linear distance, wheel track, or both,
    and saving the tuned parameters to calibration.json.
    """
    print("\n========================================================")
    print("   CNL2 PHYSICAL ROBOT ODOMETRY CALIBRATION WIZARD     ")
    print("========================================================")
    print(f"Current Settings:")
    print(f"  - Wheel Radius (R): {config.WHEEL_RADIUS_M * 100.0:.3f} cm")
    print(f"  - Track Width (L):  {config.WHEEL_BASE_M * 100.0:.3f} cm")
    print(f"  - Ticks / Rev:      {config.TICKS_PER_REV:.1f}")
    if os.path.exists(config.CALIBRATION_FILE):
        print(f"  - Calibration File: {config.CALIBRATION_FILE} (Active)")
    else:
        print(f"  - Calibration File: None (Using factory defaults)")
    print("--------------------------------------------------------")
    print("Choose calibration option:")
    print("  [1] Calibrate Linear Distance & Wheel Radius (R)")
    print("  [2] Calibrate Track Width & In-Place Rotation (L)")
    print("  [3] Run Full Calibration (Both R and L)")
    print("  [4] Reset Calibration to Factory Defaults")
    print("  [5] Exit without changes")

    try:
        choice = input("\nEnter choice [1-5]: ").strip()
    except (EOFError, KeyboardInterrupt):
        return

    cur_r = config.WHEEL_RADIUS_M
    cur_l = config.WHEEL_BASE_M

    if choice == "1":
        new_r, _ = calibrate_linear_distance(comm, nav)
        cur_r = new_r
    elif choice == "2":
        new_l, _ = calibrate_wheel_base(comm, nav)
        cur_l = new_l
    elif choice == "3":
        new_r, _ = calibrate_linear_distance(comm, nav)
        cur_r = new_r
        new_l, _ = calibrate_wheel_base(comm, nav)
        cur_l = new_l
    elif choice == "4":
        config.reset_calibration()
        print("\n[Calibrate] Reset to factory defaults completed.")
        return
    else:
        print("\nExiting calibration wizard.")
        return

    print("\n========================================================")
    print(" CALIBRATION SUMMARY")
    print("========================================================")
    print(f"Wheel Radius: {cur_r * 100.0:.3f} cm (Factory: {config.DEFAULT_WHEEL_RADIUS_M * 100.0:.3f} cm)")
    print(f"Track Width:  {cur_l * 100.0:.3f} cm (Factory: {config.DEFAULT_WHEEL_BASE_M * 100.0:.3f} cm)")
    print("--------------------------------------------------------")

    try:
        confirm = input("Save these calibrated values to calibration.json? [Y/n]: ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        confirm = "n"

    if confirm in ("", "y", "yes"):
        saved = config.save_calibration(wheel_radius_m=cur_r, wheel_base_m=cur_l)
        print(f"[Calibrate] Saved successfully to: {config.CALIBRATION_FILE}")
        print("[Calibrate] Physical parameters are now active for all missions.")
    else:
        print("[Calibrate] Discarded. Physical constants remain unchanged.")
