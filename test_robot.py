import math
import time
import threading
import serial

# Optional MPU-6050 IMU for spatial heading lock
try:
    from autonomous_navigation.imu import IMUManager
except ImportError:
    IMUManager = None

# UART configuration: change if your wiring uses another UART.
PORT = "/dev/serial0"
BAUD = 115200

# Must match the ESP32 Config.h and your actual hardware.
WHEEL_RADIUS_M = 0.0215
WHEEL_BASE_M = 0.134
TICKS_PER_REV = 720.0  # Synced with RobotController/Config.h (720.0 ticks/rev)

# Startup push & cruise configuration:
PUSH_SPEED_MPS = 0.50     # Breakaway speed demand (m/s) to overcome gearbox static friction
PUSH_DURATION_S = 0.15    # 150 ms high-torque kick
DEFAULT_CRUISE_MPS = 0.40 # Default intended cruising speed (m/s)

# Target distance tolerance.
DISTANCE_TOLERANCE_M = 0.02

# Active Heading Lock (Closed-Loop Course Correction)
ENABLE_HEADING_LOCK = True
KP_HEADING = 0.50          # Gentle differential m/s per radian of heading error
KD_HEADING = 0.08          # Damping gain to eliminate steering overshoot
MAX_STEER_CORRECTION_MPS = 0.12 # Soft maximum differential wheel speed offset (m/s)
HEADING_DEADBAND_RAD = math.radians(1.5) # 1.5° deadband: drive straight when aligned without jitter


# Initialize Serial UART
try:
    robot = serial.Serial(PORT, BAUD, timeout=0.02)
    time.sleep(1.5)
except (serial.SerialException, FileNotFoundError, OSError) as e:
    print(f"[Warning] Could not open {PORT} ({e}). Telemetry may be simulated/unavailable.")
    robot = None

# Initialize MPU-6050 IMU if present
imu = None
if IMUManager:
    try:
        _candidate_imu = IMUManager(bus_num=1, address=0x68)
        if _candidate_imu.initialize():
            _candidate_imu.calibrate(num_samples=60, quiet=False)
            _candidate_imu.start()
            imu = _candidate_imu
            print("[System] MPU-6050 active! Using spatial gyroscope for heading lock.")
        else:
            print("[System] MPU-6050 not detected. Using wheel-encoder heading lock.")
    except Exception as e:
        print(f"[System] IMU init failed ({e}). Using wheel-encoder heading lock.")
        imu = None

left_ticks = None
right_ticks = None
lock = threading.Lock()
running = True


def read_uart():
    global left_ticks, right_ticks, running

    while running:
        if not robot or not robot.is_open:
            time.sleep(0.1)
            continue

        try:
            line = robot.readline().decode(
                "utf-8", errors="ignore"
            ).strip()

            parts = line.split(",")

            if len(parts) >= 3 and parts[0] == "ENC":
                with lock:
                    left_ticks = int(parts[1])
                    right_ticks = int(parts[2])

        except (ValueError, serial.SerialException, OSError):
            pass


reader = threading.Thread(target=read_uart, daemon=True)
reader.start()


def send_velocity(left, right):
    # ESP32 expects wheel velocities in m/s.
    if robot and robot.is_open:
        try:
            robot.write(f"{left:.3f},{right:.3f}\n".encode())
            robot.flush()
        except (serial.SerialException, OSError):
            pass


def stop():
    send_velocity(0.0, 0.0)


def get_ticks():
    with lock:
        return left_ticks, right_ticks


def wait_for_encoders(timeout=5):
    start = time.monotonic()

    while time.monotonic() - start < timeout:
        left, right = get_ticks()

        if left is not None and right is not None:
            return left, right

        time.sleep(0.05)

    raise RuntimeError(
        "No encoder telemetry. Check UART wiring and ESP32 firmware."
    )


def move(distance_m, cruise_speed=DEFAULT_CRUISE_MPS):
    """
    Drives the robot a linear distance forward or backward with active heading-lock.
    Positive distance = forward.
    Negative distance = reverse.
    """
    if abs(distance_m) < 0.001:
        stop()
        return

    start_left, start_right = wait_for_encoders()

    ticks_per_metre = TICKS_PER_REV / (
        2 * math.pi * WHEEL_RADIUS_M
    )
    direction = 1 if distance_m > 0 else -1
    is_reverse = (direction < 0)

    # In reverse, cap cruise speed to 0.28 m/s for calm, steady tracking
    effective_cruise = min(0.28, cruise_speed) if is_reverse else cruise_speed

    # Limit travel time in case an encoder fails.
    timeout = max(5.0, abs(distance_m) / max(0.05, effective_cruise) * 3)
    start_time = time.monotonic()
    last_time = start_time
    last_heading_err = 0.0
    last_report = 0
    v_steer_smooth = 0.0

    start_heading_rad = imu.get_heading_rad() if (imu and imu.is_ready()) else 0.0
    sensor_tag = "IMU" if (imu and imu.is_ready()) else "ENC"

    mode_str = f"Active Heading Lock [{sensor_tag}]" if ENABLE_HEADING_LOCK else "Open-Loop"
    print(f"\nMoving {distance_m:+.2f} m [{mode_str}] (Cruise at {effective_cruise:.2f} m/s)...")

    try:
        while True:
            left, right = get_ticks()

            if left is None or right is None:
                raise RuntimeError("Encoder telemetry lost")

            now = time.monotonic()

            # Signed displacement of each wheel along the commanded direction of travel
            d_left = direction * (left - start_left) / ticks_per_metre
            d_right = direction * (right - start_right) / ticks_per_metre

            # Average travelled distance along direction
            travelled_m = (d_left + d_right) / 2.0
            remaining_m = abs(distance_m) - travelled_m

            if travelled_m >= abs(distance_m) - DISTANCE_TOLERANCE_M:
                break

            if now - start_time > timeout:
                raise TimeoutError(
                    "Movement timed out. Check the wheels and encoders."
                )

            # --- 1. DYNAMIC SPEED PROFILE ---
            elapsed = now - start_time
            if is_reverse:
                # Smooth linear acceleration ramp (prevents abrupt shock kick in reverse)
                ramp_speed = 0.18 + (0.45 * elapsed)
                current_speed = min(effective_cruise, ramp_speed)
                if remaining_m < 0.08:
                    # Smooth deceleration for the final 8 cm
                    current_speed = max(0.12, current_speed * (remaining_m / 0.08))
            else:
                if elapsed < PUSH_DURATION_S and remaining_m > 0.04:
                    # Breakaway push to overcome gearbox static friction forward
                    current_speed = PUSH_SPEED_MPS
                elif remaining_m < 0.05:
                    # Smooth deceleration for the final 5 cm to prevent overshoot
                    current_speed = max(0.08, cruise_speed * (remaining_m / 0.05))
                else:
                    # Steady cruising speed
                    current_speed = cruise_speed

            # --- 2. ACTIVE HEADING LOCK (Differential Steering Correction) ---
            if imu and imu.is_ready():
                # Gyroscope spatial heading error (immune to wheel slip)
                raw_err = imu.get_heading_rad() - start_heading_rad
                heading_err = math.atan2(math.sin(raw_err), math.cos(raw_err))
                yaw_rate = imu.get_yaw_rate_rad_s()
                # Rate damping MUST oppose angular velocity (- yaw_rate)
                d_err_term = -yaw_rate
            else:
                # Wheel-encoder disparity fallback (direction-aware)
                if is_reverse:
                    heading_err = (d_left - d_right) / WHEEL_BASE_M
                else:
                    heading_err = (d_right - d_left) / WHEEL_BASE_M
                dt = max(0.001, now - last_time)
                d_err = (heading_err - last_heading_err) / dt
                last_heading_err = heading_err
                d_err_term = d_err

            last_time = now

            if ENABLE_HEADING_LOCK:
                if is_reverse:
                    # Reverse tuning: gentle steering gain, wide deadband, low max offset
                    # Keeps BOTH wheels spinning backwards without alternating/stalling
                    kp = 0.25
                    kd = 0.10
                    deadband = math.radians(2.2)  # 2.2° deadband
                    max_steer = 0.04              # Max 0.04 m/s offset (neither wheel stalls)
                    alpha = 0.15                  # Heavy EMA smoothing (85% history / 15% new)
                else:
                    # Forward tuning
                    kp = KP_HEADING
                    kd = KD_HEADING
                    deadband = HEADING_DEADBAND_RAD
                    max_steer = MAX_STEER_CORRECTION_MPS
                    alpha = 0.30

                # Deadband filter: do not twitch wheels for tiny deviations
                if abs(heading_err) < deadband:
                    v_steer_target = 0.0
                else:
                    # Subtract deadband to start correction smoothly from zero
                    effective_err = heading_err - math.copysign(deadband, heading_err)
                    v_steer_target = (kp * effective_err) + (kd * d_err_term)
                    v_steer_target = max(-max_steer, min(max_steer, v_steer_target))

                # Exponential moving average filter for silky smooth, jitter-free correction
                v_steer_smooth = ((1.0 - alpha) * v_steer_smooth) + (alpha * v_steer_target)
            else:
                v_steer_smooth = 0.0

            # Differential wheel command:
            # Base velocity is signed by direction (+ forward, - reverse).
            # To correct heading error (veer left > 0), left wheel must ALWAYS be faster than right (cmd_l > cmd_r),
            # whether driving forward or in reverse. Adding v_steer directly to base_speed prevents sign inversion in reverse.
            base_speed = direction * current_speed
            cmd_l = base_speed + v_steer_smooth
            cmd_r = base_speed - v_steer_smooth

            # Send 20 Hz velocity packet to ESP32
            send_velocity(cmd_l, cmd_r)

            if now - last_report > 0.25:
                heading_deg = math.degrees(heading_err)
                print(
                    f"  Travelled: {travelled_m:4.2f}m / {abs(distance_m):4.2f}m | "
                    f"Heading Err: {heading_deg:+5.1f}° [{sensor_tag}] | "
                    f"Wheels: L={cmd_l:+.2f}, R={cmd_r:+.2f} m/s"
                )
                last_report = now

            time.sleep(0.04)

    finally:
        stop()

    heading_final_deg = math.degrees(heading_err)
    print(f"Movement complete: {travelled_m:.2f} m | Final Heading Drift: {heading_final_deg:+.1f}° [{sensor_tag}]\n")


def turn(target_deg, max_speed=0.18):
    """
    Pivots the robot in-place by a specific angular displacement (degrees)
    using the MPU-6050 gyroscope (or encoder ticks as fallback).
    Positive = Counter-Clockwise (Turn Left).
    Negative = Clockwise (Turn Right).
    """
    if abs(target_deg) < 1.0:
        stop()
        return

    target_rad = math.radians(target_deg)
    use_imu = (imu is not None and imu.is_ready())

    if use_imu:
        prev_heading = imu.get_heading_rad()
        sensor_tag = "IMU"
    else:
        start_left, start_right = wait_for_encoders()
        ticks_per_rad = (WHEEL_BASE_M / 2.0) * (TICKS_PER_REV / (2 * math.pi * WHEEL_RADIUS_M))
        sensor_tag = "ENC"

    timeout = max(6.0, abs(target_deg) / 14.0)
    start_time = time.monotonic()
    last_report = 0
    current_turn = 0.0

    print(f"\nTurning {target_deg:+.1f}° [{sensor_tag}] (Max speed {max_speed:.2f} m/s)...")

    try:
        while True:
            now = time.monotonic()
            if now - start_time > timeout:
                raise TimeoutError("Turn timed out.")

            if use_imu:
                # Cumulative continuous rotation: unwrapped delta integration handles 360° and multi-turns
                cur_h = imu.get_heading_rad()
                delta_h = math.atan2(math.sin(cur_h - prev_heading), math.cos(cur_h - prev_heading))
                current_turn += delta_h
                prev_heading = cur_h

                angle_error = target_rad - current_turn
                yaw_rate = imu.get_yaw_rate_rad_s()
            else:
                left, right = get_ticks()
                if left is None or right is None:
                    raise RuntimeError("Encoder telemetry lost")
                current_turn = ((right - start_right) - (left - start_left)) / (2.0 * ticks_per_rad)
                angle_error = target_rad - current_turn
                yaw_rate = 0.0

            # Stop when aligned within 2.0 degrees
            if abs(math.degrees(angle_error)) < 2.0:
                break

            # Very gentle closed-loop PD turn control (slow, steady rotation)
            kp_turn = 0.35
            kd_turn = 0.06
            omega = (kp_turn * angle_error) - (kd_turn * yaw_rate)
            omega_clamped = max(-max_speed, min(max_speed, omega))

            # Minimum pivot speed to overcome gearbox static friction without spinning aggressively
            min_pivot_speed = 0.09
            if abs(omega_clamped) < min_pivot_speed:
                omega_clamped = min_pivot_speed if omega_clamped > 0 else -min_pivot_speed

            # Differential pivot: counter-rotating wheels
            cmd_l = -omega_clamped
            cmd_r = +omega_clamped
            send_velocity(cmd_l, cmd_r)

            if now - last_report > 0.20:
                print(
                    f"  Turned: {math.degrees(current_turn):+5.1f}° / {target_deg:+5.1f}° | "
                    f"Err: {math.degrees(angle_error):+5.1f}° [{sensor_tag}]"
                )
                last_report = now

            time.sleep(0.04)

    finally:
        stop()

    final_deg = math.degrees(current_turn)
    print(f"Turn complete: {final_deg:+.1f}° (Target: {target_deg:+.1f}°) [{sensor_tag}]\n")


def print_imu_status():
    """Prints live readings from the MPU-6050 sensor."""
    if not imu or not imu.is_ready():
        print("IMU is not available or running in mock mode.")
        return

    hdg = imu.get_heading_deg()
    rate = imu.get_yaw_rate_deg_s()
    ax, ay, az = imu.get_linear_accel()
    pitch = imu.tilt_pitch_deg
    roll = imu.tilt_roll_deg
    print(f"  [IMU Status]")
    print(f"    - Heading (Yaw)   : {hdg:+6.1f}°")
    print(f"    - Turn Rate       : {rate:+6.1f}°/s")
    print(f"    - Linear Accel    : X={ax:+4.2f}, Y={ay:+4.2f}, Z={az:+4.2f} m/s²")
    print(f"    - Mounting Tilt   : Pitch={pitch:+4.1f}°, Roll={roll:+4.1f}°")


def main():
    print("Robot control ready.")
    print("Commands:")
    print("  move <dist> m           e.g. 'move 1 m' or 'move -0.5 m'")
    print("  move <dist> m <speed>   e.g. 'move 1 m 0.3' (custom cruise speed)")
    print("  turn <deg> deg          e.g. 'turn 90 deg' or 'turn -45 deg'")
    print("  imu                     Show live MPU-6050 orientation & tilt")
    print("  cal                     Recalibrate IMU zero-rate bias")
    print("  stop                    Stop immediately")
    print("  quit                    Exit")

    try:
        while True:
            command = input("robot> ").strip().lower()

            if command in ("quit", "exit", "q"):
                break

            if command in ("stop", "s"):
                stop()
                continue

            if command == "imu":
                print_imu_status()
                continue

            if command == "cal":
                if imu:
                    imu.calibrate(num_samples=80, quiet=False)
                else:
                    print("IMU not connected.")
                continue

            parts = command.split()

            if len(parts) >= 3 and parts[0] == "move":
                try:
                    distance = float(parts[1])

                    if parts[2] != "m":
                        print("Use metres, e.g. move 1 m")
                        continue

                    cruise = DEFAULT_CRUISE_MPS
                    if len(parts) >= 4:
                        cruise = float(parts[3])

                    move(distance, cruise_speed=cruise)

                except (ValueError, RuntimeError, TimeoutError) as exc:
                    stop()
                    print(f"Error: {exc}")

            elif len(parts) >= 3 and parts[0] == "turn":
                try:
                    degrees = float(parts[1])
                    if parts[2] not in ("deg", "degrees", "d"):
                        print("Use degrees, e.g. turn 90 deg")
                        continue

                    turn_speed = 0.35
                    if len(parts) >= 4:
                        turn_speed = float(parts[3])

                    turn(degrees, max_speed=turn_speed)

                except (ValueError, RuntimeError, TimeoutError) as exc:
                    stop()
                    print(f"Error: {exc}")

            else:
                print("Use: move 1 m, turn 90 deg, imu, cal, stop, or quit")

    except KeyboardInterrupt:
        print("\nStopping robot.")

    finally:
        stop()
        time.sleep(0.1)
        running = False
        if imu:
            imu.stop()
        if robot and robot.is_open:
            robot.close()


if __name__ == "__main__":
    main()

