import math
import time
import threading
import serial

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
KP_HEADING = 0.85          # Differential m/s per radian of heading error
KD_HEADING = 0.12          # Damping gain to eliminate steering overshoot
MAX_STEER_CORRECTION_MPS = 0.18 # Maximum differential wheel speed offset (m/s)

try:
    robot = serial.Serial(PORT, BAUD, timeout=0.02)
    time.sleep(1.5)
except (serial.SerialException, FileNotFoundError, OSError) as e:
    print(f"[Warning] Could not open {PORT} ({e}). Telemetry may be simulated/unavailable.")
    robot = None

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

    # Limit travel time in case an encoder fails.
    timeout = max(5.0, abs(distance_m) / max(0.05, cruise_speed) * 3)
    start_time = time.monotonic()
    last_time = start_time
    last_heading_err = 0.0
    last_report = 0

    mode_str = "Active Heading Lock" if ENABLE_HEADING_LOCK else "Open-Loop"
    print(f"\nMoving {distance_m:+.2f} m [{mode_str}] (Cruise at {cruise_speed:.2f} m/s)...")

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
            if elapsed < PUSH_DURATION_S and remaining_m > 0.04:
                # Breakaway push to overcome gearbox static friction
                current_speed = PUSH_SPEED_MPS
            elif remaining_m < 0.05:
                # Smooth deceleration for the final 5 cm to prevent overshoot
                current_speed = max(0.08, cruise_speed * (remaining_m / 0.05))
            else:
                # Steady cruising speed
                current_speed = cruise_speed

            # --- 2. ACTIVE HEADING LOCK (Differential Steering Correction) ---
            # Heading error in radians: positive means robot veered left (right wheel ahead of left)
            heading_err = (d_right - d_left) / WHEEL_BASE_M

            dt = max(0.001, now - last_time)
            d_err = (heading_err - last_heading_err) / dt
            last_time = now
            last_heading_err = heading_err

            if ENABLE_HEADING_LOCK:
                # PD controller on heading error
                v_steer = (KP_HEADING * heading_err) + (KD_HEADING * d_err)
                v_steer = max(-MAX_STEER_CORRECTION_MPS, min(MAX_STEER_CORRECTION_MPS, v_steer))
            else:
                v_steer = 0.0

            # Differential wheel command:
            # If veered left (heading_err > 0), left wheel speeds up, right wheel slows down
            cmd_l = direction * (current_speed + v_steer)
            cmd_r = direction * (current_speed - v_steer)

            # Send 20 Hz velocity packet to ESP32
            send_velocity(cmd_l, cmd_r)

            if now - last_report > 0.25:
                heading_deg = math.degrees(heading_err)
                print(
                    f"  Travelled: {travelled_m:4.2f}m / {abs(distance_m):4.2f}m | "
                    f"Heading Err: {heading_deg:+5.1f}° | "
                    f"Wheels: L={cmd_l:+.2f}, R={cmd_r:+.2f} m/s"
                )
                last_report = now

            time.sleep(0.04)

    finally:
        stop()

    heading_final_deg = math.degrees(heading_err)
    print(f"Movement complete: {travelled_m:.2f} m | Final Heading Drift: {heading_final_deg:+.1f}°\n")


def main():
    print("Robot control ready.")
    print("Commands:")
    print("  move <dist> m           e.g. 'move 1 m' or 'move -0.5 m'")
    print("  move <dist> m <speed>   e.g. 'move 1 m 0.3' (custom cruise speed)")
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

            else:
                print("Use: move 1 m, move -0.5 m, stop, or quit")

    except KeyboardInterrupt:
        print("\nStopping robot.")

    finally:
        stop()
        time.sleep(0.1)
        running = False
        robot.close()


if __name__ == "__main__":
    main()

