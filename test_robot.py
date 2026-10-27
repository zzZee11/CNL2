import math
import time
import threading
import serial

# UART configuration: change if your wiring uses another UART.
PORT = "/dev/serial0"
BAUD = 115200

# Must match the ESP32 Config.h and your actual hardware.
WHEEL_RADIUS_M = 0.020
TICKS_PER_REV = 360.0  # Verify experimentally!

# Startup push & cruise configuration:
PUSH_SPEED_MPS = 0.80     # Max power speed demand (forces ESP32 PID output to 255 PWM)
PUSH_DURATION_S = 0.18    # 180 ms high-torque kick to overcome static friction (stiction)
DEFAULT_CRUISE_MPS = 0.20 # Normal intended cruising speed

# Target distance tolerance.
DISTANCE_TOLERANCE_M = 0.02

robot = serial.Serial(PORT, BAUD, timeout=0.02)
time.sleep(2)

left_ticks = None
right_ticks = None
lock = threading.Lock()
running = True


def read_uart():
    global left_ticks, right_ticks, running

    while running:
        try:
            line = robot.readline().decode(
                "utf-8", errors="ignore"
            ).strip()

            parts = line.split(",")

            if len(parts) >= 3 and parts[0] == "ENC":
                with lock:
                    left_ticks = int(parts[1])
                    right_ticks = int(parts[2])

        except (ValueError, serial.SerialException):
            pass


reader = threading.Thread(target=read_uart, daemon=True)
reader.start()


def send_velocity(left, right):
    # ESP32 expects wheel velocities in m/s.
    robot.write(f"{left:.3f},{right:.3f}\n".encode())


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
    target_ticks = abs(distance_m) * ticks_per_metre
    direction = 1 if distance_m > 0 else -1

    # Limit travel time in case an encoder fails.
    timeout = max(5.0, abs(distance_m) / max(0.05, cruise_speed) * 3)
    start_time = time.monotonic()
    last_report = 0

    print(f"\nMoving {distance_m:+.2f} m (Startup push -> Cruise at {cruise_speed:.2f} m/s)...")

    try:
        while True:
            left, right = get_ticks()

            if left is None or right is None:
                raise RuntimeError("Encoder telemetry lost")

            left_delta = abs(left - start_left)
            right_delta = abs(right - start_right)

            # Average the two wheel distances.
            travelled_ticks = (left_delta + right_delta) / 2
            travelled_m = travelled_ticks / ticks_per_metre
            remaining_m = abs(distance_m) - travelled_m

            if travelled_m >= abs(distance_m) - DISTANCE_TOLERANCE_M:
                break

            if time.monotonic() - start_time > timeout:
                raise TimeoutError(
                    "Movement timed out. Check the wheels and encoders."
                )

            # --- DYNAMIC STARTUP PUSH & CRUISE CONTROL ---
            elapsed = time.monotonic() - start_time
            if elapsed < PUSH_DURATION_S and remaining_m > 0.04:
                # 1. High-power startup push to break static friction (demands 255 PWM from PID)
                current_speed = PUSH_SPEED_MPS
            elif remaining_m < 0.04:
                # 2. Smooth deceleration for the last 4 cm to prevent overshoot
                current_speed = max(0.08, cruise_speed * (remaining_m / 0.04))
            else:
                # 3. Steady cruising speed
                current_speed = cruise_speed

            # Refresh command faster than the ESP32 timeout.
            send_velocity(
                direction * current_speed,
                direction * current_speed,
            )

            if time.monotonic() - last_report > 0.3:
                print(f"  Travelled: {travelled_m:.2f} m / {abs(distance_m):.2f} m (Current: {current_speed:.2f} m/s)")
                last_report = time.monotonic()

            time.sleep(0.05)

    finally:
        stop()

    print(f"Movement complete: approximately {travelled_m:.2f} m\n")


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

