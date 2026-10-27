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
MAX_SPEED_MPS = 0.15   # Start slowly for testing.

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

            if len(parts) == 3 and parts[0] == "ENC":
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


def move(distance_m):
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
    timeout = max(5.0, abs(distance_m) / MAX_SPEED_MPS * 3)
    start_time = time.monotonic()
    last_report = 0

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

            if travelled_m >= abs(distance_m) - DISTANCE_TOLERANCE_M:
                break

            if time.monotonic() - start_time > timeout:
                raise TimeoutError(
                    "Movement timed out. Check the wheels and encoders."
                )

            # Refresh command faster than the ESP32 timeout.
            send_velocity(
                direction * MAX_SPEED_MPS,
                direction * MAX_SPEED_MPS,
            )

            if time.monotonic() - last_report > 0.5:
                print(f"Travelled: {travelled_m:.2f} m")
                last_report = time.monotonic()

            time.sleep(0.05)

    finally:
        stop()

    print(f"Movement complete: approximately {travelled_m:.2f} m")


def main():
    print("Robot control ready.")
    print("Commands: move 1 m | move -0.5 m | stop | quit")

    try:
        while True:
            command = input("robot> ").strip().lower()

            if command in ("quit", "exit"):
                break

            if command == "stop":
                stop()
                continue

            parts = command.split()

            if len(parts) == 3 and parts[0] == "move":
                try:
                    distance = float(parts[1])

                    if parts[2] != "m":
                        print("Use metres, e.g. move 1 m")
                        continue

                    move(distance)

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

