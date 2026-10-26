
import serial
import time

# Change this if your Raspberry Pi uses a different UART device.
SERIAL_PORT = "/dev/serial0"
BAUD_RATE = 115200

SEND_INTERVAL = 0.05  # 20 Hz

def main():
    try:
        esp = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            timeout=0.1
        )
    except serial.SerialException as e:
        print(f"Could not open serial port: {e}")
        return

    time.sleep(2)

    print("Connected to ESP32.")
    print("Enter left and right wheel speeds in m/s.")
    print("Example: 0.25,0.30")
    print("Enter 'stop' to stop the motors.")
    print("Enter 'q' to quit.")

    try:
        while True:
            command = input("\nLeft_mps,Right_mps > ").strip()

            if command.lower() == "q":
                # Send zero speed before exiting.
                esp.write(b"0.0,0.0\n")
                esp.flush()
                break

            if command.lower() == "stop":
                esp.write(b"0.0,0.0\n")
                esp.flush()
                print("Stop command sent.")
                continue

            try:
                parts = command.split(",")

                if len(parts) != 2:
                    print("Format: left_mps,right_mps")
                    continue

                left = float(parts[0].strip())
                right = float(parts[1].strip())

                if not (-2.0 <= left <= 2.0):
                    print("Left velocity must be between -2 and 2 m/s.")
                    continue

                if not (-2.0 <= right <= 2.0):
                    print("Right velocity must be between -2 and 2 m/s.")
                    continue

                # Send one newline-terminated command.
                message = f"{left},{right}\n"
                esp.write(message.encode("ascii"))
                esp.flush()

                print(f"Sent: {message.strip()}")

                # Keep sending the command to satisfy the ESP32 watchdog.
                deadline = time.monotonic() + 1.0

                while time.monotonic() < deadline:
                    esp.write(message.encode("ascii"))
                    esp.flush()
                    time.sleep(SEND_INTERVAL)

            except ValueError:
                print("Invalid numbers. Example: 0.25,0.30")

    except KeyboardInterrupt:
        print("\nInterrupted.")

    finally:
        try:
            esp.write(b"0.0,0.0\n")
            esp.flush()
            time.sleep(0.1)
            esp.close()
        except serial.SerialException:
            pass

        print("Serial connection closed.")


if __name__ == "__main__":
    main()
