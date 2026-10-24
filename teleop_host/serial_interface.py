import threading
import time
from typing import Optional, List
import serial
import serial.tools.list_ports

from telemetry_model import TelemetryData, parse_telemetry_line


class SerialManager:
    """
    Manages bidirectional USB Serial communication with the ESP32-S3.
    Runs background reader and transmitter threads to service telemetry and
    stream command heartbeats to keep the 300 ms safety watchdog fed.
    """

    def __init__(self, port: Optional[str] = None, baudrate: int = 115200, heartbeat_hz: float = 20.0):
        self.port = port
        self.baudrate = baudrate
        self.heartbeat_interval = 1.0 / heartbeat_hz

        self._serial: Optional[serial.Serial] = None
        self._running = False

        self._rx_thread: Optional[threading.Thread] = None
        self._tx_thread: Optional[threading.Thread] = None

        self._lock = threading.Lock()
        self._latest_telemetry: TelemetryData = TelemetryData()
        self._current_cmd: str = "X"
        self._current_speed: Optional[int] = 0
        self._force_send = threading.Event()

        # Telemetry statistics
        self.packet_count = 0
        self.packet_rate = 0.0
        self._last_rate_calc = time.time()
        self._packets_since_calc = 0

        # TX statistics & diagnostics
        self.tx_packet_count = 0
        self.tx_error_count = 0
        self.last_tx_error = ""
        self.last_sent_packet = ""

    @staticmethod
    def list_available_ports() -> List[str]:
        """Returns list of device names for connected USB serial ports and Raspberry Pi UARTs."""
        import os
        ports = serial.tools.list_ports.comports()
        device_list = [p.device for p in ports]
        # Check standard Raspberry Pi GPIO UART & USB symlinks if present
        for extra in ("/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/serial0", "/dev/ttyAMA0"):
            if os.path.exists(extra) and extra not in device_list:
                device_list.append(extra)
        return device_list

    def connect(self) -> bool:
        """Connect to the specified port or auto-detect an available port."""
        target_port = self.port
        if not target_port:
            available = self.list_available_ports()
            if not available:
                return False
            # Prefer ttyACM (ESP32-S3 native USB) or ttyUSB (UART bridge) or /dev/serial0 (Pi GPIO UART)
            target_port = available[0]
            found_preferred = False
            for p in available:
                if "ACM" in p or "USB" in p:
                    target_port = p
                    found_preferred = True
                    break
            if not found_preferred:
                for p in available:
                    if "serial0" in p or "ttyAMA0" in p:
                        target_port = p
                        break

        try:
            self._serial = serial.Serial(
                port=target_port,
                baudrate=self.baudrate,
                timeout=0.1,
                write_timeout=None,
                rtscts=False,
                dsrdtr=False,
                xonxoff=False,
            )
            self.port = target_port
            self._running = True

            # Start worker threads
            self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True, name="SerialRX")
            self._tx_thread = threading.Thread(target=self._tx_loop, daemon=True, name="SerialTX")
            self._rx_thread.start()
            self._tx_thread.start()

            return True
        except Exception:
            self._serial = None
            self._running = False
            return False

    def is_connected(self) -> bool:
        return self._running and self._serial is not None and self._serial.is_open

    def set_drive_command(self, cmd_char: str, speed: int = 200) -> None:
        """
        Update current drive command ('W', 'S', 'A', 'D', 'X', 'B').
        Triggers an immediate packet transmission only when the command changes,
        allowing heartbeats to stream smoothly at fixed intervals without bursts.
        """
        cmd_char = cmd_char.upper()
        target_speed = speed if cmd_char not in ("X", "B") else 0
        with self._lock:
            changed = (self._current_cmd != cmd_char or self._current_speed != target_speed)
            self._current_cmd = cmd_char
            self._current_speed = target_speed
        if changed:
            self._force_send.set()

    def set_differential_drive(self, pwmL: int, pwmR: int) -> None:
        """
        Set independent Left and Right motor PWM (-255 to +255)
        for autonomous navigation modes (A* APF / Human Follower).
        """
        clamped_l = max(-255, min(255, int(pwmL)))
        clamped_r = max(-255, min(255, int(pwmR)))
        cmd_str = f"LR,{clamped_l},{clamped_r}"
        with self._lock:
            changed = (self._current_cmd != cmd_str or self._current_speed is not None)
            self._current_cmd = cmd_str
            self._current_speed = None
        if changed:
            self._force_send.set()

    def get_telemetry(self) -> TelemetryData:
        with self._lock:
            return self._latest_telemetry

    def _send_packet(self, cmd_char: str, speed: Optional[int]) -> None:
        if not self.is_connected() or not self._serial:
            return
        if speed is not None:
            packet = f"CMD:{cmd_char},{speed}\n".encode("ascii")
        else:
            packet = f"CMD:{cmd_char}\n".encode("ascii")
        try:
            self._serial.write(packet)
            self._serial.flush()
            self.tx_packet_count += 1
            self.last_sent_packet = packet.decode("ascii").strip()
        except Exception as e:
            self.tx_error_count += 1
            self.last_tx_error = str(e)

    def _tx_loop(self) -> None:
        """Periodic transmitter loop to feed the 300 ms ESP32 watchdog."""
        while self._running:
            with self._lock:
                cmd = self._current_cmd
                speed = self._current_speed

            self._send_packet(cmd, speed)

            # Wait for next heartbeat interval or an immediate force_send trigger
            self._force_send.wait(timeout=self.heartbeat_interval)
            self._force_send.clear()

    def _rx_loop(self) -> None:
        """Continuous receiver loop for parsing incoming telemetry lines."""
        while self._running:
            if not self._serial or not self._serial.is_open:
                time.sleep(0.05)
                continue

            try:
                line_bytes = self._serial.readline()
                if not line_bytes:
                    continue

                line_str = line_bytes.decode("utf-8", errors="replace").strip()
                if not line_str:
                    continue

                tele = parse_telemetry_line(line_str)
                if tele is not None:
                    with self._lock:
                        self._latest_telemetry = tele
                    self.packet_count += 1
                    self._packets_since_calc += 1

                # Calculate packet rate every second
                now = time.time()
                elapsed = now - self._last_rate_calc
                if elapsed >= 1.0:
                    self.packet_rate = self._packets_since_calc / elapsed
                    self._packets_since_calc = 0
                    self._last_rate_calc = now

            except Exception:
                time.sleep(0.05)

    def disconnect(self) -> None:
        self._running = False
        self._force_send.set()

        # Send stop command before closing
        try:
            if self._serial and self._serial.is_open:
                self._serial.write(b"CMD:X,0\n")
                self._serial.flush()
                self._serial.close()
        except Exception:
            pass

        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=0.5)
        if self._tx_thread and self._tx_thread.is_alive():
            self._tx_thread.join(timeout=0.5)

    def close(self) -> None:
        self.disconnect()

