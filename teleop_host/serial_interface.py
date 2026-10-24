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

    @staticmethod
    def list_available_ports() -> List[str]:
        """Returns list of device names for connected USB serial ports."""
        ports = serial.tools.list_ports.comports()
        return [p.device for p in ports]

    def connect(self) -> bool:
        """Connect to the specified port or auto-detect an available port."""
        target_port = self.port
        if not target_port:
            available = self.list_available_ports()
            if not available:
                return False
            # Prefer ttyACM (ESP32-S3 native USB) or ttyUSB (UART bridge)
            target_port = available[0]
            for p in available:
                if "ACM" in p or "USB" in p:
                    target_port = p
                    break

        try:
            self._serial = serial.Serial(
                port=target_port,
                baudrate=self.baudrate,
                timeout=0.1,
                write_timeout=0.2,
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
        Triggers an immediate packet transmission.
        """
        cmd_char = cmd_char.upper()
        with self._lock:
            self._current_cmd = cmd_char
            self._current_speed = speed if cmd_char not in ("X", "B") else 0
        self._force_send.set()

    def set_differential_drive(self, pwmL: int, pwmR: int) -> None:
        """
        Set independent Left and Right motor PWM (-255 to +255)
        for autonomous navigation modes (A* APF / Human Follower).
        """
        clamped_l = max(-255, min(255, int(pwmL)))
        clamped_r = max(-255, min(255, int(pwmR)))
        with self._lock:
            self._current_cmd = f"LR,{clamped_l},{clamped_r}"
            self._current_speed = None
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
        except Exception:
            pass

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

