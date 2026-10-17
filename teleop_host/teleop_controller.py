#!/usr/bin/env python3
"""
Interactive WASD Keyboard Teleoperation & Live Telemetry Dashboard for ESP32-S3.
Streams drive commands over USB Serial at 115200 baud while parsing real-time
encoder telemetry and rendering a live Rich terminal dashboard.
Supports simultaneous multi-key presses (W+A, W+D, S+A, S+D).
"""

import argparse
import sys
import time
import threading
from typing import Optional, Set

from rich.console import Console
from rich.live import Live
from rich.table import Table
from rich.panel import Panel
from rich.layout import Layout
from rich.text import Text

from serial_interface import SerialManager
from telemetry_model import TelemetryData

console = Console()

# Detect pynput availability
HAVE_PYNPUT = False
try:
    from pynput import keyboard
    HAVE_PYNPUT = True
except ImportError:
    HAVE_PYNPUT = False


def resolve_command_from_keys(active_keys: Set[str]) -> str:
    """
    Resolve simultaneous key combinations into driving commands:
      Space       -> B (Active Brake)
      W + A       -> WA (Forward-Left arc)
      W + D       -> WD (Forward-Right arc)
      S + A       -> SA (Reverse-Left arc)
      S + D       -> SD (Reverse-Right arc)
      W           -> W (Forward)
      S           -> S (Reverse)
      A           -> A (Left)
      D           -> D (Right)
      Conflicting -> X (Stop)
    """
    if " " in active_keys:
        return "B"

    has_w = "w" in active_keys
    has_s = "s" in active_keys
    has_a = "a" in active_keys
    has_d = "d" in active_keys

    # Check forward/reverse conflict
    fwd_rev = None
    if has_w and not has_s:
        fwd_rev = "W"
    elif has_s and not has_w:
        fwd_rev = "S"

    # Check left/right conflict
    left_right = None
    if has_a and not has_d:
        left_right = "A"
    elif has_d and not has_a:
        left_right = "D"

    # Compound diagonal motion
    if fwd_rev and left_right:
        return f"{fwd_rev}{left_right}"
    elif fwd_rev:
        return fwd_rev
    elif left_right:
        return left_right
    else:
        return "X"


class KeyboardController:
    """
    Asynchronous keyboard controller supporting multi-key chord detection
    both via pynput (desktop GUI) and raw terminal stdin decay (headless/SSH).
    """

    def __init__(self, on_command_change):
        self.on_command_change = on_command_change
        self.active_keys: Set[str] = set()
        self.speed = 200
        self.running = True
        self._listener = None
        self._terminal_thread = None

    def start(self):
        use_pynput = False
        if HAVE_PYNPUT:
            try:
                self._listener = keyboard.Listener(
                    on_press=self._on_pynput_press,
                    on_release=self._on_pynput_release,
                )
                self._listener.start()
                use_pynput = True
            except Exception:
                use_pynput = False

        if not use_pynput:
            self._terminal_thread = threading.Thread(
                target=self._terminal_raw_loop, daemon=True, name="TermKeyReader"
            )
            self._terminal_thread.start()

    def stop(self):
        self.running = False
        if self._listener:
            try:
                self._listener.stop()
            except Exception:
                pass

    def _on_pynput_press(self, key):
        try:
            char = key.char.lower() if hasattr(key, "char") and key.char else None
        except Exception:
            char = None

        if char in ("w", "a", "s", "d", " "):
            if char not in self.active_keys:
                self.active_keys.add(char)
                self.on_command_change(resolve_command_from_keys(self.active_keys), self.speed)
        elif char in ("+", "="):
            self.speed = min(255, self.speed + 10)
            self.on_command_change(resolve_command_from_keys(self.active_keys), self.speed)
        elif char in ("-", "_"):
            self.speed = max(50, self.speed - 10)
            self.on_command_change(resolve_command_from_keys(self.active_keys), self.speed)
        elif char == "q":
            self.running = False

    def _on_pynput_release(self, key):
        try:
            char = key.char.lower() if hasattr(key, "char") and key.char else None
        except Exception:
            char = None

        if char in self.active_keys:
            self.active_keys.discard(char)
            self.on_command_change(resolve_command_from_keys(self.active_keys), self.speed)

    def _terminal_raw_loop(self):
        """Cross-platform non-blocking terminal input with multi-key decay tracking."""
        if sys.platform == "win32":
            import msvcrt
            key_last_seen = {}
            while self.running:
                now = time.time()
                while msvcrt.kbhit():
                    ch = msvcrt.getch().decode("ascii", errors="ignore").lower()
                    if ch in ("w", "a", "s", "d", " "):
                        key_last_seen[ch] = now
                    elif ch in ("+", "="):
                        self.speed = min(255, self.speed + 10)
                    elif ch in ("-", "_"):
                        self.speed = max(50, self.speed - 10)
                    elif ch == "q":
                        self.running = False
                        return
                active = {k for k, ts in key_last_seen.items() if (now - ts) < 0.45}
                if active != self.active_keys:
                    self.active_keys = active
                    self.on_command_change(resolve_command_from_keys(self.active_keys), self.speed)
                time.sleep(0.02)
        else:
            import termios
            import tty
            import select

            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setcbreak(fd)
                key_last_seen = {}

                while self.running:
                    rlist, _, _ = select.select([sys.stdin], [], [], 0.03)
                    now = time.time()

                    if rlist:
                        chunk = sys.stdin.read(1).lower()
                        # Drain any additional pending keystrokes in buffer
                        while select.select([sys.stdin], [], [], 0.0)[0]:
                            chunk += sys.stdin.read(1).lower()

                        for ch in chunk:
                            if ch in ("w", "a", "s", "d", " "):
                                if ch == " ":
                                    key_last_seen = {" ": now}
                                else:
                                    key_last_seen.pop(" ", None)
                                    key_last_seen[ch] = now
                            elif ch in ("+", "="):
                                self.speed = min(255, self.speed + 10)
                                self.on_command_change(resolve_command_from_keys(self.active_keys), self.speed)
                            elif ch in ("-", "_"):
                                self.speed = max(50, self.speed - 10)
                                self.on_command_change(resolve_command_from_keys(self.active_keys), self.speed)
                            elif ch == "q":
                                self.running = False
                                break

                    # Decay inactive keys (450 ms window accommodates repeat delays)
                    active = {k for k, ts in key_last_seen.items() if (now - ts) < 0.45}
                    if active != self.active_keys:
                        self.active_keys = active
                        self.on_command_change(resolve_command_from_keys(self.active_keys), self.speed)

            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def make_gauge_bar(rpm: float, max_rpm: float = 120.0, width: int = 20) -> str:
    """Render an ASCII colored bar for RPM representation."""
    abs_rpm = abs(rpm)
    filled = int(min(1.0, abs_rpm / max_rpm) * width)
    unfilled = width - filled
    direction_sign = "+" if rpm >= 0 else "-"

    if abs_rpm > 80:
        color = "red"
    elif abs_rpm > 40:
        color = "yellow"
    else:
        color = "green"

    return f"[{color}]{direction_sign} {'█' * filled}{'░' * unfilled}[/] ({rpm:+.1f} RPM)"


def generate_dashboard(
    serial_mgr: SerialManager,
    tele: TelemetryData,
    active_cmd: str,
    active_speed: int,
    active_keys: Set[str],
) -> Panel:
    """Generate the complete Rich dashboard layout."""
    # Top Status Banner
    grid = Table.grid(expand=True)
    grid.add_column(justify="left", ratio=1)
    grid.add_column(justify="right", ratio=1)

    port_str = serial_mgr.port or "AUTO-SEARCH"
    conn_status = (
        "[bold green]CONNECTED[/]" if serial_mgr.is_connected() else "[bold red]DISCONNECTED[/]"
    )
    grid.add_row(
        f" [bold cyan]Port:[/] {port_str} ({serial_mgr.baudrate} baud)",
        f"[bold cyan]Status:[/] {conn_status} | [bold cyan]Rate:[/] {serial_mgr.packet_rate:.1f} Hz ",
    )

    # Command Table
    cmd_table = Table(title="[bold yellow]Driver & Command Control[/]", expand=True)
    cmd_table.add_column("Command State", justify="center")
    cmd_table.add_column("Target PWM", justify="center")
    cmd_table.add_column("Keys Active", justify="center")
    cmd_table.add_column("ESP32 Feedback", justify="center")
    cmd_table.add_column("Watchdog Link", justify="center")

    cmd_colors = {
        "W": "[bold green]FORWARD (W)[/]",
        "S": "[bold cyan]REVERSE (S)[/]",
        "A": "[bold yellow]LEFT (A)[/]",
        "D": "[bold yellow]RIGHT (D)[/]",
        "WA": "[bold green]FWD-LEFT (W+A)[/]",
        "WD": "[bold green]FWD-RIGHT (W+D)[/]",
        "SA": "[bold cyan]REV-LEFT (S+A)[/]",
        "SD": "[bold cyan]REV-RIGHT (S+D)[/]",
        "B": "[bold red]ACTIVE BRAKE (B)[/]",
        "X": "[dim white]STOPPED (X)[/]",
    }
    cmd_display = cmd_colors.get(active_cmd, f"[white]{active_cmd}[/]")

    wdg_display = (
        "[bold red]TRIPPED (WDG)[/]" if tele.is_watchdog_tripped else "[bold green]HEALTHY (OK)[/]"
    )

    keys_str = ", ".join(k.upper() for k in sorted(active_keys)) if active_keys else "None"

    cmd_table.add_row(
        cmd_display,
        f"{active_speed} / 255",
        f"[bold magenta]{keys_str}[/]",
        f"[bold]{tele.state}[/]",
        wdg_display,
    )

    # Encoders & Odometry Table
    enc_table = Table(title="[bold yellow]Quadrature Wheel Odometry[/]", expand=True)
    enc_table.add_column("Wheel", justify="center")
    enc_table.add_column("Cumulative Ticks", justify="center")
    enc_table.add_column("Velocity Gauge & Real-Time RPM", justify="left")

    gauge_l = make_gauge_bar(tele.rpm_left)
    gauge_r = make_gauge_bar(tele.rpm_right)

    enc_table.add_row("[bold cyan]Left Wheel[/]", f"{tele.ticks_left:,}", gauge_l)
    enc_table.add_row("[bold magenta]Right Wheel[/]", f"{tele.ticks_right:,}", gauge_r)

    # Keybinding guide
    help_text = Text.from_markup(
        " [bold yellow]Multi-Key Controls:[/] [white]W+A[/]=Fwd-Left | [white]W+D[/]=Fwd-Right | "
        "[white]S+A[/]=Rev-Left | [white]S+D[/]=Rev-Right | [white]Space[/]=Brake | [white]Release[/]=Stop | "
        "[white]+/-[/]=Speed | [white]Q[/]=Quit\n"
        f" [dim]Raw Packet: {tele.raw_packet or 'None'}[/]"
    )

    # Combined Layout
    layout = Layout()
    layout.split_column(
        Layout(grid, size=2),
        Layout(cmd_table, size=5),
        Layout(enc_table, size=6),
        Layout(help_text, size=3),
    )

    return Panel(
        layout,
        title="[bold green]ESP32-S3 ROBOT CAR TELEOPERATION[/]",
        border_style="bright_blue",
        padding=(0, 1),
    )


def main():
    parser = argparse.ArgumentParser(description="ESP32-S3 Robot Car WASD Teleoperation")
    parser.add_argument("--port", type=str, default=None, help="Serial port (e.g. /dev/ttyACM0, COM3)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--speed", type=int, default=200, help="Initial PWM drive speed 0-255 (default: 200)")
    args = parser.parse_args()

    serial_mgr = SerialManager(port=args.port, baudrate=args.baud)
    if not serial_mgr.connect():
        console.print(
            f"[bold red]Error:[/] Could not open serial port '{args.port or 'auto'}'. "
            f"Available ports: {SerialManager.list_available_ports()}"
        )
        return 1

    current_cmd = "X"
    current_speed = args.speed

    def on_command_change(cmd: str, speed: int):
        nonlocal current_cmd, current_speed
        current_cmd = cmd
        current_speed = speed
        serial_mgr.set_drive_command(cmd, speed)

    kb = KeyboardController(on_command_change)
    kb.speed = args.speed
    kb.start()

    console.clear()
    try:
        with Live(console=console, screen=True, refresh_per_second=20) as live:
            while kb.running:
                tele = serial_mgr.get_telemetry()
                dashboard = generate_dashboard(
                    serial_mgr, tele, current_cmd, current_speed, kb.active_keys
                )
                live.update(dashboard)
                time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        kb.stop()
        serial_mgr.disconnect()
        console.clear()
        console.print("[bold yellow]Teleoperation session ended. Motors stopped.[/]")

    return 0


if __name__ == "__main__":
    sys.exit(main())
