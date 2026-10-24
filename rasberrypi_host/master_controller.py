#!/usr/bin/env python3
"""
Master State Machine Controller for CNL2 Autonomous Robot Car.
Orchestrates:
  1. TELEOP: Real-time WASD keyboard driving with multi-key chords (W+A, W+D, S+A, S+D)
  2. HUMAN_FOLLOW: Autonomous vision-based person tracking & following using APF
  3. GRID_NAV: Autonomous A* global waypoint navigation + APF dynamic obstacle avoidance
  4. IDLE / BRAKE: Safe holding & active electronic braking

All commands route through this Master State Machine and stream over USB Serial at 115200 baud.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time
from enum import Enum
from pathlib import Path
from typing import Any, Callable, Dict, Optional, Set, Tuple

# Ensure search paths include local directory, parent, and teleop_host
CURRENT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = CURRENT_DIR.parent
TELEOP_DIR = PROJECT_ROOT / "teleop_host"
VISION_DIR = CURRENT_DIR / "vision"

for p in (str(CURRENT_DIR), str(PROJECT_ROOT), str(TELEOP_DIR), str(VISION_DIR)):
    if p not in sys.path:
        sys.path.insert(0, p)

# Terminal UI
from rich.console import Console
from rich.layout import Layout
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

# Serial communications & Telemetry
from serial_interface import SerialManager
from telemetry_model import TelemetryData

# A* Search & Kinematics
try:
    from a_search_main import (
        create_empty_grid, astar_grid, is_path_blocked, mark_obstacle,
        world_to_grid, calculate_apf, calculate_kinematics, SENSOR_ANGLES,
    )
    HAVE_ASTAR = True
except Exception as e:
    HAVE_ASTAR = False

# Vision Pipeline Bridge
try:
    from config import Config
    from vision_bridge import VisionBridge, make_target_fn, make_stop_fn
    HAVE_VISION = True
except Exception:
    HAVE_VISION = False

# Keyboard input detection (pynput for desktop, termios for headless SSH)
HAVE_PYNPUT = False
try:
    from pynput import keyboard
    HAVE_PYNPUT = True
except ImportError:
    HAVE_PYNPUT = False

console = Console()


# ==============================================================================
# Master State Machine Modes
# ==============================================================================

class MasterState(str, Enum):
    IDLE = "IDLE"
    TELEOP = "TELEOP"
    HUMAN_FOLLOW = "HUMAN_FOLLOW"
    GRID_NAV = "GRID_NAV"
    BRAKE = "BRAKE"
    ERROR = "ERROR"


# ==============================================================================
# WASD Key Resolution
# ==============================================================================

def resolve_wasd_command(active_keys: Set[str]) -> str:
    """
    Resolve simultaneous keys into driving commands:
      Space       -> B (Active Brake)
      W + A       -> WA (Forward-Left arc)
      W + D       -> WD (Forward-Right arc)
      S + A       -> SA (Reverse-Left arc)
      S + D       -> SD (Reverse-Right arc)
      W           -> W (Forward)
      S           -> S (Reverse)
      A           -> A (Pivot Left)
      D           -> D (Pivot Right)
      Conflicting -> X (Stop)
    """
    if " " in active_keys:
        return "B"

    has_w = "w" in active_keys
    has_s = "s" in active_keys
    has_a = "a" in active_keys
    has_d = "d" in active_keys

    fwd_rev = None
    if has_w and not has_s:
        fwd_rev = "W"
    elif has_s and not has_w:
        fwd_rev = "S"

    left_right = None
    if has_a and not has_d:
        left_right = "A"
    elif has_d and not has_a:
        left_right = "D"

    if fwd_rev and left_right:
        return f"{fwd_rev}{left_right}"
    elif fwd_rev:
        return fwd_rev
    elif left_right:
        return left_right
    else:
        return "X"


# ==============================================================================
# Autonomous Velocity to Motor PWM Converter
# ==============================================================================

def velocity_to_pwm(v_m_s: float, max_v: float = 1.0, min_pwm: int = 50, max_pwm: int = 255) -> int:
    """Map linear wheel velocity (m/s) to motor PWM voltage (-255 to +255)."""
    if abs(v_m_s) < 0.02:
        return 0
    sign = 1 if v_m_s > 0 else -1
    ratio = min(1.0, abs(v_m_s) / max_v)
    pwm = int(min_pwm + ratio * (max_pwm - min_pwm))
    return sign * pwm


# ==============================================================================
# Subsystem: Autonomous Human Follower (Vision + APF)
# ==============================================================================

class HumanFollower:
    """Follows a tracked human target maintaining safe distance using APF."""

    def __init__(self, target_fn: Callable[[], Optional[Tuple[float, float]]], stop_fn: Callable[[], bool], follow_dist: float = 1.0):
        self.target_fn = target_fn
        self.stop_fn = stop_fn
        self.follow_dist = follow_dist
        self.lost_ticks = 0
        self.max_lost_ticks = 15  # 1.5 seconds at 10 Hz

    def step(self, robot_x: float, robot_y: float, robot_theta: float, tof_readings: Tuple[float, float, float]) -> Tuple[str, float, float]:
        target = self.target_fn()
        if target is None:
            self.lost_ticks += 1
            if self.lost_ticks > self.max_lost_ticks:
                return "LOST", 0.0, 0.0
            return "WAITING", 0.0, 0.0

        self.lost_ticks = 0
        if self.stop_fn():
            return "HUMAN_STOPPED", 0.0, 0.0

        pos = (robot_x, robot_y)
        dx = target[0] - pos[0]
        dy = target[1] - pos[1]
        dist = math.hypot(dx, dy)

        if dist <= self.follow_dist:
            return "HOLD_DISTANCE", 0.0, 0.0

        k = (dist - self.follow_dist) / dist
        goal = (pos[0] + dx * k, pos[1] + dy * k)

        if HAVE_ASTAR:
            f_total = calculate_apf(pos, goal, tof_readings, robot_theta)
            vl, vr = calculate_kinematics(f_total, robot_theta)
        else:
            # Fallback simple proportional turning
            angle_err = math.atan2(dy, dx) - robot_theta
            angle_err = math.atan2(math.sin(angle_err), math.cos(angle_err))
            v = min(0.5, dist * 0.3)
            vl = v - angle_err * 0.2
            vr = v + angle_err * 0.2

        return "FOLLOWING", vl, vr


# ==============================================================================
# Subsystem: Autonomous Grid Navigator (A* + Dynamic APF)
# ==============================================================================

class GridNavigator:
    """Global A* waypoint planner with local APF obstacle avoidance."""

    def __init__(self):
        self.grid = create_empty_grid() if HAVE_ASTAR else None
        self.goal: Optional[Tuple[float, float]] = None
        self.waypoints: list[Tuple[float, float]] = []

    def start(self, start_pos: Tuple[float, float], goal_pos: Tuple[float, float]) -> bool:
        if not HAVE_ASTAR:
            return False
        self.grid = create_empty_grid()
        self.goal = goal_pos
        self.waypoints = astar_grid(start_pos, goal_pos, self.grid)
        if not self.waypoints:
            return False
        if len(self.waypoints) > 1:
            self.waypoints.pop(0)  # Remove current position
        return True

    def step(self, pos: Tuple[float, float], theta: float, tof: Tuple[float, float, float]) -> Tuple[str, float, float]:
        if not HAVE_ASTAR or not self.waypoints:
            return "DONE", 0.0, 0.0

        # Dynamic obstacle mapping from distance sensors
        map_updated = False
        num_sensors = min(len(tof), len(SENSOR_ANGLES))
        for i in range(num_sensors):
            if tof[i] < 2.0:
                a = theta + SENSOR_ANGLES[i]
                ox = pos[0] + tof[i] * math.cos(a)
                oy = pos[1] + tof[i] * math.sin(a)
                gx, gy = world_to_grid(ox, oy)
                if self.grid[gx, gy] == 0:
                    mark_obstacle(self.grid, ox, oy)
                    map_updated = True

        # Dynamic replan if obstacle blocks upcoming waypoint
        if map_updated and is_path_blocked(self.waypoints, self.grid) and self.goal:
            new_plan = astar_grid(pos, self.goal, self.grid)
            if not new_plan:
                return "BLOCKED", 0.0, 0.0
            self.waypoints = new_plan[1:] if len(new_plan) > 1 else new_plan

        # Pop reached waypoints (within 15 cm)
        while self.waypoints and math.dist(pos, self.waypoints[0]) < 0.15:
            self.waypoints.pop(0)

        if not self.waypoints:
            return "REACHED_GOAL", 0.0, 0.0

        target_wp = self.waypoints[0]
        f = calculate_apf(pos, target_wp, tof, theta)
        vl, vr = calculate_kinematics(f, theta)
        return "NAVIGATING", vl, vr


# ==============================================================================
# Master State Machine & Keyboard Handler
# ==============================================================================

class MasterStateMachine:
    """
    Central Controller for the CNL2 Robot Car.
    Coordinates modes, processes WASD inputs, dispatches commands to the ESP32,
    and renders the real-time live dashboard.
    """

    def __init__(self, port: Optional[str] = None, baudrate: int = 115200, default_speed: int = 200):
        self.port = port
        self.baudrate = baudrate
        self.speed = default_speed
        self.active_state = MasterState.TELEOP
        self.running = True

        # Communication
        self.serial_mgr = SerialManager(port=port, baudrate=baudrate)

        # Keyboard Tracking & SSH Adaptations
        self.is_ssh = bool(os.environ.get("SSH_CLIENT") or os.environ.get("SSH_TTY") or not os.environ.get("DISPLAY"))
        self.key_decay_s = 0.85 if self.is_ssh else 0.70
        self.cruise_mode = False  # Toggle between Momentary (hold) and Cruise (latched) mode
        self.active_keys: Set[str] = set()
        self.key_last_seen: Dict[str, float] = {}
        self.last_wasd_cmd = "X"

        # Autonomous Subsystems
        self.target_fn = lambda: None
        self.stop_fn = lambda: False
        self.vision_bridge: Optional[VisionBridge] = None

        self.follower = HumanFollower(self.target_fn, self.stop_fn)
        self.navigator = GridNavigator()

        # Telemetry estimated pose
        self.est_x = 1.0
        self.est_y = 1.0
        self.est_theta = 0.0
        self.tof_readings = (2.0, 2.0, 2.0)
        self.subsystem_status = "IDLE"

        # Worker threads
        self._input_thread: Optional[threading.Thread] = None
        self._listener = None

    def set_mode(self, new_mode: MasterState):
        """Switch Master State Machine mode safely."""
        if self.active_state == new_mode:
            return
        old_mode = self.active_state
        self.active_state = new_mode

        if new_mode == MasterState.TELEOP:
            self.serial_mgr.set_drive_command("X", 0)
        elif new_mode == MasterState.HUMAN_FOLLOW:
            self.follower.lost_ticks = 0
            self.subsystem_status = "STARTING_VISION_FOLLOW"
        elif new_mode == MasterState.GRID_NAV:
            start = (self.est_x, self.est_y)
            goal = (9.0, 9.0)
            ok = self.navigator.start(start, goal)
            self.subsystem_status = "PLANNING_ASTAR" if ok else "ASTAR_FAILED"
            if not ok:
                self.active_state = MasterState.IDLE
        elif new_mode == MasterState.BRAKE:
            self.serial_mgr.set_drive_command("B", 0)
        elif new_mode == MasterState.IDLE:
            self.serial_mgr.set_drive_command("X", 0)

    def initialize_vision(self):
        """Initialize lightweight camera pipeline if available."""
        if not HAVE_VISION:
            return
        try:
            models_dir = VISION_DIR / "models"
            tflite_path = models_dir / "detect.tflite"
            model_file = tflite_path if tflite_path.is_file() else models_dir / "yolo26n.pt"

            cfg = Config(
                model_path=model_file,
                debug_view=False,
                inference_threads=2,
            )
            self.vision_bridge = VisionBridge.from_config(cfg).start()
            self.target_fn = make_target_fn(self.vision_bridge, None)
            self.stop_fn = make_stop_fn(self.vision_bridge)
            self.follower.target_fn = self.target_fn
            self.follower.stop_fn = self.stop_fn
        except Exception:
            pass

    # ──────────────────────────────────────────────────────────────────────────
    # Input Handling (Desktop GUI + Raw Headless Terminal)
    # ──────────────────────────────────────────────────────────────────────────

    def start_input(self):
        if HAVE_PYNPUT and os.environ.get("DISPLAY"):
            try:
                self._listener = keyboard.Listener(
                    on_press=self._on_pynput_press,
                    on_release=self._on_pynput_release,
                )
                self._listener.start()
                return
            except Exception:
                pass

        # Headless terminal raw mode
        self._input_thread = threading.Thread(target=self._terminal_raw_loop, daemon=True, name="RawInput")
        self._input_thread.start()

    def _on_pynput_press(self, key):
        try:
            if hasattr(key, "char") and key.char:
                char = key.char.lower()
                self._handle_char_press(char)
            elif key == keyboard.Key.space:
                self._handle_char_press(" ")
        except Exception:
            pass

    def _on_pynput_release(self, key):
        try:
            if hasattr(key, "char") and key.char:
                char = key.char.lower()
                self.active_keys.discard(char)
            elif key == keyboard.Key.space:
                self.active_keys.discard(" ")
        except Exception:
            pass

    def _handle_char_press(self, char: str):
        now = time.time()
        self.key_last_seen[char] = now

        # Mode switching hotkeys
        if char in ("t", "1"):
            self.set_mode(MasterState.TELEOP)
            return
        elif char in ("f", "2"):
            self.set_mode(MasterState.HUMAN_FOLLOW)
            return
        elif char in ("n", "3"):
            self.set_mode(MasterState.GRID_NAV)
            return
        elif char in ("i", "0"):
            self.set_mode(MasterState.IDLE)
            return
        elif char == "c":
            # Toggle between Momentary (hold) and Cruise (latched) mode
            self.cruise_mode = not self.cruise_mode
            if not self.cruise_mode:
                self.active_keys.clear()
            return
        elif char in ("+", "="):
            self.speed = min(255, self.speed + 15)
            return
        elif char in ("-", "_"):
            self.speed = max(50, self.speed - 15)
            return
        elif char == "q":
            self.running = False
            return

        # Cruise control / Latched logic (ideal for SSH)
        if self.cruise_mode:
            if char in (" ", "x"):
                self.active_keys.clear()
                if char == " ":
                    self.active_keys.add(" ")
            elif char == "w":
                self.active_keys = {"w"}
            elif char == "s":
                self.active_keys = {"s"}
            elif char == "a":
                if "w" in self.active_keys:
                    self.active_keys = {"w", "a"}
                elif "s" in self.active_keys:
                    self.active_keys = {"s", "a"}
                else:
                    self.active_keys = {"a"}
            elif char == "d":
                if "w" in self.active_keys:
                    self.active_keys = {"w", "d"}
                elif "s" in self.active_keys:
                    self.active_keys = {"s", "d"}
                else:
                    self.active_keys = {"d"}
            return

        # Momentary hold logic
        if char in ("w", "s", "a", "d", " "):
            self.active_keys.add(char)
        elif char == "x":
            self.active_keys.clear()

    def _terminal_raw_loop(self):
        import select
        import termios
        import tty

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while self.running:
                rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
                if rlist:
                    chars = sys.stdin.read(1)
                    if chars:
                        self._handle_char_press(chars.lower())

                # Clean up decayed keys in momentary mode (Linux TTY key-up emulation)
                if not self.cruise_mode:
                    now = time.time()
                    decayed = [k for k, last_t in self.key_last_seen.items() if now - last_t > self.key_decay_s]
                    for k in decayed:
                        self.active_keys.discard(k)
                        del self.key_last_seen[k]
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    # ──────────────────────────────────────────────────────────────────────────
    # State Machine Main Loop
    # ──────────────────────────────────────────────────────────────────────────

    def tick(self):
        """Execute one iteration of the Master State Machine."""
        telemetry = self.serial_mgr.get_telemetry()

        # Update dynamic ToF distance sensor readings if arriving in telemetry
        if telemetry.has_distance_sensors:
            self.tof_readings = telemetry.tof_meters

        # Check safety watchdog connection
        if not self.serial_mgr.is_connected():
            if self.active_state not in (MasterState.IDLE, MasterState.ERROR):
                self.subsystem_status = "SERIAL_DISCONNECTED"
            return

        # 1. TELEOP MODE (Direct WASD Manual Control)
        if self.active_state == MasterState.TELEOP:
            cmd = resolve_wasd_command(self.active_keys)
            self.last_wasd_cmd = cmd
            if cmd == "B":
                self.serial_mgr.set_drive_command("B", 0)
                self.subsystem_status = "BRAKING"
            elif cmd == "X":
                self.serial_mgr.set_drive_command("X", 0)
                self.subsystem_status = "STOPPED"
            else:
                self.serial_mgr.set_drive_command(cmd, self.speed)
                self.subsystem_status = f"DRIVING_{cmd}"

        # 2. HUMAN FOLLOW MODE (Autonomous Vision Tracking)
        elif self.active_state == MasterState.HUMAN_FOLLOW:
            status, vl, vr = self.follower.step(
                self.est_x, self.est_y, self.est_theta, self.tof_readings
            )
            self.subsystem_status = status
            pwmL = velocity_to_pwm(vl, max_pwm=self.speed)
            pwmR = velocity_to_pwm(vr, max_pwm=self.speed)
            self.serial_mgr.set_differential_drive(pwmL, pwmR)

        # 3. GRID NAVIGATION MODE (Autonomous A* Planner)
        elif self.active_state == MasterState.GRID_NAV:
            pos = (self.est_x, self.est_y)
            status, vl, vr = self.navigator.step(pos, self.est_theta, self.tof_readings)
            self.subsystem_status = status
            if status == "REACHED_GOAL":
                self.active_state = MasterState.IDLE
                self.serial_mgr.set_drive_command("X", 0)
            else:
                pwmL = velocity_to_pwm(vl, max_pwm=self.speed)
                pwmR = velocity_to_pwm(vr, max_pwm=self.speed)
                self.serial_mgr.set_differential_drive(pwmL, pwmR)

        # 4. ACTIVE BRAKE MODE
        elif self.active_state == MasterState.BRAKE:
            self.serial_mgr.set_drive_command("B", 0)
            self.subsystem_status = "ACTIVE_BRAKE"

        # 5. IDLE / STOP MODE
        elif self.active_state == MasterState.IDLE:
            self.serial_mgr.set_drive_command("X", 0)
            self.subsystem_status = "IDLE_STANDBY"

    # ──────────────────────────────────────────────────────────────────────────
    # Rich Live UI Dashboard
    # ──────────────────────────────────────────────────────────────────────────

    def render_dashboard(self) -> Layout:
        layout = Layout()
        layout.split_column(
            Layout(name="header", size=3),
            Layout(name="banner", size=3),
            Layout(name="tables"),
            Layout(name="footer", size=3),
        )

        tele = self.serial_mgr.get_telemetry()
        conn_str = "[bold green]CONNECTED[/]" if self.serial_mgr.is_connected() else "[bold red]DISCONNECTED[/]"
        port_str = self.serial_mgr.port or "AUTO-SEARCH"

        # Header
        tx_stat = f"[bold cyan]TX:[/] {self.serial_mgr.tx_packet_count}"
        if self.serial_mgr.tx_error_count > 0:
            tx_stat += f" ([bold red]Err: {self.serial_mgr.tx_error_count}[/])"
        header_text = Text.from_markup(
            f" [bold cyan]Port:[/] {port_str} | [bold cyan]Status:[/] {conn_str} | "
            f"[bold cyan]RX Rate:[/] {self.serial_mgr.packet_rate:.1f} Hz | "
            f"{tx_stat} | [bold cyan]Drive PWM:[/] [bold yellow]{self.speed}/255[/]"
        )
        layout["header"].update(Panel(header_text, style="cyan"))

        # Master State Banner
        mode_colors = {
            MasterState.TELEOP: "[bold black on green]  ★ MODE: MANUAL TELEOP (WASD ACTIVE)  [/]",
            MasterState.HUMAN_FOLLOW: "[bold black on cyan]  ★ MODE: AUTONOMOUS HUMAN FOLLOW (VISION)  [/]",
            MasterState.GRID_NAV: "[bold black on magenta]  ★ MODE: AUTONOMOUS GRID NAVIGATION (A*)  [/]",
            MasterState.IDLE: "[bold white on dark_blue]  ★ MODE: IDLE (STANDBY)  [/]",
            MasterState.BRAKE: "[bold white on red]  ★ MODE: ACTIVE BRAKE  [/]",
            MasterState.ERROR: "[bold white on red]  ★ MODE: ERROR / STOP  [/]",
        }
        banner_content = mode_colors.get(self.active_state, f"MODE: {self.active_state}")
        layout["banner"].update(Panel(Text.from_markup(f"{banner_content}  |  Subsystem: [bold yellow]{self.subsystem_status}[/]"), style="bold"))

        # Tables Split (Left: Odometry + Distance Sensors, Right: State Machine & Controls)
        layout["tables"].split_row(
            Layout(name="left_col", ratio=1),
            Layout(name="right_col", ratio=1),
        )
        layout["tables"]["left_col"].split_column(
            Layout(name="odometry", size=7),
            Layout(name="distance_sensors", size=8),
        )

        is_connected = self.serial_mgr.is_connected()
        if not is_connected:
            motor_status = "[dim red]DISCONNECTED[/]"
            wdg_display = "[bold red]OFFLINE (NO LINK)[/]"
        elif tele.is_watchdog_tripped:
            motor_status = f"[bold yellow]{tele.state}[/]"
            wdg_display = "[bold red]TRIPPED (TIMEOUT)[/]"
        elif tele.is_healthy:
            motor_status = f"[bold green]{tele.state}[/]"
            wdg_display = "[bold green]HEALTHY (OK)[/]"
        else:
            motor_status = f"[dim]{tele.state}[/]"
            wdg_display = f"[yellow]{tele.flags}[/]"

        # Odometry Table
        enc_table = Table(title="[bold yellow]ESP32 Wheel Odometry[/]", expand=True)
        enc_table.add_column("Wheel", justify="center")
        enc_table.add_column("Encoder Ticks", justify="center")
        enc_table.add_column("Speed (RPM)", justify="center")
        enc_table.add_column("Motor Status", justify="center")

        enc_table.add_row("Left Wheel (M1)", f"{tele.ticks_left:,}", f"{tele.rpm_left:+.1f}", motor_status)
        enc_table.add_row("Right Wheel (M2)", f"{tele.ticks_right:,}", f"{tele.rpm_right:+.1f}", motor_status)
        layout["tables"]["left_col"]["odometry"].update(enc_table)

        # Distance Sensors Table (3x VL53L0X ToF)
        dist_table = Table(title="[bold yellow]VL53L0X Distance Sensors (ToF)[/]", expand=True)
        dist_table.add_column("Sensor", justify="center")
        dist_table.add_column("Angle", justify="center")
        dist_table.add_column("Distance", justify="center")
        dist_table.add_column("Proximity Status", justify="center")

        def _fmt_tof(name: str, angle: str, d_mm: int):
            if not is_connected or d_mm < 0:
                return (name, angle, "[dim]-- mm[/]", "[dim]N/A (OFFLINE)[/]")
            cm = d_mm / 10.0
            if d_mm < 200:
                return (name, angle, f"[bold red]{d_mm} mm ({cm:.1f} cm)[/]", "[bold red]DANGER (<20cm)[/]")
            elif d_mm < 400:
                return (name, angle, f"[bold yellow]{d_mm} mm ({cm:.1f} cm)[/]", "[bold yellow]CAUTION (<40cm)[/]")
            else:
                return (name, angle, f"[bold green]{d_mm} mm ({cm:.1f} cm)[/]", "[bold green]CLEAR[/]")

        dist_table.add_row(*_fmt_tof("Left (S1)", "+45°", tele.dist_left))
        dist_table.add_row(*_fmt_tof("Center (S2)", "0° FWD", tele.dist_center))
        dist_table.add_row(*_fmt_tof("Right (S3)", "-45°", tele.dist_right))
        layout["tables"]["left_col"]["distance_sensors"].update(dist_table)

        # Command & Navigation Table
        nav_table = Table(title="[bold yellow]State Machine & Controls[/]", expand=True)
        nav_table.add_column("Property", justify="left")
        nav_table.add_column("Current Value", justify="center")

        pressed_str = ", ".join(sorted(self.active_keys)).upper() if self.active_keys else "NONE"
        drive_mode_str = "[bold cyan]CRUISE (LATCHED)[/]" if self.cruise_mode else "[bold green]MOMENTARY (HOLD)[/]"
        nav_table.add_row("Drive Input Mode", drive_mode_str)
        nav_table.add_row("Active Held Keys", f"[bold white]{pressed_str}[/]")
        nav_table.add_row("Last Drive Command", f"[bold green]{self.last_wasd_cmd}[/]")
        nav_table.add_row("Last Serial Packet", f"[bold green]{self.serial_mgr.last_sent_packet}[/]" if self.serial_mgr.last_sent_packet else "[dim]NONE[/]")
        tx_status_str = f"[bold red]ERR: {self.serial_mgr.last_tx_error}[/]" if self.serial_mgr.tx_error_count > 0 else f"[bold green]OK ({self.serial_mgr.tx_packet_count} pkts)[/]"
        nav_table.add_row("Serial TX Status", tx_status_str)
        nav_table.add_row("Safety Watchdog", wdg_display)
        nav_table.add_row("Vision Bridge", "[green]READY[/]" if HAVE_VISION else "[dim]DISABLED[/]")
        nav_table.add_row("A* Grid Planner", "[green]READY[/]" if HAVE_ASTAR else "[dim]DISABLED[/]")
        layout["tables"]["right_col"].update(nav_table)

        # Footer Shortcuts
        footer_text = Text.from_markup(
            "[bold white]Shortcuts:[/] "
            "[bold green][T/1][/] Teleop  "
            "[bold cyan][F/2][/] Follow  "
            "[bold magenta][N/3][/] Nav  "
            "[bold yellow][I/0][/] Idle  "
            "[bold magenta][C][/] Cruise/Hold  "
            "[bold red][Space][/] Brake  "
            "[bold blue][+/-][/] Speed  "
            "[bold red][Q][/] Quit"
        )
        layout["footer"].update(Panel(footer_text, style="blue"))

        return layout


# ==============================================================================
# Main Runner
# ==============================================================================

def main():
    parser = argparse.ArgumentParser(description="CNL2 Master State Machine Controller")
    parser.add_argument("--port", type=str, default=None, help="Serial device port (e.g. /dev/ttyACM0 or /dev/serial0)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default 115200)")
    parser.add_argument("--speed", type=int, default=200, help="Default motor speed PWM (50 - 255)")
    parser.add_argument("--ssh", action="store_true", help="Force SSH-optimized low-bandwidth terminal mode")
    args = parser.parse_args()

    master = MasterStateMachine(port=args.port, baudrate=args.baud, default_speed=args.speed)
    if args.ssh:
        master.is_ssh = True
        master.key_decay_s = 0.85

    # 1. Connect Serial Link to ESP32
    console.print("[cyan]Connecting to ESP32-S3 microcontroller...[/]")
    if not master.serial_mgr.connect():
        console.print("[yellow]Notice: Serial port not found immediately. Running background auto-reconnect...[/]")

    # 2. Initialize Vision Subsystem (if camera is present)
    master.initialize_vision()

    # 3. Start Keyboard Listener
    master.start_input()

    # Refresh rate: 10 Hz over SSH (smooth, zero terminal flicker, low bandwidth) or 20 Hz locally
    refresh_rate = 10 if master.is_ssh else 20

    # 4. Master State Machine Execution Loop with Live Dashboard
    try:
        with Live(master.render_dashboard(), console=console, screen=True, refresh_per_second=refresh_rate) as live:
            while master.running:
                master.tick()
                live.update(master.render_dashboard())
                time.sleep(1.0 / refresh_rate)
    except KeyboardInterrupt:
        pass
    finally:
        console.print("[bold red]Stopping robot and releasing serial link...[/]")
        master.serial_mgr.set_drive_command("B", 0)
        time.sleep(0.1)
        master.serial_mgr.set_drive_command("X", 0)
        time.sleep(0.1)
        master.serial_mgr.close()
        if master.vision_bridge:
            master.vision_bridge.stop()
        # Restore cursor visibility in terminal
        sys.stdout.write("\033[?25h\n")
        sys.stdout.flush()
        console.print("[bold green]Master State Machine terminated cleanly.[/]")
        master.serial_mgr.close()
        if master.vision_bridge:
            master.vision_bridge.stop()
        console.print("[bold green]Master State Machine terminated cleanly.[/]")


if __name__ == "__main__":
    main()
