#!/usr/bin/env python3
import sys
import time
import unittest
from pathlib import Path

# Add project root and subdirectories to sys.path
ROOT_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT_DIR / "rasberrypi_host"))
sys.path.insert(0, str(ROOT_DIR / "teleop_host"))

from master_controller import resolve_wasd_command, MasterStateMachine
from serial_interface import SerialManager


class TestHostTimingAndKeys(unittest.TestCase):

    def test_wasd_command_resolution(self):
        # Single directions
        self.assertEqual(resolve_wasd_command({"w"}), "W")
        self.assertEqual(resolve_wasd_command({"s"}), "S")
        self.assertEqual(resolve_wasd_command({"a"}), "A")
        self.assertEqual(resolve_wasd_command({"d"}), "D")

        # Compound arc turns
        self.assertEqual(resolve_wasd_command({"w", "a"}), "WA")
        self.assertEqual(resolve_wasd_command({"w", "d"}), "WD")
        self.assertEqual(resolve_wasd_command({"s", "a"}), "SA")
        self.assertEqual(resolve_wasd_command({"s", "d"}), "SD")

        # Priority Active Brake on Space Bar
        self.assertEqual(resolve_wasd_command({" "}), "B")
        self.assertEqual(resolve_wasd_command({"w", " "}), "B")
        self.assertEqual(resolve_wasd_command({"w", "a", " "}), "B")

        # Empty or conflict
        self.assertEqual(resolve_wasd_command(set()), "X")
        self.assertEqual(resolve_wasd_command({"w", "s"}), "X")

    def test_serial_manager_command_deduplication(self):
        mgr = SerialManager(port="DUMMY", baudrate=115200)

        # First command change should set _force_send
        mgr._force_send.clear()
        mgr.set_drive_command("W", 200)
        self.assertTrue(mgr._force_send.is_set())

        # Clear event
        mgr._force_send.clear()

        # 30 repeated identical commands (representing 20 Hz key hold or loop ticks)
        for _ in range(30):
            mgr.set_drive_command("W", 200)

        # _force_send must NOT have been set again!
        self.assertFalse(mgr._force_send.is_set())

        # Change command to Brake
        mgr.set_drive_command("B", 0)
        self.assertTrue(mgr._force_send.is_set())

    def test_latched_mode_tap_to_move_space_to_brake(self):
        master = MasterStateMachine()
        self.assertTrue(master.cruise_mode)  # Latched mode active by default

        # Tap 'w' -> car moves forward
        master._handle_char_press("w")
        self.assertIn("w", master.active_keys)
        self.assertEqual(resolve_wasd_command(master.active_keys), "W")

        # Key release in latched mode must NOT stop the robot
        class DummyKey:
            char = "w"
        master._on_pynput_release(DummyKey())
        self.assertIn("w", master.active_keys)
        self.assertEqual(resolve_wasd_command(master.active_keys), "W")

        # Tap 'a' while moving forward -> compound arc forward-left
        master._handle_char_press("a")
        self.assertEqual(resolve_wasd_command(master.active_keys), "WA")

        # Hit Space Bar -> Emergency Brake halts all motion
        master._handle_char_press(" ")
        self.assertEqual(resolve_wasd_command(master.active_keys), "B")


if __name__ == "__main__":
    unittest.main()

