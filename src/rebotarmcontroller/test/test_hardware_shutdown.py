"""Shutdown regression checks using simulated motor and transport boundaries."""

import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from rebotarmcontroller.hardware_manager import HardwareManager


class HardwareShutdownTests(unittest.TestCase):
    def make_hardware(self, *, enabled):
        hardware = HardwareManager.__new__(HardwareManager)
        hardware._cmd_lock = threading.RLock()
        hardware._connected = True
        hardware._enabled = enabled
        hardware._control_output_enabled = enabled
        hardware._gravity_comp_active = False
        hardware._state_machine = "IDLE"
        hardware._homing_thread = None
        events = []
        hardware._robot = Mock(control_loop_active=enabled, has_gripper=False)

        def stop_loop():
            events.append("stop")
            hardware._robot.control_loop_active = False

        hardware._robot.stop_control_loop.side_effect = stop_loop
        hardware._robot.disable_all.side_effect = lambda: events.append("disable")
        hardware._robot.disconnect.side_effect = lambda: events.append("disconnect")
        hardware._endpos_ctrl = SimpleNamespace(
            _running=enabled,
            safe_home=Mock(side_effect=lambda: events.append("home")),
        )
        hardware.stop_motion = Mock()

        def start_loop():
            events.append("enable")
            hardware._robot.control_loop_active = True

        hardware._start_endpos_loop = Mock(side_effect=start_loop)
        return hardware, events

    def assert_closed(self, hardware):
        self.assertFalse(hardware._connected)
        self.assertFalse(hardware.enabled)
        self.assertFalse(hardware._control_output_enabled)
        self.assertEqual(hardware.state_machine, "IDLE")

    def test_disabled_shutdown_never_enables_or_homes(self):
        for disable_after_safe_home in (True, False):
            with self.subTest(disable_after_safe_home=disable_after_safe_home):
                hardware, events = self.make_hardware(enabled=False)
                hardware.shutdown(disable_after_safe_home=disable_after_safe_home)
                self.assertNotIn("enable", events)
                self.assertNotIn("home", events)
                self.assertIn("disconnect", events)
                self.assert_closed(hardware)

    def test_enabled_shutdown_homes_before_disabling(self):
        hardware, events = self.make_hardware(enabled=True)
        hardware.shutdown()
        self.assertLess(events.index("home"), events.index("disable"))
        self.assertLess(events.index("disable"), events.index("disconnect"))
        self.assert_closed(hardware)

    def test_shutdown_after_demo_cleanup_does_not_restart_motors(self):
        hardware, events = self.make_hardware(enabled=True)
        # GravityCompensation finishes with these two service operations.
        hardware.safe_home()
        hardware.disable()
        events.clear()
        hardware.shutdown()
        self.assertNotIn("enable", events)
        self.assertNotIn("home", events)
        self.assert_closed(hardware)

    def test_shutdown_preserves_disable_after_safe_home_option(self):
        hardware, events = self.make_hardware(enabled=True)
        hardware.shutdown(disable_after_safe_home=False)
        self.assertIn("home", events)
        hardware._robot.disable_all.assert_not_called()
        hardware._robot.disconnect.assert_called_once_with()
        self.assert_closed(hardware)

    def test_disconnected_shutdown_sends_no_commands(self):
        hardware, events = self.make_hardware(enabled=False)
        hardware._connected = False
        hardware.shutdown()
        self.assertEqual(events, [])

    def test_repeated_shutdown_sends_no_additional_commands(self):
        hardware, events = self.make_hardware(enabled=True)
        hardware.shutdown()
        first_shutdown = events.copy()
        hardware.shutdown()
        self.assertEqual(events, first_shutdown)


if __name__ == "__main__":
    unittest.main()
