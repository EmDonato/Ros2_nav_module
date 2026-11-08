"""Priority, deadman and timeout contracts for command arbitration."""
import importlib.util
from pathlib import Path
import unittest
from geometry_msgs.msg import Twist

spec = importlib.util.spec_from_file_location('velocity_mux', Path(__file__).resolve().parents[1] / 'scripts/velocity_mux.py')
mux = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mux)


def velocity(value):
    message = Twist()
    message.linear.x = value
    return message


class ArbitrationTests(unittest.TestCase):
    def setUp(self):
        self.policy = mux.Arbitration('joystick', 0.3, 9)
        self.buttons = [0] * 10

    def test_navigation_wins_without_joystick(self):
        self.policy.receive_navigation(velocity(0.1), 1.0)
        source, command = self.policy.select(1.1)
        self.assertEqual((source, command.linear.x), ('navigation', 0.1))

    def test_held_joystick_has_priority_even_at_zero(self):
        self.policy.receive_navigation(velocity(0.1), 1.0)
        self.buttons[9] = 1
        self.policy.joy(self.buttons, 1.0)
        self.policy.receive_manual(velocity(0.0), 1.01)
        source, command = self.policy.select(1.1)
        self.assertEqual((source, command.linear.x), ('manual', 0.0))

    def test_release_returns_to_navigation(self):
        self.buttons[9] = 1
        self.policy.joy(self.buttons, 1.0)
        self.policy.receive_manual(velocity(-0.2), 1.01)
        self.policy.receive_navigation(velocity(0.1), 1.05)
        self.assertEqual(self.policy.select(1.1)[0], 'manual')
        self.buttons[9] = 0
        self.policy.joy(self.buttons, 1.11)
        self.assertEqual(self.policy.select(1.12)[0], 'navigation')

    def test_lost_joystick_returns_to_fresh_navigation(self):
        self.buttons[9] = 1
        self.policy.joy(self.buttons, 1.0)
        self.policy.receive_navigation(velocity(0.1), 1.4)
        self.assertEqual(self.policy.select(1.5)[0], 'navigation')

    def test_stale_sources_stop(self):
        self.policy.receive_navigation(velocity(0.1), 1.0)
        source, command = self.policy.select(2.0)
        self.assertEqual((source, command.linear.x), ('stop', 0.0))

    def test_held_button_with_lost_manual_commands_blocks_navigation(self):
        self.buttons[9] = 1
        self.policy.joy(self.buttons, 1.0)
        self.policy.receive_manual(velocity(0.2), 1.01)
        self.policy.joy(self.buttons, 1.5)
        self.policy.receive_navigation(velocity(0.1), 1.5)
        source, command = self.policy.select(1.55)
        self.assertEqual((source, command.linear.x), ('manual', 0.0))

    def test_idle_keyboard_zeroes_do_not_block_navigation(self):
        self.policy.mode = 'keyboard'
        self.policy.receive_manual(velocity(0.2), 1.0)
        self.policy.receive_manual(velocity(0.0), 1.4)
        self.policy.receive_navigation(velocity(0.1), 1.4)
        self.assertEqual(self.policy.select(1.5)[0], 'navigation')

    def test_nan_navigation_stops(self):
        self.policy.receive_navigation(velocity(float('nan')), 1.0)
        self.assertEqual(self.policy.select(1.1)[0], 'stop')

    def test_old_manual_command_not_reused_on_reengagement(self):
        self.policy.receive_manual(velocity(0.2), 1.0)
        self.buttons[9] = 1
        self.policy.joy(self.buttons, 1.1)
        self.assertEqual(self.policy.select(1.11)[1].linear.x, 0.0)
