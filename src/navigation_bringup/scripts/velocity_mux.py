#!/usr/bin/env python3
"""Joystick/keyboard priority over Nav2; one physical TwistStamped output."""
import math
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.executors import ExternalShutdownException


def finite(command):
    return all(math.isfinite(v) for vector in (command.linear, command.angular)
               for v in (vector.x, vector.y, vector.z))


class Arbitration:
    """Use monotonic receipt times so a paused simulation cannot latch a command."""
    def __init__(self, mode, timeout, button):
        self.mode, self.timeout, self.button = mode, timeout, button
        self.manual = self.navigation = None
        self.manual_time = self.nav_time = self.joy_time = float('-inf')
        self.keyboard_time = self.manual_since = float('-inf')
        self.held = False

    def joy(self, buttons, now):
        held = 0 <= self.button < len(buttons) and bool(buttons[self.button])
        if held and not self.held:
            self.manual_since = now
        self.held, self.joy_time = held, now

    def receive_manual(self, command, now):
        if not finite(command):
            self.manual = None
            return
        self.manual, self.manual_time = command, now
        if any(abs(v) > 1e-6 for v in (command.linear.x, command.angular.z)):
            self.keyboard_time = now

    def receive_navigation(self, command, now):
        self.navigation = command if finite(command) else None
        self.nav_time = now

    def select(self, now):
        manual_active = (
            self.mode == 'joystick' and self.held and 0 <= now - self.joy_time <= self.timeout
        ) or (self.mode == 'keyboard' and 0 <= now - self.keyboard_time <= self.timeout)
        if manual_active:
            fresh = self.manual is not None and 0 <= now - self.manual_time <= self.timeout
            if self.mode == 'joystick':
                fresh = fresh and self.manual_time >= self.manual_since
            return 'manual', self.manual if fresh else Twist()
        if self.navigation is not None and 0 <= now - self.nav_time <= self.timeout:
            return 'navigation', self.navigation
        return 'stop', Twist()


class VelocityMux(Node):
    def __init__(self):
        super().__init__('velocity_mux')
        mode = self.declare_parameter('manual_mode', 'joystick').value
        timeout = self.declare_parameter('timeout', 0.3).value
        button = self.declare_parameter('enable_button', 9).value
        rate = self.declare_parameter('publish_hz', 50.0).value
        if mode not in ('joystick', 'keyboard', 'none'):
            raise ValueError('manual_mode must be joystick, keyboard or none')
        if not all(math.isfinite(v) and v > 0 for v in (timeout, rate)):
            raise ValueError('timeout and publish_hz must be finite and positive')
        if button < 0:
            raise ValueError('enable_button must be nonnegative')
        self.policy = Arbitration(mode, timeout, button)
        self.max_age = timeout
        self.source = None
        output_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        input_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.output = self.create_publisher(TwistStamped, 'output', output_qos)
        self.status = self.create_publisher(String, 'selected_source', output_qos)
        self.create_subscription(Twist, 'navigation', self.navigation, input_qos)
        self.create_subscription(TwistStamped, 'manual', self.manual, input_qos)
        self.create_subscription(Joy, 'joy', self.joy, input_qos)
        self.create_timer(1.0 / rate, self.publish, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f'Command arbiter ready: manual={mode}, priority=manual > navigation, timeout={timeout}s')

    def navigation(self, message):
        self.policy.receive_navigation(message, time.monotonic())

    def manual(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if 0 <= age <= self.max_age:
            self.policy.receive_manual(message.twist, time.monotonic())

    def joy(self, message):
        self.policy.joy(message.buttons, time.monotonic())

    def publish(self):
        source, command = self.policy.select(time.monotonic())
        if source != self.source:
            self.get_logger().info(f'Command source: {source}')
            self.source = source
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.twist = command
        self.output.publish(message)
        self.status.publish(String(data=source))


def main(args=None):
    rclpy.init(args=args)
    node = VelocityMux()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
