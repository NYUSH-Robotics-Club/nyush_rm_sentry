#!/usr/bin/env python3
"""Send Nav2 body velocities to the existing sentry bridge radar PTY.

The radar frame carries normalized chassis requests. The C-board omni
kinematics converts them back to m/s and rad/s using the same configured limits.
Only the bridge opens the physical USB CDC device.
"""

import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from odin_chassis_protocol import normalized_velocity, odin_autonomy_frame, radar_frame


class OdinChassisSender(Node):
    def __init__(self):
        super().__init__('odin_chassis_sender')
        self.declare_parameter('input_topic', '/cmd_vel_chassis')
        self.declare_parameter('radar_pty', '/tmp/nyush-rm-sentry-radar')
        self.declare_parameter('max_translation_mps', 0.90)
        self.declare_parameter('max_rotation_radps', 1.20)
        self.declare_parameter('command_timeout_sec', 0.20)
        self.declare_parameter('rate_hz', 20.0)
        self.input_topic = str(self.get_parameter('input_topic').value)
        self.radar_pty = str(self.get_parameter('radar_pty').value)
        self.max_translation = float(self.get_parameter('max_translation_mps').value)
        self.max_rotation = float(self.get_parameter('max_rotation_radps').value)
        self.timeout = float(self.get_parameter('command_timeout_sec').value)
        rate = float(self.get_parameter('rate_hz').value)
        if not all(math.isfinite(value) for value in
                   (self.max_translation, self.max_rotation, self.timeout, rate)):
            raise ValueError('speed, timeout and rate parameters must be finite')
        if self.max_translation <= 0 or self.max_rotation <= 0 or not 0 < self.timeout < 0.25:
            raise ValueError('speed limits must be positive and command timeout must be under 0.25 s')
        if rate < 10 or rate > 100:
            raise ValueError('rate_hz must be between 10 and 100')

        self.last_cmd = (0.0, 0.0, 0.0)
        self.last_cmd_at = None
        self.fd = None
        self.last_error_at = 0.0
        self.last_status_at = 0.0
        self.status_pub = self.create_publisher(String, '/odin_dashboard/chassis_link', 10)
        self.create_subscription(Twist, self.input_topic, self.on_velocity, 10)
        self.create_timer(1.0 / rate, self.send_current)
        self.get_logger().info('Chassis sender: %s -> %s' % (self.input_topic, self.radar_pty))

    def on_velocity(self, message):
        try:
            self.last_cmd = normalized_velocity(
                message.linear.x, message.linear.y, message.angular.z,
                self.max_translation, self.max_rotation)
            self.last_cmd_at = time.monotonic()
        except ValueError as exc:
            self.last_cmd_at = None
            self.get_logger().error(str(exc))

    def ensure_port(self):
        try:
            if self.fd is not None and os.stat(self.radar_pty).st_ino == os.fstat(self.fd).st_ino:
                return True
        except OSError:
            pass
        self.close_port()
        try:
            self.fd = os.open(self.radar_pty, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            return True
        except OSError as exc:
            now = time.monotonic()
            if now - self.last_error_at >= 5.0:
                self.get_logger().warning('Radar PTY unavailable: %s' % exc)
                self.last_error_at = now
            return False

    def close_port(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def send_current(self):
        now = time.monotonic()
        fresh = self.last_cmd_at is not None and now - self.last_cmd_at <= self.timeout
        command = self.last_cmd if fresh else (0.0, 0.0, 0.0)
        connected = self.ensure_port()
        if connected:
            try:
                if os.write(self.fd, odin_autonomy_frame()) != 16 or \
                        os.write(self.fd, radar_frame(*command)) != 19:
                    raise OSError('short radar control frame write')
            except OSError as exc:
                self.close_port()
                connected = False
                if now - self.last_error_at >= 5.0:
                    self.get_logger().warning('Radar PTY write failed: %s' % exc)
                    self.last_error_at = now
        if now - self.last_status_at >= 0.5:
            status = String()
            status.data = json.dumps({
                'connected': connected, 'nav_command_fresh': fresh,
                'normalized_vx': command[0], 'normalized_vy': command[1],
                'normalized_wz': command[2], 'radar_pty': self.radar_pty,
            })
            self.status_pub.publish(status)
            self.last_status_at = now

    def shutdown(self):
        if self.fd is not None:
            try:
                os.write(self.fd, radar_frame(0.0, 0.0, 0.0))
                os.write(self.fd, odin_autonomy_frame(False))
            except OSError:
                pass
        self.close_port()


def main():
    rclpy.init()
    node = OdinChassisSender()
    try:
        rclpy.spin(node)
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
