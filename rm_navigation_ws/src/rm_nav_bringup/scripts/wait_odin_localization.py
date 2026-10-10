#!/usr/bin/env python3
"""Wait for Odin relocalization while turning its gimbal continuously."""

import math
import os
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from odin_chassis_protocol import gimbal_scan_frame


def yaw_from_quaternion(q):
    values = (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(value) for value in values) or sum(value*value for value in values) < 0.5:
        return None
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))


class GimbalScan:
    def __init__(self, node):
        node.declare_parameter('scan_enabled', True)
        node.declare_parameter('scan_rate_deg_s', 60.0)
        node.declare_parameter('scan_max_duration_sec', 0.0)
        node.declare_parameter('radar_pty', '/tmp/nyush-rm-sentry-radar')
        self.node = node
        self.enabled = bool(node.get_parameter('scan_enabled').value)
        self.rate = float(node.get_parameter('scan_rate_deg_s').value)
        self.max_duration = float(node.get_parameter('scan_max_duration_sec').value)
        self.path = str(node.get_parameter('radar_pty').value)
        if not all(math.isfinite(value) for value in (self.rate, self.max_duration)) or \
                not 0 < self.rate <= 60 or self.max_duration < 0:
            raise ValueError('scan requires rate <=60 deg/s and nonnegative duration')
        self.fd = None
        self.last_error = 0.0
        self.last_odom = 0.0
        self.yaw = None
        self.last_motion_yaw = None
        self.last_motion_at = None
        self.started_at = None
        self.finished = False
        node.create_subscription(Odometry, '/odin1/odometry', self.on_odom,
                                 qos_profile_sensor_data)

    def on_odom(self, msg):
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        if yaw is not None:
            self.yaw = yaw
            self.last_odom = time.monotonic()

    def _open(self):
        try:
            if self.fd is not None and os.fstat(self.fd).st_ino == os.stat(self.path).st_ino:
                return True
        except OSError:
            pass
        self.close()
        try:
            self.fd = os.open(self.path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            return True
        except OSError as exc:
            now = time.monotonic()
            if now - self.last_error >= 5:
                self.node.get_logger().warning(f'Gimbal scan unavailable ({self.path}): {exc}')
                self.last_error = now
            return False

    def send(self, rate):
        if not self._open():
            return False
        try:
            if os.write(self.fd, gimbal_scan_frame(rate)) != 16:
                raise OSError('short robot-control frame write')
            return True
        except OSError as exc:
            self.node.get_logger().warning(f'Gimbal scan write failed: {exc}')
            self.close()
            return False

    def tick(self):
        if not self.enabled or self.finished or self.yaw is None:
            return
        now = time.monotonic()
        if now - self.last_odom > 1.0:
            if self.started_at is not None:
                self.node.get_logger().warning('Odin odometry stopped; automatic gimbal scan disabled')
                self.finished = True
                self.stop()
            return
        if self.started_at is not None and self.max_duration > 0 and \
                now - self.started_at >= self.max_duration:
            self.node.get_logger().warning('Gimbal scan time limit reached; move the whole robot to relocalize')
            self.finished = True
            self.stop()
            return
        if self.last_motion_yaw is not None:
            moved = math.atan2(math.sin(self.yaw-self.last_motion_yaw),
                               math.cos(self.yaw-self.last_motion_yaw))
            if abs(moved) >= math.radians(2):
                self.last_motion_yaw = self.yaw
                self.last_motion_at = now
            elif now - self.last_motion_at > 8.0:
                self.node.get_logger().warning('No measured gimbal turn; stopping automatic scan')
                self.finished = True
                self.stop()
                return
        if self.send(self.rate):
            if self.started_at is None:
                self.started_at = now
                self.last_motion_yaw = self.yaw
                self.last_motion_at = now
                self.node.get_logger().info(
                    f'Continuous gimbal rotation started: {self.rate:g} deg/s')

    def stop(self):
        if self.fd is not None:
            try:
                os.write(self.fd, gimbal_scan_frame(0.0))
            except OSError:
                pass
        self.close()
        self.started_at = None
        self.last_motion_yaw = None
        self.last_motion_at = None

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


def main():
    rclpy.init()
    node = Node('wait_odin_localization')
    node.declare_parameter('timeout_sec', 0.0)
    timeout = float(node.get_parameter('timeout_sec').value)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    del listener
    scan = GimbalScan(node)
    deadline = time.monotonic() + timeout if timeout > 0 else None
    try:
        node.get_logger().info('Move the whole robot through the mapped scene; waiting for Odin relocalization')
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            try:
                transform = buffer.lookup_transform('map', 'odom', Time())
                node.get_logger().info(
                    'Odin relocalized: map<-odom translation '
                    f'({transform.transform.translation.x:.3f}, '
                    f'{transform.transform.translation.y:.3f}, '
                    f'{transform.transform.translation.z:.3f}) m')
                return 0
            except TransformException:
                pass
            scan.tick()
            if deadline is not None and time.monotonic() >= deadline:
                node.get_logger().error('Timed out waiting for Odin map/odom transform')
                return 1
    finally:
        scan.stop()
        node.destroy_node()
        rclpy.shutdown()
    return 1


if __name__ == '__main__':
    sys.exit(main())
