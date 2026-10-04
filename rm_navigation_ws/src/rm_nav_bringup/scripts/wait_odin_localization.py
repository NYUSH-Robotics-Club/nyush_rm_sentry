#!/usr/bin/env python3
"""Exit successfully when Odin's relocalization map/odom transform appears."""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def main():
    rclpy.init()
    node = Node('wait_odin_localization')
    node.declare_parameter('timeout_sec', 0.0)
    timeout = float(node.get_parameter('timeout_sec').value)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    del listener
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
            if deadline is not None and time.monotonic() >= deadline:
                node.get_logger().error('Timed out waiting for Odin map/odom transform')
                return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 1


if __name__ == '__main__':
    sys.exit(main())
