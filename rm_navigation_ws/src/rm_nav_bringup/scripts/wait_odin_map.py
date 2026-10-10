#!/usr/bin/env python3
"""Wait for Nav2's latched occupancy map before starting navigation nodes."""

import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def main():
    rclpy.init()
    node = Node('wait_odin_map')
    node.declare_parameter('timeout_sec', 45.0)
    timeout = float(node.get_parameter('timeout_sec').value)
    deadline = time.monotonic() + timeout
    received = False

    def on_map(message):
        nonlocal received
        if message.info.width > 0 and message.info.height > 0 and message.data:
            received = True

    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(OccupancyGrid, '/map', on_map, qos)
    try:
        node.get_logger().info('Waiting for active Nav2 map server to publish /map')
        while rclpy.ok() and not received:
            rclpy.spin_once(node, timeout_sec=0.2)
            if time.monotonic() >= deadline:
                node.get_logger().error('Timed out waiting for /map; check map_server lifecycle and map file')
                return 1
        if received:
            node.get_logger().info('Nav2 occupancy map received')
            return 0
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
