#!/usr/bin/env python3
"""Record a simple 2-D Nav2 occupancy map alongside Odin's native .bin map."""

import math
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


def rotate(points, q):
    x, y, z, w = q.x, q.y, q.z, q.w
    matrix = np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])
    return points @ matrix.T


def ray_cells(start, end):
    """Grid cells from sensor to hit, excluding the hit cell."""
    x0, y0 = start
    x1, y1 = end
    count = max(abs(x1 - x0), abs(y1 - y0))
    if count == 0:
        return []
    return [(round(x0 + (x1-x0)*i/count), round(y0 + (y1-y0)*i/count))
            for i in range(count)]


class OdinGridMap(Node):
    def __init__(self):
        super().__init__('odin_grid_map')
        self.declare_parameter('map_prefix', '/tmp/odin_map')
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('sensor_height', 0.45)
        self.prefix = Path(self.get_parameter('map_prefix').value)
        self.resolution = float(self.get_parameter('resolution').value)
        self.sensor_height = float(self.get_parameter('sensor_height').value)
        if self.resolution <= 0 or self.sensor_height <= 0:
            raise ValueError('resolution and sensor_height must be positive')
        self.cells = {}  # (ix, iy): 0=free, 100=occupied
        self.scans = 0
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(PointCloud2, '/odin1/cloud_raw', self.on_cloud, 10)
        self.grid_pub = self.create_publisher(
            OccupancyGrid, '/odin_dashboard/live_map',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.create_timer(2.0, self.publish_map)
        self.create_service(Trigger, '/odin_grid_map/ready', self.ready)
        self.create_service(Trigger, '/odin_grid_map/save', self.save)
        self.get_logger().info('Recording Nav2 grid from Odin raw cloud in odom frame')

    def on_cloud(self, msg):
        # Odin mapping starts with coincident map and odom origins. The native
        # .bin remains the source of truth for device relocalization.
        try:
            tf = self.buffer.lookup_transform('odom', msg.header.frame_id, Time())
        except TransformException:
            return
        try:
            # Odin cloud_raw also contains uint8/uint16 fields. Humble's
            # read_points_numpy rejects mixed-type clouds even when only
            # x/y/z are requested; read the structured fields explicitly.
            raw = point_cloud2.read_points(
                msg, field_names=('x', 'y', 'z'), skip_nans=True)
            points = np.column_stack((raw['x'], raw['y'], raw['z']))
        except (AssertionError, KeyError, ValueError) as exc:
            self.get_logger().error(
                f'Cannot decode Odin raw cloud: {exc}', throttle_duration_sec=5.0)
            return
        if len(points) == 0:
            return
        points = np.asarray(points, dtype=np.float64)[::max(1, len(points)//1200)]
        tr = tf.transform.translation
        origin = np.array([tr.x, tr.y, tr.z])
        world = rotate(points, tf.transform.rotation) + origin
        start = tuple(np.floor(origin[:2] / self.resolution).astype(int))
        floor_z = origin[2] - self.sensor_height
        for x, y, z in world:
            distance = math.hypot(x-origin[0], y-origin[1])
            if not (0.15 <= distance <= 8.0 and floor_z-0.1 <= z <= floor_z+1.8):
                continue
            end = (math.floor(x/self.resolution), math.floor(y/self.resolution))
            if abs(end[0]-start[0]) > 160 or abs(end[1]-start[1]) > 160:
                continue
            for cell in ray_cells(start, end):
                self.cells.setdefault(cell, 0)
            if z >= floor_z+0.12:
                self.cells[end] = 100
            else:
                self.cells.setdefault(end, 0)
        self.scans += 1

    def publish_map(self):
        if self.scans == 0 or not self.cells:
            return
        xs, ys = zip(*self.cells)
        min_x, max_x = min(xs)-2, max(xs)+2
        min_y, max_y = min(ys)-2, max(ys)+2
        width, height = max_x-min_x+1, max_y-min_y+1
        if width*height > 30_000_000:
            self.get_logger().error('Live occupancy grid exceeds 30 million cells')
            return
        grid = np.full((height, width), -1, dtype=np.int8)
        for (x, y), value in self.cells.items():
            grid[y-min_y, x-min_x] = value
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.info.resolution = self.resolution
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = min_x*self.resolution
        msg.info.origin.position.y = min_y*self.resolution
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.ravel().tolist()
        self.grid_pub.publish(msg)

    def ready(self, request, response):
        del request
        if self.scans < 10 or len(self.cells) < 100:
            response.message = 'Not enough point clouds/TF for an occupancy map'
            return response
        response.success = True
        response.message = f'Grid ready ({self.scans} scans, {len(self.cells)} cells)'
        return response

    def save(self, request, response):
        del request
        if self.scans < 10 or len(self.cells) < 100:
            response.message = 'Not enough point clouds/TF for an occupancy map'
            return response
        xs, ys = zip(*self.cells)
        min_x, max_x = min(xs)-2, max(xs)+2
        min_y, max_y = min(ys)-2, max(ys)+2
        width, height = max_x-min_x+1, max_y-min_y+1
        if width*height > 30_000_000:
            response.message = 'Map exceeds 30 million cells; check odometry and resolution'
            return response
        image = np.full((height, width), 205, dtype=np.uint8)
        for (x, y), value in self.cells.items():
            image[max_y-y, x-min_x] = 0 if value == 100 else 254
        self.prefix.parent.mkdir(parents=True, exist_ok=True)
        pgm = self.prefix.with_suffix('.pgm')
        yaml_path = self.prefix.with_suffix('.yaml')
        pgm.write_bytes(f'P5\n{width} {height}\n255\n'.encode() + image.tobytes())
        yaml_path.write_text(
            f'image: {pgm.name}\nresolution: {self.resolution}\n'
            f'origin: [{min_x*self.resolution}, {min_y*self.resolution}, 0.0]\n'
            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n',
            encoding='utf-8')
        response.success = True
        response.message = f'Saved {pgm} and {yaml_path} ({self.scans} scans)'
        self.get_logger().info(response.message)
        return response


def main():
    rclpy.init()
    node = OdinGridMap()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
