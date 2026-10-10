#!/usr/bin/env python3
"""Publish a small, rate-limited Odin camera stream for remote dashboards."""

from io import BytesIO
import time

from PIL import Image, UnidentifiedImageError
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


CAMERA_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


def make_preview(source, max_width, quality):
    with Image.open(BytesIO(source.data)) as original:
        image = original.convert('RGB')
    image.thumbnail((max_width, max_width), Image.BILINEAR)
    output = BytesIO()
    image.save(output, format='JPEG', quality=quality)
    preview = CompressedImage()
    preview.header = source.header
    preview.format = 'jpeg'
    preview.data = output.getvalue()
    return preview


class OdinCameraPreview(Node):
    def __init__(self):
        super().__init__('odin_camera_preview')
        self.declare_parameter('max_width', 640)
        self.declare_parameter('quality', 65)
        self.declare_parameter('rate_hz', 5.0)
        self.max_width = int(self.get_parameter('max_width').value)
        self.quality = int(self.get_parameter('quality').value)
        rate_hz = float(self.get_parameter('rate_hz').value)
        if self.max_width < 64 or not 1 <= self.quality <= 95 or not 0 < rate_hz <= 30:
            raise ValueError('invalid camera preview width, JPEG quality, or frame rate')
        self.latest = None
        self.latest_at = 0.0
        self.last_error_at = 0.0
        self.publisher = self.create_publisher(
            CompressedImage, '/odin_dashboard/camera_preview', CAMERA_QOS)
        self.create_subscription(
            CompressedImage, '/odin1/image/compressed', self.on_image, CAMERA_QOS)
        self.create_timer(1.0 / rate_hz, self.publish_preview)

    def on_image(self, message):
        self.latest = message
        self.latest_at = time.monotonic()

    def publish_preview(self):
        source = self.latest
        self.latest = None
        if source is None or time.monotonic() - self.latest_at > 0.5:
            return
        try:
            self.publisher.publish(make_preview(source, self.max_width, self.quality))
        except (OSError, ValueError, UnidentifiedImageError) as exc:
            now = time.monotonic()
            if now - self.last_error_at >= 5.0:
                self.get_logger().warning(f'Could not create Odin camera preview: {exc}')
                self.last_error_at = now


def main():
    rclpy.init()
    node = OdinCameraPreview()
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
