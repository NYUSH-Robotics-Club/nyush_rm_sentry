import importlib.util
import math
from pathlib import Path
import unittest

from nav_msgs.msg import Odometry


script = Path(__file__).resolve().parents[1] / 'scripts' / 'odin_nav_odometry.py'
spec = importlib.util.spec_from_file_location('odin_nav_odometry', script)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class OdinNavOdometryTest(unittest.TestCase):
    mount = (-0.15, 0.0, -0.39, 0.0, 0.0, math.pi)

    def test_rear_facing_mount_rotates_world_velocity_to_body(self):
        source = Odometry()
        source.header.frame_id = 'odom'
        source.child_frame_id = 'imu'
        source.pose.pose.orientation.w = 1.0
        source.twist.twist.linear.x = -0.2
        source.twist.twist.linear.y = 0.1

        output = module.adapt_odom(source, self.mount)

        self.assertEqual(output.child_frame_id, 'base_link')
        self.assertAlmostEqual(output.pose.pose.position.x, -0.15)
        self.assertAlmostEqual(output.pose.pose.position.z, -0.39)
        self.assertAlmostEqual(output.twist.twist.linear.x, 0.2)
        self.assertAlmostEqual(output.twist.twist.linear.y, -0.1)

    def test_mount_offset_changes_velocity_during_rotation(self):
        source = Odometry()
        source.pose.pose.orientation.z = math.sqrt(0.5)
        source.pose.pose.orientation.w = math.sqrt(0.5)
        source.twist.twist.linear.y = 1.0
        source.twist.twist.angular.z = 1.0

        output = module.adapt_odom(source, self.mount)

        self.assertAlmostEqual(output.pose.pose.position.x, 0.0, places=6)
        self.assertAlmostEqual(output.pose.pose.position.y, -0.15, places=6)
        self.assertAlmostEqual(output.twist.twist.linear.x, -1.0, places=6)
        self.assertAlmostEqual(output.twist.twist.linear.y, 0.15, places=6)
        self.assertAlmostEqual(output.twist.twist.angular.z, 1.0, places=6)

    def test_invalid_orientation_is_rejected(self):
        source = Odometry()
        source.pose.pose.orientation.w = 0.0
        with self.assertRaises(ValueError):
            module.adapt_odom(source, self.mount)


if __name__ == '__main__':
    unittest.main()
