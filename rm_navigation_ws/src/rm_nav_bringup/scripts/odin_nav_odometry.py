#!/usr/bin/env python3
"""Adapt Odin velocity to Nav2's gimbal-forward virtual-centre frame."""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def rotate(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


def adapt_odom(source, mount):
    """Odin reports linear velocity in odom, despite child_frame_id=imu."""
    output = Odometry()
    output.header = source.header
    output.child_frame_id = 'base_link'

    orientation = source.pose.pose.orientation
    q_imu = (orientation.x, orientation.y, orientation.z, orientation.w)
    length = math.sqrt(sum(value * value for value in q_imu))
    if length < 1e-9 or not math.isfinite(length):
        raise ValueError('invalid Odin orientation')
    q_imu = tuple(value / length for value in q_imu)
    q_mount = quaternion_from_rpy(*mount[3:])
    q_base = multiply(q_imu, q_mount)
    offset = rotate(q_imu, mount[:3])

    origin = source.pose.pose.position
    output.pose.pose.position.x = origin.x + offset[0]
    output.pose.pose.position.y = origin.y + offset[1]
    output.pose.pose.position.z = origin.z + offset[2]
    output.pose.pose.orientation.x = q_base[0]
    output.pose.pose.orientation.y = q_base[1]
    output.pose.pose.orientation.z = q_base[2]
    output.pose.pose.orientation.w = q_base[3]
    output.pose.covariance = source.pose.covariance

    angular = source.twist.twist.angular
    w_odom = (angular.x, angular.y, angular.z)
    linear = source.twist.twist.linear
    v_imu = (linear.x, linear.y, linear.z)
    v_base_odom = (
        v_imu[0] + w_odom[1] * offset[2] - w_odom[2] * offset[1],
        v_imu[1] + w_odom[2] * offset[0] - w_odom[0] * offset[2],
        v_imu[2] + w_odom[0] * offset[1] - w_odom[1] * offset[0],
    )
    inverse = (-q_base[0], -q_base[1], -q_base[2], q_base[3])
    v_body = rotate(inverse, v_base_odom)
    w_body = rotate(inverse, w_odom)
    output.twist.twist.linear.x = v_body[0]
    output.twist.twist.linear.y = v_body[1]
    output.twist.twist.linear.z = v_body[2]
    output.twist.twist.angular.x = w_body[0]
    output.twist.twist.angular.y = w_body[1]
    output.twist.twist.angular.z = w_body[2]
    output.twist.covariance = source.twist.covariance
    return output


class OdinNavOdometry(Node):
    def __init__(self):
        super().__init__('odin_nav_odometry')
        self.declare_parameter('imu_to_base', [-0.15, 0.0, -0.39, 0.0, 0.0, math.pi])
        self.mount = tuple(float(v) for v in self.get_parameter('imu_to_base').value)
        if len(self.mount) != 6 or not all(math.isfinite(v) for v in self.mount):
            raise ValueError('imu_to_base must contain six finite numbers')
        self.publisher = self.create_publisher(
            Odometry, '/odin1/nav2_odometry', qos_profile_sensor_data)
        self.create_subscription(
            Odometry, '/odin1/odometry', self.on_odometry, qos_profile_sensor_data)

    def on_odometry(self, message):
        try:
            self.publisher.publish(adapt_odom(message, self.mount))
        except ValueError as exc:
            self.get_logger().warning(str(exc))


def main():
    rclpy.init()
    node = OdinNavOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
