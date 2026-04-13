#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


def quaternion_to_yaw(x, y, z, w):
    """
    Convert quaternion to yaw (Z-axis rotation), radians.
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    """
    Normalize angle to [-pi, pi].
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class YawCorrectionNode(Node):
    def __init__(self):
        super().__init__('yaw_correction_node')

        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('invert_yaw', False)

        imu_topic = self.get_parameter('imu_topic').value
        self.publish_debug = self.get_parameter('publish_debug').value
        self.invert_yaw = self.get_parameter('invert_yaw').value

        self.sub_imu = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            10
        )

        self.pub_yaw_raw = self.create_publisher(Float64, '/imu/yaw_raw', 10)
        self.pub_yaw_corrected = self.create_publisher(Float64, '/imu/yaw_corrected', 10)
        self.pub_heading_deg = self.create_publisher(Float64, '/imu/heading_deg', 10)

        self.zero_yaw = None
        self.prev_corrected_yaw = None
        self.unwrapped_yaw = 0.0
        self.msg_count = 0

        self.get_logger().info(f'Yaw correction node started. Subscribed to {imu_topic}')

    def imu_callback(self, msg: Imu):
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        yaw_raw = quaternion_to_yaw(qx, qy, qz, qw)

        if self.invert_yaw:
            yaw_raw = -yaw_raw

        if self.zero_yaw is None:
            self.zero_yaw = yaw_raw
            self.prev_corrected_yaw = 0.0
            self.unwrapped_yaw = 0.0
            self.get_logger().info(
                f'Initial yaw locked. zero_yaw = {self.zero_yaw:.4f} rad '
                f'({math.degrees(self.zero_yaw):.2f} deg)'
            )

        corrected_yaw = normalize_angle(yaw_raw - self.zero_yaw)

        delta = corrected_yaw - self.prev_corrected_yaw
        delta = normalize_angle(delta)
        self.unwrapped_yaw += delta
        self.prev_corrected_yaw = corrected_yaw

        heading_deg = math.degrees(self.unwrapped_yaw)

        raw_msg = Float64()
        raw_msg.data = yaw_raw

        corrected_msg = Float64()
        corrected_msg.data = self.unwrapped_yaw

        heading_msg = Float64()
        heading_msg.data = heading_deg

        self.pub_yaw_raw.publish(raw_msg)
        self.pub_yaw_corrected.publish(corrected_msg)
        self.pub_heading_deg.publish(heading_msg)

        self.msg_count += 1
        if self.publish_debug and self.msg_count % 100 == 0:
            self.get_logger().info(
                f'yaw_raw={yaw_raw:.4f} rad | '
                f'yaw_corrected={self.unwrapped_yaw:.4f} rad | '
                f'heading={heading_deg:.2f} deg'
            )


def main(args=None):
    rclpy.init(args=args)
    node = YawCorrectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
