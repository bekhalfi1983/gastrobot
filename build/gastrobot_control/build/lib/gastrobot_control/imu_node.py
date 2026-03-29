#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

import board
import busio
import time

from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_ACCELEROMETER
)


class IMUNode(Node):

    def __init__(self):
        super().__init__('imu_node')

        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)

        # ===== INIT I2C =====
        self.get_logger().info("Initializing IMU...")

        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.bno = BNO08X_I2C(i2c, address=0x4B)
        except Exception as e:
            self.get_logger().error(f"I2C INIT FAILED: {e}")
            raise

        time.sleep(1.0)

        # ===== ENABLE FEATURES =====
        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE)

        time.sleep(1.0)

        # ===== LOOP =====
        self.timer = self.create_timer(0.02, self.publish_imu)  # 50 Hz

        self.get_logger().info("IMU Node Running (BNO085 @ 0x4B)")

    def publish_imu(self):

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        try:
            quat = self.bno.quaternion   # (w, x, y, z)
            gyro = self.bno.gyro
            accel = (0.0, 0.0, 0.0)

            # ===== ORIENTATION =====
            msg.orientation.w = quat[0]
            msg.orientation.x = quat[1]
            msg.orientation.y = quat[2]
            msg.orientation.z = quat[3]

            # ===== GYRO =====
            msg.angular_velocity.x = gyro[0]
            msg.angular_velocity.y = gyro[1]
            msg.angular_velocity.z = gyro[2]

            # ===== ACCEL =====
            msg.linear_acceleration.x = accel[0]
            msg.linear_acceleration.y = accel[1]
            msg.linear_acceleration.z = accel[2]

            # ===== COVARIANCE =====
            msg.orientation_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01
            ]

            msg.angular_velocity_covariance = [
                0.02, 0.0, 0.0,
                0.0, 0.02, 0.0,
                0.0, 0.0, 0.02
            ]

            msg.linear_acceleration_covariance = [
                0.10, 0.0, 0.0,
                0.0, 0.10, 0.0,
                0.0, 0.0, 0.10
            ]

            self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"IMU read error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
