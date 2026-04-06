#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

import board
import busio

try:
    from adafruit_bno08x.i2c import BNO08X_I2C
    from adafruit_bno08x import BNO_REPORT_GAME_ROTATION_VECTOR
    HAS_ADAFRUIT = True
except ImportError:
    HAS_ADAFRUIT = False


class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        self.get_logger().info('Starting BNO08x IMU Node (I2C, 0x4B, game rotation vector)...')

        self.bno = None
        self.i2c = None

        if HAS_ADAFRUIT:
            self.try_initialize_hardware()
        else:
            self.get_logger().error('Adafruit BNO08x library not found.')

        self.timer = self.create_timer(0.05, self.publish_imu)  # 20 Hz

    def try_initialize_hardware(self):
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.bno = BNO08X_I2C(self.i2c, address=0x4B)
            self.bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
            time.sleep(1.0)
            self.get_logger().info('BNO08x hardware initialized successfully.')
        except Exception as e:
            self.get_logger().error(f'Hardware init failed: {e}')
            self.bno = None

    def publish_imu(self):
        if self.bno is None:
            return

        try:
            time.sleep(0.005)
            q = self.bno.game_quaternion
            if q is None or len(q) != 4:
                return

            qx, qy, qz, qw = q

            if any(math.isnan(v) for v in (qx, qy, qz, qw)):
                self.get_logger().warn('Rejected NaN quaternion.')
                return

            norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
            if norm < 0.5 or norm > 1.5:
                self.get_logger().warn(
                    f'Rejected invalid quaternion norm={norm:.3f}: {(qx, qy, qz, qw)}'
                )
                return

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link'

            msg.orientation.x = qx
            msg.orientation.y = qy
            msg.orientation.z = qz
            msg.orientation.w = qw

            msg.orientation_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01
            ]

            msg.angular_velocity_covariance = [
                -1.0, 0.0, 0.0,
                0.0, -1.0, 0.0,
                0.0, 0.0, -1.0
            ]

            msg.linear_acceleration_covariance = [
                -1.0, 0.0, 0.0,
                0.0, -1.0, 0.0,
                0.0, 0.0, -1.0
            ]

            self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().warn(f'IMU read error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
