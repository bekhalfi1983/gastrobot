#!/usr/bin/env python3

import math
import struct
import serial

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


PORT = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
BAUD = 115200


def euler_to_quaternion(yaw_deg, pitch_deg, roll_deg):
    yaw = math.radians(yaw_deg)
    pitch = math.radians(pitch_deg)
    roll = math.radians(roll_deg)

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw


class IMURVCNode(Node):
    def __init__(self):
        super().__init__('imu_uart_node')

        self.pub = self.create_publisher(Imu, '/imu/data', 10)

        try:
            self.ser = serial.Serial(PORT, BAUD, timeout=0.05)
            self.get_logger().info(f"UART IMU connected on {PORT}")
        except Exception as e:
            self.get_logger().error(f"Serial open failed: {e}")
            raise

        self.buffer = bytearray()

        self.good_packets = 0
        self.bad_headers = 0

        # For angular velocity estimation
        self.prev_time = None
        self.prev_yaw = None
        self.prev_pitch = None
        self.prev_roll = None

        self.timer = self.create_timer(0.01, self.read_imu)  # 100 Hz

    def read_imu(self):
        try:
            data = self.ser.read(256)
            if data:
                self.buffer.extend(data)

            # Prevent buffer overflow
            if len(self.buffer) > 2048:
                self.buffer = self.buffer[-512:]

            while len(self.buffer) >= 19:

                # Check header
                if self.buffer[0] != 0xAA or self.buffer[1] != 0xAA:
                    self.buffer.pop(0)
                    self.bad_headers += 1
                    continue

                packet = self.buffer[:19]
                del self.buffer[:19]

                try:
                    # Extract angles (0.01 deg units)
                    yaw_raw = struct.unpack('<h', packet[3:5])[0]
                    pitch_raw = struct.unpack('<h', packet[5:7])[0]
                    roll_raw = struct.unpack('<h', packet[7:9])[0]

                    yaw_deg = yaw_raw / 100.0
                    pitch_deg = pitch_raw / 100.0
                    roll_deg = roll_raw / 100.0

                    # Convert to quaternion
                    qx, qy, qz, qw = euler_to_quaternion(yaw_deg, pitch_deg, roll_deg)

                    # Validate quaternion
                    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
                    if not (0.5 < norm < 1.5):
                        continue

                    # Compute angular velocity
                    current_time = self.get_clock().now().nanoseconds * 1e-9

                    if self.prev_time is not None:
                        dt = current_time - self.prev_time

                        if dt > 0.001:
                            yaw_rad = math.radians(yaw_deg)
                            pitch_rad = math.radians(pitch_deg)
                            roll_rad = math.radians(roll_deg)

                            prev_yaw_rad = math.radians(self.prev_yaw)
                            prev_pitch_rad = math.radians(self.prev_pitch)
                            prev_roll_rad = math.radians(self.prev_roll)

                            wz = (yaw_rad - prev_yaw_rad) / dt
                            wy = (pitch_rad - prev_pitch_rad) / dt
                            wx = (roll_rad - prev_roll_rad) / dt
                        else:
                            wx = wy = wz = 0.0
                    else:
                        wx = wy = wz = 0.0

                    # Build ROS IMU message
                    msg = Imu()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'imu_link'

                    # Orientation
                    msg.orientation.x = qx
                    msg.orientation.y = qy
                    msg.orientation.z = qz
                    msg.orientation.w = qw

                    msg.orientation_covariance = [
                        0.01, 0.0, 0.0,
                        0.0, 0.01, 0.0,
                        0.0, 0.0, 0.01
                    ]

                    # Angular velocity (derived)
                    msg.angular_velocity.x = wx
                    msg.angular_velocity.y = wy
                    msg.angular_velocity.z = wz

                    msg.angular_velocity_covariance = [
                        0.02, 0.0, 0.0,
                        0.0, 0.02, 0.0,
                        0.0, 0.0, 0.02
                    ]

                    # No accel in RVC
                    msg.linear_acceleration_covariance = [
                        -1.0, 0.0, 0.0,
                        0.0, -1.0, 0.0,
                        0.0, 0.0, -1.0
                    ]

                    self.pub.publish(msg)

                    # Save previous values
                    self.prev_time = current_time
                    self.prev_yaw = yaw_deg
                    self.prev_pitch = pitch_deg
                    self.prev_roll = roll_deg

                    self.good_packets += 1

                    if self.good_packets % 100 == 0:
                        self.get_logger().info(
                            f"Published {self.good_packets} packets | "
                            f"yaw={yaw_deg:.2f} pitch={pitch_deg:.2f} roll={roll_deg:.2f}"
                        )

                except Exception as e:
                    self.get_logger().warn(f"Packet parse error: {e}")

        except Exception as e:
            self.get_logger().warn(f"UART read error: {e}")


def main():
    rclpy.init()
    node = IMURVCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
