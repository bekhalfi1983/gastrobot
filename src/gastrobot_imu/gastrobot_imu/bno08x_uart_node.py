#!/usr/bin/env python3

import math
import struct
import threading
import time
from typing import Optional

import serial
from serial import SerialException

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def euler_to_quaternion(yaw_deg: float, pitch_deg: float, roll_deg: float):
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


def wrap_rad(diff: float) -> float:
    while diff > math.pi:
        diff -= 2.0 * math.pi
    while diff < -math.pi:
        diff += 2.0 * math.pi
    return diff


class IMURVCNode(Node):
    def __init__(self):
        super().__init__('imu_uart_node')

        self.declare_parameter(
            'port',
            '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
        )
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('read_chunk_size', 256)
        self.declare_parameter('timer_period', 0.01)
        self.declare_parameter('reconnect_interval', 1.0)
        self.declare_parameter('max_buffer_bytes', 2048)

        self.port = str(self.get_parameter('port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.read_chunk_size = int(self.get_parameter('read_chunk_size').value)
        self.timer_period = float(self.get_parameter('timer_period').value)
        self.reconnect_interval = float(self.get_parameter('reconnect_interval').value)
        self.max_buffer_bytes = int(self.get_parameter('max_buffer_bytes').value)

        self.pub = self.create_publisher(Imu, '/imu/data', 10)

        self.ser: Optional[serial.Serial] = None
        self.serial_lock = threading.Lock()
        self.last_connect_attempt = 0.0

        self.buffer = bytearray()
        self.good_packets = 0
        self.bad_headers = 0
        self.packet_errors = 0
        self.last_warn_time = 0.0

        # For angular velocity estimation
        self.prev_time = None
        self.prev_yaw_rad = None
        self.prev_pitch_rad = None
        self.prev_roll_rad = None

        self.connect_serial(force=True)
        self.timer = self.create_timer(self.timer_period, self.read_imu)

    # ---------------- Serial helpers ----------------

    def connect_serial(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force and (now - self.last_connect_attempt) < self.reconnect_interval:
            return

        self.last_connect_attempt = now
        self.close_serial()

        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.05,
                exclusive=True,
            )
            self.buffer = bytearray()
            self.get_logger().info(f'UART IMU connected on {self.port} @ {self.baudrate}')
        except Exception as e:
            self.ser = None
            self.get_logger().error(f'Serial open failed: {e}')

    def close_serial(self) -> None:
        with self.serial_lock:
            if self.ser is not None:
                try:
                    if self.ser.is_open:
                        self.ser.close()
                except Exception:
                    pass
                finally:
                    self.ser = None

    def mark_serial_unhealthy(self, reason: str) -> None:
        now = time.monotonic()
        if now - self.last_warn_time > 1.0:
            self.last_warn_time = now
            self.get_logger().warn(reason)
        self.close_serial()

    # ---------------- Main read loop ----------------

    def read_imu(self) -> None:
        if self.ser is None:
            self.connect_serial()
            return

        try:
            with self.serial_lock:
                if self.ser is None:
                    return
                data = self.ser.read(self.read_chunk_size)

            # Empty read is normal sometimes; do not treat as fatal.
            if not data:
                return

            self.buffer.extend(data)

            # Prevent unbounded growth
            if len(self.buffer) > self.max_buffer_bytes:
                self.buffer = self.buffer[-512:]

            while len(self.buffer) >= 19:
                # Packet header for RVC stream
                if self.buffer[0] != 0xAA or self.buffer[1] != 0xAA:
                    self.buffer.pop(0)
                    self.bad_headers += 1
                    continue

                packet = self.buffer[:19]
                del self.buffer[:19]

                try:
                    # Angles stored in 0.01 degree units
                    yaw_raw = struct.unpack('<h', packet[3:5])[0]
                    pitch_raw = struct.unpack('<h', packet[5:7])[0]
                    roll_raw = struct.unpack('<h', packet[7:9])[0]

                    yaw_deg = yaw_raw / 100.0
                    pitch_deg = pitch_raw / 100.0
                    roll_deg = roll_raw / 100.0

                    qx, qy, qz, qw = euler_to_quaternion(yaw_deg, pitch_deg, roll_deg)

                    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
                    if not (0.5 < norm < 1.5):
                        self.packet_errors += 1
                        continue

                    current_time = self.get_clock().now().nanoseconds * 1e-9
                    yaw_rad = math.radians(yaw_deg)
                    pitch_rad = math.radians(pitch_deg)
                    roll_rad = math.radians(roll_deg)

                    if self.prev_time is not None:
                        dt = current_time - self.prev_time
                        if dt > 0.001:
                            wz = wrap_rad(yaw_rad - self.prev_yaw_rad) / dt
                            wy = wrap_rad(pitch_rad - self.prev_pitch_rad) / dt
                            wx = wrap_rad(roll_rad - self.prev_roll_rad) / dt
                        else:
                            wx = wy = wz = 0.0
                    else:
                        wx = wy = wz = 0.0

                    msg = Imu()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = self.frame_id

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

                    # Derived angular velocity
                    msg.angular_velocity.x = wx
                    msg.angular_velocity.y = wy
                    msg.angular_velocity.z = wz
                    msg.angular_velocity_covariance = [
                        0.02, 0.0, 0.0,
                        0.0, 0.02, 0.0,
                        0.0, 0.0, 0.02
                    ]

                    # No linear acceleration available in this RVC stream
                    msg.linear_acceleration_covariance = [
                        -1.0, 0.0, 0.0,
                        0.0, -1.0, 0.0,
                        0.0, 0.0, -1.0
                    ]

                    self.pub.publish(msg)

                    self.prev_time = current_time
                    self.prev_yaw_rad = yaw_rad
                    self.prev_pitch_rad = pitch_rad
                    self.prev_roll_rad = roll_rad

                    self.good_packets += 1
                    if self.good_packets % 100 == 0:
                        self.get_logger().info(
                            f'Published {self.good_packets} packets | '
                            f'yaw={yaw_deg:.2f} pitch={pitch_deg:.2f} roll={roll_deg:.2f}'
                        )

                except Exception as e:
                    self.packet_errors += 1
                    now = time.monotonic()
                    if now - self.last_warn_time > 1.0:
                        self.last_warn_time = now
                        self.get_logger().warn(f'Packet parse error: {e}')

        except (SerialException, OSError) as e:
            self.mark_serial_unhealthy(f'UART read error: {e}')
        except Exception as e:
            now = time.monotonic()
            if now - self.last_warn_time > 1.0:
                self.last_warn_time = now
                self.get_logger().warn(f'Unexpected IMU read issue: {e}')

    # ---------------- Shutdown ----------------

    def destroy_node(self):
        self.close_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IMURVCNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
