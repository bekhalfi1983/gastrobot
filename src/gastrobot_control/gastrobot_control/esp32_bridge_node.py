#!/usr/bin/env python3

import serial
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist


class ESP32BridgeNode(Node):
    def __init__(self):
        super().__init__('esp32_bridge_node')

        self.declare_parameter(
            'port',
            '/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_a67f795ee971f0119cc3f99e1045c30f-if00-port0'
        )
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_radius', 0.072)
        self.declare_parameter('wheel_base', 0.560)

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)

        self.ser = None
        self.last_left_ticks = None
        self.last_right_ticks = None

        self.tick_pub = self.create_publisher(Int32MultiArray, '/wheel_ticks', 10)
        self.rpm_pub = self.create_publisher(Float32MultiArray, '/wheel_rpm', 10)

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.timer = self.create_timer(0.02, self.read_serial)

        self.get_logger().info('=== ESP32 BRIDGE NODE RUNNING ===')
        self.get_logger().info(f'Connecting to {self.port}')
        self.connect_serial()

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.01)
            self.get_logger().info('ESP32 CONNECTED')
        except Exception as e:
            self.get_logger().error(f'Serial connection failed: {e}')
            self.ser = None

    def cmd_vel_callback(self, msg: Twist):
        if self.ser is None:
            return

        v = msg.linear.x
        w = msg.angular.z

        v_l = v - (w * self.wheel_base / 2.0)
        v_r = v + (w * self.wheel_base / 2.0)

        rpm_l = (v_l / (2.0 * 3.141592653589793 * self.wheel_radius)) * 60.0
        rpm_r = (v_r / (2.0 * 3.141592653589793 * self.wheel_radius)) * 60.0

        line = f'V,{rpm_l:.2f},{rpm_r:.2f}\n'

        try:
            self.ser.write(line.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f'Serial write failed: {e}')
            self.ser = None

    def read_serial(self):
        if self.ser is None:
            self.connect_serial()
            return

        try:
            while self.ser.in_waiting > 0:
                raw = self.ser.readline().decode('utf-8', errors='ignore').strip()

                if not raw:
                    continue

                # ESP32 ROS tick line format: T,left,right
                if raw.startswith('T,'):
                    parts = raw.split(',')

                    if len(parts) != 3:
                        continue

                    left = int(parts[1])
                    right = int(parts[2])

                    tick_msg = Int32MultiArray()
                    tick_msg.data = [left, right]
                    self.tick_pub.publish(tick_msg)

                    # Optional wheel_rpm estimate from tick delta
                    if self.last_left_ticks is not None and self.last_right_ticks is not None:
                        dl = left - self.last_left_ticks
                        dr = right - self.last_right_ticks

                        dt = 0.02  # timer period
                        cpr = 1425.0

                        rpm_l = (dl / cpr) * (60.0 / dt)
                        rpm_r = (dr / cpr) * (60.0 / dt)

                        rpm_msg = Float32MultiArray()
                        rpm_msg.data = [float(rpm_l), float(rpm_r)]
                        self.rpm_pub.publish(rpm_msg)

                    self.last_left_ticks = left
                    self.last_right_ticks = right

                # Ignore debug lines like:
                # DBG T:50.00 | L:...
        except Exception as e:
            self.get_logger().error(f'Serial read failed: {e}')
            self.ser = None


def main(args=None):
    rclpy.init(args=args)
    node = ESP32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser is not None:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
