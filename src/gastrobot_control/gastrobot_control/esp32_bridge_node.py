#!/usr/bin/env python3

import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist


class ESP32BridgeNode(Node):
    def __init__(self):
        super().__init__('esp32_bridge_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_radius', 0.072)
        self.declare_parameter('wheel_base', 0.56)

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)

        self.ser = None

        self.tick_pub = self.create_publisher(Int32MultiArray, '/wheel_ticks', 10)

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.timer = self.create_timer(0.02, self.read_serial)

        self.get_logger().info('=== ESP32 BRIDGE NODE RUNNING ===')
        self.connect_serial()

    # ================= SERIAL =================
    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.01)
            self.get_logger().info('ESP32 CONNECTED')
        except Exception as e:
            self.get_logger().error(f'Serial failed: {e}')
            self.ser = None

    # ================= CMD_VEL =================
    def cmd_vel_callback(self, msg: Twist):
        if self.ser is None:
            return

        v = msg.linear.x
        w = msg.angular.z

        v_l = v - (w * self.wheel_base / 2.0)
        v_r = v + (w * self.wheel_base / 2.0)

        rpm_l = (v_l / (2.0 * 3.14159 * self.wheel_radius)) * 60.0
        rpm_r = (v_r / (2.0 * 3.14159 * self.wheel_radius)) * 60.0

        line = f'CMD,{rpm_l:.2f},{rpm_r:.2f}\n'

        try:
            self.ser.write(line.encode())
        except:
            self.ser = None

    # ================= SERIAL READ =================
    def read_serial(self):
        if self.ser is None:
            self.connect_serial()
            return

        try:
            while self.ser.in_waiting > 0:
                raw = self.ser.readline().decode(errors='ignore').strip()

                if not raw:
                    continue

                # ================= DATA PARSE =================
                if raw.startswith('DATA,'):
                    parts = raw.split(',')

                    if len(parts) >= 12:
                        try:
                            # NEW: use REAL cumulative counts
                            left = int(parts[8])
                            right = int(parts[9])

                            msg = Int32MultiArray()
                            msg.data = [left, right]
                            self.tick_pub.publish(msg)

                        except:
                            self.get_logger().warn(f'Bad DATA: {raw}')

        except Exception as e:
            self.get_logger().error(f'Serial error: {e}')
            self.ser = None


def main(args=None):
    rclpy.init(args=args)
    node = ESP32BridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
