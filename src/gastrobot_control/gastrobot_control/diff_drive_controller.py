#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class DiffDriveController(Node):
    def __init__(self):
        super().__init__("diff_drive_controller")

        # Physical specs
        self.radius = 0.072   # 144 mm diameter wheel
        self.base = 0.57      # wheel separation used by controller

        # Output publisher
        self.rpm_pub = self.create_publisher(Float32MultiArray, "/wheel_cmd_rpm", 10)

        # Input command
        self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 10)

        self.get_logger().info("DiffDriveController active: /cmd_vel -> /wheel_cmd_rpm")

    def cmd_cb(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        # Differential drive wheel linear velocities
        v_l = v - (w * self.base / 2.0)
        v_r = v + (w * self.base / 2.0)

        # Convert linear wheel velocity to wheel RPM
        circumference = 2.0 * math.pi * self.radius
        l_rpm = (v_l / circumference) * 60.0
        r_rpm = (v_r / circumference) * 60.0

        # ================= BIAS CORRECTION =================
        left_bias = 1.09   # adjust this
        right_bias = 1.00

        l_rpm *= left_bias
        r_rpm *= right_bias
        # ==================================================

        out = Float32MultiArray()
        out.data = [float(l_rpm), float(r_rpm)]
        self.rpm_pub.publish(out)


def main():
    rclpy.init()
    node = DiffDriveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
