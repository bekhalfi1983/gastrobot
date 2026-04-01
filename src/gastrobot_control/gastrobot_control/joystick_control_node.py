#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class JoystickControlNode(Node):

    def __init__(self):
        super().__init__('joystick_control_node')

        # ================= LIMITS =================
        self.max_linear = 0.45
        self.max_angular = 1.2

        self.deadzone = 0.05

        # ================= ACCEL =================
        self.max_accel = 0.08
        self.max_turn_accel = 0.15

        # ================= MIN COMMAND (KEY FIX) =================
        self.min_linear = 0.08   # 🔥 prevents motor jitter
        self.min_angular = 0.1

        # ================= STATE =================
        self.linear_prev = 0.0
        self.angular_prev = 0.0
        self.speed_mode = 1.0

        # ================= ROS =================
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lift_pub = self.create_publisher(String, '/lift_cmd', 10)

        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.last_triangle = 0
        self.last_x = 0

        self.get_logger().info("Joystick Control READY (STABLE MODE)")

    # ================= DEADZONE =================
    def dz(self, val):
        return 0.0 if abs(val) < self.deadzone else val

    # ================= ACCEL LIMIT =================
    def limit_accel(self, target, current, max_delta):
        delta = target - current
        if abs(delta) > max_delta:
            delta = max_delta if delta > 0 else -max_delta
        return current + delta

    # ================= MIN OUTPUT FIX =================
    def apply_minimum(self, value, minimum):
        if value == 0.0:
            return 0.0
        if abs(value) < minimum:
            return minimum if value > 0 else -minimum
        return value

    # ================= CALLBACK =================
    def joy_callback(self, msg):

        # -------- INPUT --------
        forward = self.dz(msg.axes[1])
        turn = self.dz(msg.axes[3])

        # -------- SPEED MODES --------
        if msg.buttons[4]:
            self.speed_mode = 0.5
        elif msg.buttons[5]:
            self.speed_mode = 1.0

        # -------- TARGET --------
        linear_target = self.max_linear * self.speed_mode * forward
        angular_target = self.max_angular * self.speed_mode * turn

        # -------- ACCEL LIMIT --------
        linear = self.limit_accel(linear_target, self.linear_prev, self.max_accel)
        angular = self.limit_accel(angular_target, self.angular_prev, self.max_turn_accel)

        # -------- MINIMUM FORCE (KEY) --------
        linear = self.apply_minimum(linear, self.min_linear)
        angular = self.apply_minimum(angular, self.min_angular)

        self.linear_prev = linear
        self.angular_prev = angular

        # -------- PUBLISH --------
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_pub.publish(twist)

        # -------- LIFT --------
        triangle = msg.buttons[2]
        x_button = msg.buttons[0]

        if triangle and not self.last_triangle:
            self.send_lift("UP")

        if x_button and not self.last_x:
            self.send_lift("HOME")

        self.last_triangle = triangle
        self.last_x = x_button

    def send_lift(self, cmd):
        msg = String()
        msg.data = cmd
        self.lift_pub.publish(msg)


def main():
    rclpy.init()
    node = JoystickControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
