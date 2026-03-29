#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class JoystickControlNode(Node):

    def __init__(self):
        super().__init__('joystick_control_node')

        # ================= SAFE LIMITS =================
        self.max_linear = 0.14
        self.max_angular = 0.55

        self.deadzone = 0.08

        # ================= SMOOTHING =================
        self.alpha = 0.04

        # ================= ACCELERATION =================
        self.max_accel = 0.01          # 🔥 VERY SOFT START
        self.max_turn_accel = 0.04

        # ================= EXPO =================
        self.expo = 0.85              # 🔥 VERY SMOOTH RESPONSE

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

        self.get_logger().info("Joystick Control READY (ULTRA SMOOTH MODE)")

    # ================= DEADZONE =================
    def dz(self, val):
        return 0.0 if abs(val) < self.deadzone else val

    # ================= STRONG EXPO =================
    def expo_curve(self, x):
        return (1 - self.expo) * x + self.expo * (x ** 3)

    # ================= ACCEL LIMIT =================
    def limit_accel(self, target, current, max_delta):
        delta = target - current
        if abs(delta) > max_delta:
            delta = max_delta if delta > 0 else -max_delta
        return current + delta

    # ================= CALLBACK =================
    def joy_callback(self, msg):

        # -------- AXES --------
        forward_raw = self.dz(msg.axes[1])
        turn_raw = self.dz(msg.axes[3])

        # -------- EXPO --------
        forward = self.expo_curve(forward_raw)
        turn = self.expo_curve(turn_raw)

        # -------- SPEED MODES --------
        if msg.buttons[4]:       # L1 → precision
            self.speed_mode = 0.3
        elif msg.buttons[5]:     # R1 → normal
            self.speed_mode = 1.0

        # -------- DYNAMIC LIMIT (ANTI-WHEELIE) --------
        # reduces top speed when stick is pushed hard
        dynamic_limit = 1.0 - 0.4 * abs(forward)

        # -------- TARGET --------
        linear_target = self.max_linear * self.speed_mode * forward * dynamic_limit
        angular_target = self.max_angular * self.speed_mode * turn

        # -------- SOFT START --------
        linear = self.limit_accel(linear_target, self.linear_prev, self.max_accel)
        angular = self.limit_accel(angular_target, self.angular_prev, self.max_turn_accel)

        # -------- EXTRA SMOOTH --------
        linear = self.alpha * linear + (1 - self.alpha) * self.linear_prev
        angular = self.alpha * angular + (1 - self.alpha) * self.angular_prev

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
        self.get_logger().info(f"Lift: {cmd}")


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
