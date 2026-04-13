#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


def clamp(val, min_val, max_val):
    return max(min(val, max_val), min_val)


def angle_diff(a, b):
    diff = a - b
    while diff > 180:
        diff -= 360
    while diff < -180:
        diff += 360
    return diff


class HeadingHoldNode(Node):
    def __init__(self):
        super().__init__('heading_hold_node')

        # ===== PARAMETERS (STABLE + SMOOTH) =====
        self.declare_parameter('kp', 0.04)
        self.declare_parameter('max_angular_z', 0.12)
        self.declare_parameter('deadband_deg', 2.0)
        self.declare_parameter('enable_threshold', 0.05)

        self.kp = self.get_parameter('kp').value
        self.max_angular = self.get_parameter('max_angular_z').value
        self.deadband = self.get_parameter('deadband_deg').value
        self.enable_threshold = self.get_parameter('enable_threshold').value

        # ===== SUBSCRIBERS =====
        self.sub_cmd = self.create_subscription(
            Twist,
            '/cmd_vel_out',
            self.cmd_callback,
            10
        )

        self.sub_heading = self.create_subscription(
            Float64,
            '/imu/heading_deg',
            self.heading_callback,
            10
        )

        # ===== PUBLISHER =====
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel_pre_smooth', 10)

        # ===== STATE =====
        self.current_heading = 0.0
        self.target_heading = None
        self.active = False

        # 🔥 smoothing memory
        self.prev_correction = 0.0

        # 🔥 IMU filtering
        self.prev_heading = None
        self.max_heading_jump = 10.0  # degrees per update

        self.get_logger().info("✅ Heading Hold Node (FINAL STABLE VERSION)")

    # ===== IMU CALLBACK (FILTERED) =====
    def heading_callback(self, msg):
        new_heading = msg.data

        if self.prev_heading is None:
            self.prev_heading = new_heading
            self.current_heading = new_heading
            return

        diff = angle_diff(new_heading, self.prev_heading)

        # 🔥 REJECT SUDDEN IMU JUMPS
        if abs(diff) > self.max_heading_jump:
            return

        # 🔥 LIGHT SMOOTHING
        alpha = 0.3
        smoothed = alpha * new_heading + (1 - alpha) * self.prev_heading

        self.current_heading = smoothed
        self.prev_heading = smoothed

    # ===== CMD CALLBACK =====
    def cmd_callback(self, msg):
        output = Twist()
        output.linear = msg.linear
        output.angular = msg.angular

        moving_forward = abs(msg.linear.x) > self.enable_threshold
        rotating = abs(msg.angular.z) > 0.05

        # ===== RESET when user rotates =====
        if rotating:
            self.active = False
            self.target_heading = None
            self.prev_correction = 0.0
            self.pub_cmd.publish(msg)
            return

        # ===== ACTIVATE heading lock =====
        if moving_forward and not self.active:
            self.target_heading = self.current_heading
            self.active = True
            self.get_logger().info(
                f"🔒 Heading locked at {self.target_heading:.2f} deg"
            )

        # ===== DEACTIVATE when stopped =====
        if not moving_forward:
            self.active = False
            self.target_heading = None
            self.prev_correction = 0.0
            self.pub_cmd.publish(msg)
            return

        # ===== APPLY CORRECTION =====
        if self.active and self.target_heading is not None:
            error = angle_diff(self.target_heading, self.current_heading)

            # clamp large jumps
            error = clamp(error, -15.0, 15.0)

            # deadband
            if abs(error) < self.deadband:
                correction = 0.0
            else:
                # 🔥 softened response
                correction = self.kp * error * 0.5

            # clamp angular velocity
            correction = clamp(correction, -self.max_angular, self.max_angular)

            # 🔥 SMOOTH CORRECTION
            alpha = 0.2
            smooth_correction = alpha * correction + (1 - alpha) * self.prev_correction
            self.prev_correction = smooth_correction

            # 🔥 BLENDING (CRITICAL FIX)
            output.angular.z = msg.angular.z + smooth_correction

        self.pub_cmd.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = HeadingHoldNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
