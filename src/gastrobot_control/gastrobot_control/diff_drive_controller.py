#!/usr/bin/env python3
"""
diff_drive_controller.py  (ROS 2 Jazzy)

Converts /cmd_vel → /wheel_cmd_rpm

Output format:
[ left_rpm, right_rpm ]
"""

import math
import time
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def rate_limit(target, prev, max_delta):
    if target > prev + max_delta:
        return prev + max_delta
    if target < prev - max_delta:
        return prev - max_delta
    return target


class DiffDriveController(Node):

    def __init__(self):

        super().__init__("diff_drive_controller")

        # =============================
        # Robot Parameters (UPDATED)
        # =============================

        self.declare_parameter("wheel_radius_m", 0.072)   # 144mm diameter
        self.declare_parameter("wheel_base_m", 0.56)      # center-to-center

        self.declare_parameter("max_wheel_rpm", 160.0)

        # Safety
        self.declare_parameter("cmd_timeout_s", 0.4)

        # Loop rate
        self.declare_parameter("rate_hz", 30.0)

        # Smoothing
        self.declare_parameter("enable_ramp", True)
        self.declare_parameter("max_rpm_slew_per_s", 400.0)

        # Scaling (for tuning later)
        self.declare_parameter("linear_scale", 1.0)
        self.declare_parameter("angular_scale", 1.0)

        # =============================
        # Load Parameters
        # =============================

        self.wheel_radius_m = float(self.get_parameter("wheel_radius_m").value)
        self.wheel_base_m = float(self.get_parameter("wheel_base_m").value)

        self.max_wheel_rpm = float(self.get_parameter("max_wheel_rpm").value)

        self.cmd_timeout_s = float(self.get_parameter("cmd_timeout_s").value)

        self.rate_hz = float(self.get_parameter("rate_hz").value)

        self.enable_ramp = bool(self.get_parameter("enable_ramp").value)
        self.max_rpm_slew_per_s = float(self.get_parameter("max_rpm_slew_per_s").value)

        self.linear_scale = float(self.get_parameter("linear_scale").value)
        self.angular_scale = float(self.get_parameter("angular_scale").value)

        # =============================
        # ROS I/O
        # =============================

        self.sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_cb,
            10
        )

        self.pub = self.create_publisher(
            Float32MultiArray,
            "/wheel_cmd_rpm",
            10
        )

        # =============================
        # State
        # =============================

        self.last_cmd_time = 0.0

        self.target_left_rpm = 0.0
        self.target_right_rpm = 0.0

        self.out_left_rpm = 0.0
        self.out_right_rpm = 0.0

        self.last_loop_time = time.monotonic()
        self.last_debug_time = time.monotonic()

        # =============================
        # Timer
        # =============================

        self.timer = self.create_timer(
            1.0 / max(self.rate_hz, 1.0),
            self.loop
        )

        self.get_logger().info(
            f"DiffDrive started "
            f"(radius={self.wheel_radius_m:.3f} m, base={self.wheel_base_m:.3f} m)"
        )

    # ===================================
    # Receive cmd_vel
    # ===================================

    def cmd_cb(self, msg):

        now = time.monotonic()
        self.last_cmd_time = now

        v = float(msg.linear.x) * self.linear_scale
        w = float(msg.angular.z) * self.angular_scale

        # Differential kinematics
        v_left = v - (w * self.wheel_base_m / 2.0)
        v_right = v + (w * self.wheel_base_m / 2.0)

        denom = (2.0 * math.pi * self.wheel_radius_m)

        if denom <= 0.0:
            self.target_left_rpm = 0.0
            self.target_right_rpm = 0.0
            return

        left_rpm = (v_left / denom) * 60.0
        right_rpm = (v_right / denom) * 60.0

        self.target_left_rpm = clamp(
            left_rpm,
            -self.max_wheel_rpm,
            self.max_wheel_rpm
        )

        self.target_right_rpm = clamp(
            right_rpm,
            -self.max_wheel_rpm,
            self.max_wheel_rpm
        )

    # ===================================
    # Main Control Loop
    # ===================================

    def loop(self):

        now = time.monotonic()
        dt = max(now - self.last_loop_time, 1e-6)
        self.last_loop_time = now

        # -------------------------
        # Watchdog
        # -------------------------
        if (now - self.last_cmd_time) > self.cmd_timeout_s:
            self.target_left_rpm = 0.0
            self.target_right_rpm = 0.0

        # -------------------------
        # Ramp Limiting
        # -------------------------
        if self.enable_ramp:

            max_delta = self.max_rpm_slew_per_s * dt

            self.out_left_rpm = rate_limit(
                self.target_left_rpm,
                self.out_left_rpm,
                max_delta
            )

            self.out_right_rpm = rate_limit(
                self.target_right_rpm,
                self.out_right_rpm,
                max_delta
            )

        else:

            self.out_left_rpm = self.target_left_rpm
            self.out_right_rpm = self.target_right_rpm

        # -------------------------
        # Publish
        # -------------------------
        msg = Float32MultiArray()
        msg.data = [
            float(self.out_left_rpm),
            float(self.out_right_rpm)
        ]

        self.pub.publish(msg)

        # -------------------------
        # Debug (1 Hz)
        # -------------------------
        if now - self.last_debug_time > 1.0:
            self.get_logger().info(
                f"L:{self.out_left_rpm:.1f} RPM | R:{self.out_right_rpm:.1f} RPM"
            )
            self.last_debug_time = now


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