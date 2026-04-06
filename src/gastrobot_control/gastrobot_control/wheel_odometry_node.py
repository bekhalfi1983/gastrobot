#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__('wheel_odometry_node')

        # ================= PARAMETERS =================
        self.declare_parameter('wheel_radius', 0.072)     # meters
        self.declare_parameter('wheel_base', 0.40)        # meters
        self.declare_parameter('ticks_per_rev', 712.0)    # ESP32 currently uses CPR 712
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_rate', 30.0)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.ticks_per_rev = float(self.get_parameter('ticks_per_rev').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        # ================= STATE =================
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.current_left_ticks = None
        self.current_right_ticks = None
        self.prev_left_ticks = None
        self.prev_right_ticks = None
        self.have_ticks = False

        self.last_time = self.get_clock().now()

        # ================= ROS =================
        self.sub_ticks = self.create_subscription(
            Int32MultiArray,
            '/wheel_ticks',
            self.wheel_ticks_callback,
            10
        )

        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.update_odometry)

        self.get_logger().info('Wheel Odometry Node Started')

    def wheel_ticks_callback(self, msg: Int32MultiArray):
        if len(msg.data) < 2:
            self.get_logger().warn('Received /wheel_ticks with fewer than 2 values')
            return

        # These are TOTAL / cumulative counts from the bridge
        self.current_left_ticks = int(msg.data[0])
        self.current_right_ticks = int(msg.data[1])
        self.have_ticks = True

        # Initialize previous values on first message
        if self.prev_left_ticks is None or self.prev_right_ticks is None:
            self.prev_left_ticks = self.current_left_ticks
            self.prev_right_ticks = self.current_right_ticks

    def yaw_to_quaternion(self, yaw: float):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qz, qw

    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        if dt <= 0.0:
            return

        # Default: no movement
        delta_left_ticks = 0
        delta_right_ticks = 0

        if self.have_ticks and self.current_left_ticks is not None and self.current_right_ticks is not None:
            # IMPORTANT:
            # /wheel_ticks is cumulative, so we must subtract previous totals
            delta_left_ticks = self.current_left_ticks - self.prev_left_ticks
            delta_right_ticks = self.current_right_ticks - self.prev_right_ticks

            self.prev_left_ticks = self.current_left_ticks
            self.prev_right_ticks = self.current_right_ticks

        # Convert ticks to wheel travel
        meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev
        d_left = float(delta_left_ticks) * meters_per_tick
        d_right = float(delta_right_ticks) * meters_per_tick

        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.wheel_base

        # Midpoint integration
        theta_mid = self.theta + (d_theta / 2.0)
        self.x += d_center * math.cos(theta_mid)
        self.y += d_center * math.sin(theta_mid)
        self.theta += d_theta

        # Normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        vx = d_center / dt
        vth = d_theta / dt

        qz, qw = self.yaw_to_quaternion(self.theta)

        # ================= ODOM MESSAGE =================
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = vth

        # Reasonable planar covariance
        odom.pose.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]

        odom.twist.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.02
        ]

        self.pub_odom.publish(odom)

        # ================= TF =================
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame

            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0

            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw

            self.tf_broadcaster.sendTransform(t)

        self.last_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
