#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

class DiffDriveController(Node):
    def __init__(self):
        super().__init__("diff_drive_controller")

        # 1. Physical Specs (Updated to your hardware)
        self.radius = 0.072        # 144mm diameter
        self.base = 0.57           # 620mm outer - 50mm wheel width
        self.tpr = 1425.1          # goBILDA Yellow Jacket 50.9:1 spec
        self.m_per_tick = (2 * math.pi * self.radius) / self.tpr

        # 2. Odometry State
        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.last_left, self.last_right = None, None
        self.imu_yaw, self.has_imu = 0.0, False
        self.last_time = self.get_clock().now()

        # 3. ROS I/O
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.rpm_pub = self.create_publisher(Float32MultiArray, "/wheel_cmd_rpm", 10)

        # Subscriptions
        self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 10)
        self.create_subscription(Int32MultiArray, "/wheel_ticks", self.ticks_cb, 10)
        self.create_subscription(Imu, "/imu/data", self.imu_cb, 10)

        self.get_logger().info("Gastrobot: Direct Odometry + IMU Active")

    def imu_cb(self, msg):
        q = msg.orientation
        # Convert Quaternion to Yaw
        self.imu_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self.has_imu = True

    def ticks_cb(self, msg):
        if len(msg.data) < 2: return
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        
        if self.last_left is None:
            self.last_left, self.last_right, self.last_time = msg.data[0], msg.data[1], now
            return

        # Distance Math
        d_l = (msg.data[0] - self.last_left) * self.m_per_tick
        d_r = (msg.data[1] - self.last_right) * self.m_per_tick
        d_dist = (d_l + d_r) / 2.0

        # Heading: Use IMU as absolute truth, fall back to encoders if IMU dies
        if self.has_imu:
            self.th = self.imu_yaw
        else:
            self.th += (d_r - d_l) / self.base

        # Integrate Position
        self.x += d_dist * math.cos(self.th)
        self.y += d_dist * math.sin(self.th)

        # Broadcast the "Moving Robot" transform
        self.publish_odom_and_tf(now, d_dist/dt if dt > 0 else 0.0)
        self.last_left, self.last_right, self.last_time = msg.data[0], msg.data[1], now

    def publish_odom_and_tf(self, now, linear_vel):
        # Publish TF: odom -> base_footprint (This moves the box in Rviz)
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = now.to_msg(), "odom", "base_footprint"
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        t.transform.rotation.z, t.transform.rotation.w = math.sin(self.th/2), math.cos(self.th/2)
        self.tf_broadcaster.sendTransform(t)

        # Publish Odom topic for Navigation
        o = Odometry()
        o.header = t.header
        o.child_frame_id = t.child_frame_id
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation = t.transform.rotation
        o.twist.twist.linear.x = float(linear_vel)
        self.odom_pub.publish(o)

    def cmd_cb(self, msg):
        v, w = msg.linear.x, msg.angular.z
        v_l = v - (w * self.base / 2.0)
        v_r = v + (w * self.base / 2.0)
        
        # Convert to RPM for ESP32
        circ = 2 * math.pi * self.radius
        l_rpm = (v_l / circ) * 60.0
        r_rpm = (v_r / circ) * 60.0
        
        out = Float32MultiArray()
        out.data = [float(l_rpm), float(r_rpm)]
        self.rpm_pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(DiffDriveController())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
