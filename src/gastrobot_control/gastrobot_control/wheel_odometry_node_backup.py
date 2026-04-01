import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math
import time


class WheelOdom(Node):
    def __init__(self):
        super().__init__('wheel_odometry_node')

        # ===== PARAMETERS =====
        self.wheel_radius = 0.072  # meters
        self.wheel_base = 0.56     # distance between wheels
        self.cpr = 1425            # encoder counts per revolution

        # ===== STATE =====
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = time.time()

        self.prev_l = 0
        self.prev_r = 0

        # ===== SUBSCRIBE =====
        self.sub = self.create_subscription(
            Int32MultiArray,
            '/wheel_ticks',
            self.tick_callback,
            10
        )

        # ===== PUBLISH =====
        self.pub = self.create_publisher(Odometry, '/wheel_odom', 10)

        self.get_logger().info("Wheel Odometry Node Started")


    def tick_callback(self, msg):
        l_ticks = msg.data[0]
        r_ticks = msg.data[1]

        # ===== TIME =====
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        if dt == 0:
            return

        # ===== DELTA TICKS =====
        dl = l_ticks - self.prev_l
        dr = r_ticks - self.prev_r

        self.prev_l = l_ticks
        self.prev_r = r_ticks

        # ===== DISTANCE =====
        dist_l = (dl / self.cpr) * 2 * math.pi * self.wheel_radius
        dist_r = (dr / self.cpr) * 2 * math.pi * self.wheel_radius

        # ===== KINEMATICS =====
        d_center = (dist_l + dist_r) / 2.0
        d_theta = (dist_r - dist_l) / self.wheel_base

        # ===== UPDATE POSE =====
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)
        self.theta += d_theta

        # ===== VELOCITY =====
        vx = d_center / dt
        vth = d_theta / dt

        # ===== QUATERNION =====
        q = Quaternion()
        q.z = math.sin(self.theta / 2.0)
        q.w = math.cos(self.theta / 2.0)

        # ===== ODOM MSG =====
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q

        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth

        self.pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
