import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped


class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager')

        # ---------- TABLE POSITIONS ----------
        self.table_positions = {
            "table_1": (1.0, 0.0),
            "table_2": (2.0, 0.0),
            "table_3": (3.0, 0.0),
            "table_4": (4.0, 0.0)
        }

        self.home_position = (0.0, 0.0)

        # ---------- PUBLISHERS ----------
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.robot_state_pub = self.create_publisher(String, '/robot_state', 10)
        self.current_table_pub = self.create_publisher(String, '/current_table', 10)
        self.lift_pub = self.create_publisher(String, '/lift_command', 10)

        # ---------- SUBSCRIBERS ----------
        self.create_subscription(String, '/navigate_to_table', self.table_cb, 10)
        self.create_subscription(Bool, '/cancel_navigation', self.cancel_cb, 10)
        self.create_subscription(Bool, '/estop', self.estop_cb, 10)
        self.create_subscription(Bool, '/go_home', self.home_cb, 10)

        self.state = "IDLE"

        self.get_logger().info("Mission Manager Started")

    # ------------------------------
    # NAVIGATE TO TABLE
    # ------------------------------
    def table_cb(self, msg):

        table = msg.data

        if table not in self.table_positions:
            self.get_logger().warn("Unknown table")
            return

        x, y = self.table_positions[table]

        goal = PoseStamped()

        goal.header.frame_id = "map"
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0

        self.goal_pub.publish(goal)

        state_msg = String()
        state_msg.data = "NAVIGATING"
        self.robot_state_pub.publish(state_msg)

        table_msg = String()
        table_msg.data = table
        self.current_table_pub.publish(table_msg)

        self.state = "NAVIGATING"

        self.get_logger().info(f"Going to {table}")

    # ------------------------------
    # CANCEL
    # ------------------------------
    def cancel_cb(self, msg):

        if msg.data:

            state_msg = String()
            state_msg.data = "IDLE"

            self.robot_state_pub.publish(state_msg)

            self.state = "IDLE"

            self.get_logger().info("Mission cancelled")

    # ------------------------------
    # HOME
    # ------------------------------
    def home_cb(self, msg):

        if msg.data:

            x, y = self.home_position

            goal = PoseStamped()

            goal.header.frame_id = "map"
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.orientation.w = 1.0

            self.goal_pub.publish(goal)

            state_msg = String()
            state_msg.data = "NAVIGATING"

            self.robot_state_pub.publish(state_msg)

            self.get_logger().info("Returning Home")

    # ------------------------------
    # ESTOP
    # ------------------------------
    def estop_cb(self, msg):

        if msg.data:

            state_msg = String()
            state_msg.data = "ESTOP"

            self.robot_state_pub.publish(state_msg)

            self.get_logger().error("EMERGENCY STOP")


def main(args=None):

    rclpy.init(args=args)

    node = MissionManager()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
