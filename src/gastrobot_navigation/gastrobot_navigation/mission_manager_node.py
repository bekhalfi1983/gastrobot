import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


def yaw_to_quat(yaw):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager')

        # --------------------------
        # Table Poses (EDIT LATER)
        # --------------------------
        self.table_poses = {
            "table_1": (1.0, 0.0, 0.0),
            "table_2": (2.0, 0.0, 0.0),
            "table_3": (3.0, 0.0, 0.0),
            "table_4": (4.0, 0.0, 0.0),
            "home":    (0.0, 0.0, 0.0),
        }

        # --------------------------
        # Action Client
        # --------------------------
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_goal_handle = None

        # --------------------------
        # State
        # --------------------------
        self.robot_state = "IDLE"
        self.current_table = "-"
        self.estop_latched = False

        # --------------------------
        # Subscribers (from GUI)
        # --------------------------
        self.create_subscription(String, '/navigate_to_table', self.on_table, 10)
        self.create_subscription(Bool, '/go_home', self.on_home, 10)
        self.create_subscription(Bool, '/cancel_navigation', self.on_cancel, 10)
        self.create_subscription(Bool, '/estop', self.on_estop, 10)

        # --------------------------
        # Publishers (to GUI + Lift)
        # --------------------------
        self.state_pub = self.create_publisher(String, '/robot_state', 10)
        self.table_pub = self.create_publisher(String, '/current_table', 10)
        self.goal_reached_pub = self.create_publisher(Bool, '/goal_reached', 10)

        self.publish_state()

        self.get_logger().info("Mission Manager Ready")

    # =========================================================
    # STATE MANAGEMENT
    # =========================================================
    def set_state(self, state):
        self.robot_state = state
        self.publish_state()

    def publish_state(self):
        msg = String()
        msg.data = self.robot_state
        self.state_pub.publish(msg)

        t = String()
        t.data = self.current_table
        self.table_pub.publish(t)

    # =========================================================
    # GUI CALLBACKS
    # =========================================================
    def on_table(self, msg):
        if self.estop_latched:
            self.get_logger().warn("E-STOP active, ignoring command")
            return

        table_id = msg.data.strip()

        if table_id not in self.table_poses:
            self.get_logger().error(f"Unknown table: {table_id}")
            return

        self.current_table = table_id
        self.publish_state()
        self.send_goal(table_id)

    def on_home(self, msg):
        if msg.data and not self.estop_latched:
            self.current_table = "home"
            self.publish_state()
            self.send_goal("home")

    def on_cancel(self, msg):
        if msg.data:
            self.cancel_goal()
            self.set_state("IDLE")

    def on_estop(self, msg):
        if msg.data:
            self.get_logger().error("E-STOP ACTIVATED")
            self.estop_latched = True
            self.cancel_goal()
            self.set_state("ESTOP")

    # =========================================================
    # NAVIGATION
    # =========================================================
    def send_goal(self, table_id):

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 not available")
            self.set_state("ERROR")
            return

        x, y, yaw = self.table_poses[table_id]
        qx, qy, qz, qw = yaw_to_quat(yaw)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.set_state("NAVIGATING")

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            self.set_state("ERROR")
            return

        self.current_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):

        if self.estop_latched:
            return

        status = future.result().status

        # 4 = SUCCEEDED
        if status == 4:
            self.get_logger().info("Navigation Succeeded")
            self.set_state("AT_TABLE")

            # Trigger lift
            msg = Bool()
            msg.data = True
            self.goal_reached_pub.publish(msg)

            self.set_state("SERVING")

        else:
            self.get_logger().error("Navigation Failed")
            self.set_state("ERROR")

        self.current_goal_handle = None

    def cancel_goal(self):
        if self.current_goal_handle is not None:
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
