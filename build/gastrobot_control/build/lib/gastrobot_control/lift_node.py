import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial
import threading


class LiftNode(Node):

    def __init__(self):
        super().__init__('lift_node')

        # SERIAL PORT (Arduino)
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

        # ROS topics
        self.command_sub = self.create_subscription(
            String,
            '/lift_cmd',
            self.command_callback,
            10
        )

        self.state_pub = self.create_publisher(
            String,
            '/lift/state',
            10
        )

        # Start serial reader thread
        self.serial_thread = threading.Thread(target=self.read_serial)
        self.serial_thread.daemon = True
        self.serial_thread.start()

        self.get_logger().info("Lift node started")

    # -----------------------------
    # Send command to Arduino
    # -----------------------------
    def command_callback(self, msg):

        cmd = msg.data.strip()

        self.get_logger().info(f"Lift command: {cmd}")

        try:
            self.ser.write((cmd + "\n").encode())
        except Exception as e:
            self.get_logger().error(f"Serial write error: {e}")

    # -----------------------------
    # Read Arduino feedback
    # -----------------------------
    def read_serial(self):

        while rclpy.ok():

            try:
                line = self.ser.readline().decode().strip()

                if line != "":
                    self.get_logger().info(f"Lift: {line}")

                    msg = String()
                    msg.data = line
                    self.state_pub.publish(msg)

            except Exception:
                pass


def main(args=None):

    rclpy.init(args=args)

    node = LiftNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
