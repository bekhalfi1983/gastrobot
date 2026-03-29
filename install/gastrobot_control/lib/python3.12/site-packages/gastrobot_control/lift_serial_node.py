import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time


class LiftSerialNode(Node):

    def __init__(self):
        super().__init__('lift_serial_node')

        # 🔥 STABLE SERIAL PORT
        self.port = '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_9503230373735171A2E1-if00'
        self.baud = 115200

        self.ser = None
        self.connect_serial()

        # ROS interfaces
        self.cmd_sub = self.create_subscription(
            String,
            '/lift_cmd',
            self.cmd_callback,
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/lift_status',
            10
        )

        # Timer for reading serial
        self.timer = self.create_timer(0.05, self.read_serial)

        self.get_logger().info("Lift Serial Node Started")

    # =========================
    # SERIAL CONNECTION
    # =========================
    def connect_serial(self):
        while True:
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
                time.sleep(2)  # allow Arduino reset
                self.get_logger().info(f"Connected to {self.port}")
                break
            except Exception as e:
                self.get_logger().error(f"Serial connect failed: {e}")
                time.sleep(2)

    # =========================
    # RECEIVE COMMAND FROM ROS
    # =========================
    def cmd_callback(self, msg):
        if self.ser is None:
            return

        try:
            cmd = msg.data.strip()
            self.ser.write((cmd + '\n').encode())  # 🔥 CRITICAL
            self.get_logger().info(f"Sent: {cmd}")
        except Exception as e:
            self.get_logger().error(f"Write failed: {e}")
            self.reconnect()

    # =========================
    # READ FROM SERIAL
    # =========================
    def read_serial(self):
        if self.ser is None:
            return

        try:
            if self.ser.in_waiting:
                line = self.ser.readline().decode().strip()
                if line:
                    msg = String()
                    msg.data = line
                    self.status_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Read failed: {e}")
            self.reconnect()

    # =========================
    # AUTO RECONNECT
    # =========================
    def reconnect(self):
        try:
            if self.ser:
                self.ser.close()
        except:
            pass

        self.get_logger().warn("Reconnecting serial...")
        self.connect_serial()


# =========================
# MAIN
# =========================
def main(args=None):
    rclpy.init(args=args)
    node = LiftSerialNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
