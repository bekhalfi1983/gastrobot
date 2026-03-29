#!/usr/bin/env python3

import time
import serial
import rclpy
import re

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray


class MotorDriveNode(Node):

    def __init__(self):
        super().__init__("motor_drive_node")
        self.get_logger().info("=== MAIN WORKSPACE NODE RUNNING ===")

        # ================= PARAMETERS =================
        self.declare_parameter("port", "/dev/ttyUSB1")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("max_accel", 50.0)   # RPM per second
        self.declare_parameter("cmd_timeout", 0.5)  # seconds

        self.port = self.get_parameter("port").value
        self.baud = self.get_parameter("baud").value
        self.max_accel = self.get_parameter("max_accel").value
        self.cmd_timeout = self.get_parameter("cmd_timeout").value

        # ================= STATE =================
        self.left_cmd_target = 0.0
        self.right_cmd_target = 0.0

        self.left_cmd = 0.0
        self.right_cmd = 0.0

        self.last_cmd_time = time.monotonic()

        # encoder tracking
        self.last_left_raw = None
        self.last_right_raw = None

        self.total_left = 0
        self.total_right = 0

        self.buffer = ""

        self.last_debug_time = time.monotonic()
        self.last_upload_time = time.monotonic()
        self.last_speed_send_time = time.monotonic()

        # ================= ROS =================
        self.sub = self.create_subscription(
            Float32MultiArray,
            "/wheel_cmd_rpm",
            self.cmd_callback,
            10
        )

        self.pub_ticks_total = self.create_publisher(
            Int32MultiArray,
            "/wheel_ticks_total",
            10
        )

        self.pub_ticks = self.create_publisher(
            Int32MultiArray,
            "/wheel_ticks",
            10
        )

        self.connect_driver()

        self.timer = self.create_timer(0.01, self.loop)

        self.get_logger().info("motor_drive_node started")

    # ================= SERIAL =================
    def connect_driver(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=0.01)
        time.sleep(0.2)

        self.get_logger().info(f"Connected to driver on {self.port}")

        self.ser.write(b"$spd:0,0,0,0#")
        time.sleep(0.05)

        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        self.ser.write(b"$upload:1,0,0#")
        time.sleep(0.05)

    # ================= COMMAND =================
    def cmd_callback(self, msg):
        if len(msg.data) < 2:
           return

        self.left_cmd = int(msg.data[0])
        self.right_cmd = int(msg.data[1])

        self.get_logger().info(f"CMD RECEIVED: L={self.left_cmd} R={self.right_cmd}")
    # ================= SMOOTHING =================
    def apply_accel_limit(self, current, target, dt):
        delta = target - current
        max_delta = self.max_accel * dt

        if abs(delta) > max_delta:
            delta = max_delta if delta > 0 else -max_delta

        return current + delta

    # ================= SEND SPEED =================
    def send_speed(self, dt):

        # WATCHDOG
        if time.monotonic() - self.last_cmd_time > self.cmd_timeout:
            self.left_cmd_target = 0
            self.right_cmd_target = 0

        # APPLY ACCEL LIMIT
        self.left_cmd = self.apply_accel_limit(
            self.left_cmd, self.left_cmd_target, dt
        )
        self.right_cmd = self.apply_accel_limit(
            self.right_cmd, self.right_cmd_target, dt
        )
        self.get_logger().info(f"SENDING: {cmd}")

        # MOTOR MAPPING
        m1 = 0
        m2 = int(self.right_cmd)
        m3 = 0
        m4 = int(self.left_cmd)

        cmd = f"$spd:{m1},{m2},{m3},{m4}#"
        self.ser.write(cmd.encode())

    # ================= ENCODER =================
    def valid_packet(self, msg):
        pattern = r"^\$MAll:-?\d+,-?\d+,-?\d+,-?\d+#$"
        return re.match(pattern, msg) is not None

    def parse_mall(self, msg):
        if not self.valid_packet(msg):
            return

        try:
            body = msg.replace("$MAll:", "").replace("#", "")
            parts = body.split(",")

            right_raw = int(parts[1])
            left_raw = int(parts[3])

            if self.last_left_raw is None:
                self.last_left_raw = left_raw
                self.last_right_raw = right_raw
                return

            dl = left_raw - self.last_left_raw
            dr = right_raw - self.last_right_raw

            self.last_left_raw = left_raw
            self.last_right_raw = right_raw

            self.total_left += dl
            self.total_right += dr

            msg_tot = Int32MultiArray()
            msg_tot.data = [self.total_left, self.total_right]
            self.pub_ticks_total.publish(msg_tot)

            msg_d = Int32MultiArray()
            msg_d.data = [dl, dr]
            self.pub_ticks.publish(msg_d)

        except Exception as e:
            self.get_logger().warn(f"Parse error: {e}")

    # ================= SERIAL READ =================
    def read_serial(self):
        try:
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting).decode(errors="ignore")
                self.buffer += data

            while True:
                start = self.buffer.find("$MAll:")
                if start == -1:
                    self.buffer = self.buffer[-16:]
                    break

                end = self.buffer.find("#", start)
                if end == -1:
                    self.buffer = self.buffer[start:]
                    break

                msg = self.buffer[start:end + 1]
                self.buffer = self.buffer[end + 1:]

                self.parse_mall(msg)

        except Exception as e:
            self.get_logger().error(f"Serial error: {e}")

    # ================= MAIN LOOP =================
    def loop(self):
        now = time.monotonic()

        self.read_serial()

        if now - self.last_upload_time > 1.0:
            self.ser.write(b"$upload:1,0,0#")
            self.last_upload_time = now

        dt = 0.05
        if now - self.last_speed_send_time > dt:
            self.send_speed(dt)
            self.last_speed_send_time = now

    def destroy_node(self):
        try:
            self.ser.write(b"$spd:0,0,0,0#")
            self.ser.close()
        except Exception:
            pass

        super().destroy_node()


def main():
    rclpy.init()
    node = MotorDriveNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
