#!/usr/bin/env python3

import math
import threading
import time
from typing import Optional

import serial
from serial import SerialException

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist


def safe_float(value: str) -> Optional[float]:
    try:
        return float(value)
    except Exception:
        return None


def safe_int(value: str) -> Optional[int]:
    try:
        return int(value)
    except Exception:
        return None


class ESP32BridgeNode(Node):
    def __init__(self):
        super().__init__('esp32_bridge_node')

        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('baud', 115200)  # legacy fallback
        self.declare_parameter('wheel_radius', 0.072)
        self.declare_parameter('wheel_base', 0.56)
        self.declare_parameter('read_chunk_size', 256)
        self.declare_parameter('reconnect_interval', 1.0)
        self.declare_parameter('max_buffer_chars', 4096)
        self.declare_parameter('debug_bad_packets', False)

        self.port = str(self.get_parameter('port').value)

        # Support both "baudrate" and legacy "baud"
        baudrate_param = int(self.get_parameter('baudrate').value)
        baud_param = int(self.get_parameter('baud').value)
        self.baudrate = baudrate_param if baudrate_param > 0 else baud_param

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.read_chunk_size = int(self.get_parameter('read_chunk_size').value)
        self.reconnect_interval = float(self.get_parameter('reconnect_interval').value)
        self.max_buffer_chars = int(self.get_parameter('max_buffer_chars').value)
        self.debug_bad_packets = bool(self.get_parameter('debug_bad_packets').value)

        # Serial state
        self.ser: Optional[serial.Serial] = None
        self.serial_lock = threading.Lock()
        self.last_connect_attempt = 0.0
        self.text_buffer = ''

        # Diagnostics
        self.bad_packet_count = 0
        self.good_packet_count = 0
        self.last_bad_log_time = 0.0
        self.last_cmd_write_error_time = 0.0

        # Publishers
        self.tick_pub = self.create_publisher(Int32MultiArray, '/wheel_ticks', 10)

        # Subscribers
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Timer
        self.timer = self.create_timer(0.02, self.read_serial)

        self.get_logger().info('=== ESP32 BRIDGE NODE RUNNING ===')
        self.connect_serial(force=True)

    # ---------------- Serial helpers ----------------

    def connect_serial(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force and (now - self.last_connect_attempt) < self.reconnect_interval:
            return

        self.last_connect_attempt = now
        self.close_serial()

        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.01,
                write_timeout=0.05,
                exclusive=True,
            )
            self.text_buffer = ''
            self.get_logger().info(f'ESP32 CONNECTED on {self.port} @ {self.baudrate}')
        except Exception as e:
            self.ser = None
            self.get_logger().error(f'Serial open failed: {e}')

    def close_serial(self) -> None:
        with self.serial_lock:
            if self.ser is not None:
                try:
                    if self.ser.is_open:
                        self.ser.close()
                except Exception:
                    pass
                finally:
                    self.ser = None

    def mark_serial_unhealthy(self, reason: str) -> None:
        self.get_logger().error(reason)
        self.close_serial()

    # ---------------- Cmd_vel -> ESP32 ----------------

    def cmd_vel_callback(self, msg: Twist) -> None:
        if self.ser is None:
            return

        # --- ORIENTATION CONTROL ---
        # If the robot drives backward physically, add a minus sign: -float(msg.linear.x)
        v = float(msg.linear.x)
        w = float(msg.angular.z)

        v_l = v - (w * self.wheel_base / 2.0)
        v_r = v + (w * self.wheel_base / 2.0)

        rpm_l = (v_l / (2.0 * math.pi * self.wheel_radius)) * 60.0
        rpm_r = (v_r / (2.0 * math.pi * self.wheel_radius)) * 60.0

        line = f'CMD,{rpm_l:.2f},{rpm_r:.2f}\n'.encode('utf-8')

        try:
            with self.serial_lock:
                if self.ser is None:
                    return
                self.ser.write(line)
        except (SerialException, OSError) as e:
            now = time.monotonic()
            if now - self.last_cmd_write_error_time > 1.0:
                self.last_cmd_write_error_time = now
                self.get_logger().error(f'CMD write failed: {e}')
            self.close_serial()
        except Exception as e:
            now = time.monotonic()
            if now - self.last_cmd_write_error_time > 1.0:
                self.last_cmd_write_error_time = now
                self.get_logger().error(f'Unexpected CMD write error: {e}')
            self.close_serial()

    # ---------------- Serial parsing ----------------

    def parse_and_publish_data_line(self, raw_line: str) -> None:
        line = raw_line.strip()
        if not line or not line.startswith('DATA,'):
            return

        parts = [p.strip() for p in line.split(',')]

        if len(parts) < 12:
            self.bad_packet_count += 1
            self.maybe_log_bad_packet(f'Short DATA packet ({len(parts)} fields): {line}')
            return

        left_ticks = safe_int(parts[8])
        right_ticks = safe_int(parts[9])

        if left_ticks is None or right_ticks is None:
            self.bad_packet_count += 1
            self.maybe_log_bad_packet(f'Bad DATA: {line}')
            return

        tick_msg = Int32MultiArray()
        # --- ODOMETRY SYNC ---
        # If the robot icon in RViz moves backward while the robot moves forward, 
        # add minus signs: [-left_ticks, -right_ticks]
        tick_msg.data = [left_ticks, right_ticks]
        self.tick_pub.publish(tick_msg)

        self.good_packet_count += 1

    def maybe_log_bad_packet(self, message: str) -> None:
        now = time.monotonic()
        if self.debug_bad_packets or (now - self.last_bad_log_time > 1.0):
            self.last_bad_log_time = now
            self.get_logger().debug(message)

    def process_text_buffer(self) -> None:
        self.text_buffer = self.text_buffer.replace('\r', '\n')
        extracted = []

        while True:
            start = self.text_buffer.find('DATA,')
            if start < 0:
                if len(self.text_buffer) > self.max_buffer_chars:
                    self.text_buffer = self.text_buffer[-256:]
                break

            if start > 0:
                self.text_buffer = self.text_buffer[start:]

            nl = self.text_buffer.find('\n', 5)
            nxt = self.text_buffer.find('DATA,', 5)

            if nl == -1 and nxt == -1:
                if len(self.text_buffer) > self.max_buffer_chars:
                    self.text_buffer = self.text_buffer[-512:]
                break

            candidates = [x for x in (nl, nxt) if x != -1]
            end = min(candidates)

            frame = self.text_buffer[:end].strip()
            self.text_buffer = self.text_buffer[end:]

            if self.text_buffer.startswith('\n'):
                self.text_buffer = self.text_buffer[1:]

            if frame.startswith('DATA,'):
                extracted.append(frame)

        for frame in extracted:
            self.parse_and_publish_data_line(frame)

    def read_serial(self) -> None:
        if self.ser is None:
            self.connect_serial()
            return

        try:
            with self.serial_lock:
                if self.ser is None:
                    return
                waiting = self.ser.in_waiting
                if waiting <= 0:
                    return
                chunk = self.ser.read(min(waiting, self.read_chunk_size))

            if not chunk:
                return

            decoded = chunk.decode('utf-8', errors='ignore')
            if not decoded:
                return

            self.text_buffer += decoded
            if len(self.text_buffer) > self.max_buffer_chars:
                self.text_buffer = self.text_buffer[-1024:]

            self.process_text_buffer()

        except (SerialException, OSError) as e:
            self.mark_serial_unhealthy(f'Serial read error: {e}')
        except Exception as e:
            self.get_logger().warn(f'Unexpected serial parse/read issue: {e}')

    def destroy_node(self):
        self.close_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ESP32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
