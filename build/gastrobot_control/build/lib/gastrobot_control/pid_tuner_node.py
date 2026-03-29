#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import time
import threading
import csv
import os


class PIDTunerNode(Node):

    def __init__(self):
        super().__init__('pid_tuner_node')

        # ===== PARAMETERS =====
        self.port = '/dev/ttyUSB1'   # adjust if needed
        self.baud = 115200

        self.csv_path = '/home/gastrobot/pid_logs/test_log.csv'

        # ===== SERIAL =====
        self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        self.get_logger().info(f"Connected to {self.port}")

        # ===== CSV LOGGING =====
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)

        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        self.csv_writer.writerow([
            "pc_time",
            "esp_time",
            "target_l",
            "target_r",
            "measured_l",
            "measured_r",
            "pwm_l",
            "pwm_r",
            "error_l",
            "error_r"
        ])

        # ===== SERIAL THREAD =====
        self.thread = threading.Thread(target=self.read_serial, daemon=True)
        self.thread.start()

        # ===== START TEST =====
        time.sleep(2)
        threading.Thread(target=self.run_tests, daemon=True).start()

    # ================= SERIAL =================

    def send(self, msg):
        full = msg + '\n'
        self.ser.write(full.encode())
        self.get_logger().info(f"TX: {msg}")

    # ================= READ =================

    def read_serial(self):
        while rclpy.ok():
            try:
                line = self.ser.readline().decode(errors='ignore').strip()
                if not line:
                    continue

                if line.startswith("DBG,"):
                    self.handle_dbg(line)
                else:
                    self.get_logger().info(f"ESP: {line}")

            except Exception as e:
                self.get_logger().error(f"Serial error: {e}")

    # ================= PARSE DBG =================

    def handle_dbg(self, line):
        parts = line.split(',')

        if len(parts) != 10:
            return

        _, t, tl, tr, ml, mr, pl, pr, el, er = parts

        # log to CSV
        self.csv_writer.writerow([
            time.time(),
            float(t),
            float(tl),
            float(tr),
            float(ml),
            float(mr),
            float(pl),
            float(pr),
            float(el),
            float(er)
        ])
        self.csv_file.flush()

    # ================= TEST SEQUENCE =================

    def run_tests(self):

        tests = [
            {"kp": 0.4, "ki": 0.0, "kd": 0.0},
            {"kp": 0.6, "ki": 0.0, "kd": 0.0},
            {"kp": 0.8, "ki": 0.0, "kd": 0.05},
            {"kp": 0.8, "ki": 0.01, "kd": 0.05},
        ]

        for i, t in enumerate(tests):

            self.get_logger().info(f"===== TEST {i} =====")

            # Label in CSV
            self.csv_writer.writerow([f"TEST {i}", t])
            self.csv_file.flush()

            # Set PID
            self.send(f"PID,{t['kp']},{t['ki']},{t['kd']}")
            time.sleep(1)

            # Reset integrator
            self.send("RST")
            time.sleep(0.2)

            # ===== FORWARD =====
            self.send("CMD,15,15")
            time.sleep(2.5)

            # Stop
            self.send("CMD,0,0")
            time.sleep(1.5)

            # Reset again
            self.send("RST")
            time.sleep(0.2)

            # ===== REVERSE =====
            self.send("CMD,-15,-15")
            time.sleep(2.5)

            # Stop
            self.send("CMD,0,0")
            time.sleep(2.0)

        self.get_logger().info("ALL TESTS COMPLETE")

    # ================= CLEANUP =================

    def destroy_node(self):
        try:
            self.send("CMD,0,0")
        except:
            pass

        self.csv_file.close()
        self.ser.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = PIDTunerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
