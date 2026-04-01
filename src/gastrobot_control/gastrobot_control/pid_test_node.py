import rclpy
from rclpy.node import Node
import serial
import time
import csv

class PIDTestNode(Node):

    def __init__(self):
        super().__init__('pid_test_node')

        # ===== SERIAL =====
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)

        # ===== DEFAULT TEST PARAMS =====
        self.test_duration = 6.0

        self.kp = 1.2
        self.ki = 0.0
        self.kd = 0.05

        self.bias = 2.3   # LOCKED from your test
        self.target_rpm = 30.0

        self.test_id = 0

        self.get_logger().info("PID TEST NODE READY")

        self.run_tests()

    # =========================
    def send(self, cmd):
        print(f"TX: {cmd}")
        self.ser.write((cmd + '\n').encode())
        time.sleep(0.05)

    # =========================
    def run_tests(self):
        while True:

            # ===== INTERACTIVE INPUT =====
            print("\n=== NEW TEST SETUP ===")

            kp = input(f"Kp ({self.kp}): ")
            ki = input(f"Ki ({self.ki}): ")
            kd = input(f"Kd ({self.kd}): ")
            rpm = input(f"Target RPM ({self.target_rpm}): ")

            if kp: self.kp = float(kp)
            if ki: self.ki = float(ki)
            if kd: self.kd = float(kd)
            if rpm: self.target_rpm = float(rpm)

            input("\n👉 Place robot at start and press ENTER...")

            self.test_id += 1
            filename = f"/home/gastrobot/test_{self.test_id}.csv"

            self.get_logger().info(f"Starting Test #{self.test_id}")

            # ===== HARD RESET SEQUENCE =====
            self.send("CMD,0,0")
            time.sleep(0.5)

            self.send("RST")
            time.sleep(0.5)

            self.send(f"PID,{self.kp},{self.ki},{self.kd}")
            time.sleep(0.1)

            self.send(f"BIAS,{self.bias}")
            time.sleep(0.5)

            # ===== START MOTION =====
            self.send(f"CMD,{self.target_rpm},{self.target_rpm}")

            # allow ramp-up
            time.sleep(1.0)

            start_time = time.time()
            data = []

            # ===== LOG LOOP =====
            while time.time() - start_time < self.test_duration:
                line = self.ser.readline().decode(errors='ignore').strip()

                if line:
                    print(line)

                if line.startswith("DBG"):
                    parts = line.split(',')
                    if len(parts) == 10:
                        data.append(parts)

            # ===== STOP =====
            self.send("CMD,0,0")

            self.get_logger().info("Test complete. Saving data...")

            # ===== SAVE CSV =====
            with open(filename, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    "time", "target_l", "target_r",
                    "rpm_l", "rpm_r",
                    "pwm_l", "pwm_r",
                    "error_l", "error_r"
                ])
                writer.writerows(data)

            self.get_logger().info(f"Saved: {filename}")

            # ===== CONTINUE =====
            cont = input("Run another test? (y/n): ")
            if cont.lower() != 'y':
                break


def main(args=None):
    rclpy.init(args=args)
    node = PIDTestNode()
    rclpy.shutdown()
