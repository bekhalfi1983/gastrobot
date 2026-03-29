import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist

import time
import math
import lgpio

# ================= CONFIG =================

TRIG = 23

SENSORS = {
    "front_left": 17,
    "front_right": 27,
    "back_left": 22,
    "back_right": 5,
    "left_side": 6,
    "right_side": 13
}

SENSOR_ORDER = list(SENSORS.keys())

SPEED_CM_PER_US = 0.0343
TIMEOUT = 0.02
TRIG_PULSE = 30e-6
THRESHOLD = 0.40  # meters


# ================= NODE =================

class UltrasonicNode(Node):

    def __init__(self):
        super().__init__('ultrasonic_node')

        # Robot motion state
        self.vx = 0.0
        self.wz = 0.0

        # Publishers
        self.range_pub = self.create_publisher(
            Float32MultiArray,
            '/gastrobot/ultrasonic/ranges',
            10
        )

        self.detect_pub = self.create_publisher(
            String,
            '/gastrobot/ultrasonic/detected',
            10
        )

        # Subscribe to velocity
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        # GPIO setup
        self.chip = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.chip, TRIG, 0)

        for pin in SENSORS.values():
            lgpio.gpio_claim_input(self.chip, pin)

        time.sleep(1.0)

        # Timer (10 Hz)
        self.timer = self.create_timer(0.1, self.read_selected)

        self.get_logger().info("Ultrasonic node READY")

    # ================= CMD =================

    def cmd_callback(self, msg):
        self.vx = msg.linear.x
        self.wz = msg.angular.z

    # ================= SENSOR SELECTION =================

    def get_active_sensors(self):
        active = set()

        if self.vx > 0.02:
            active.update(["front_left", "front_right"])

        elif self.vx < -0.02:
            active.update(["back_left", "back_right"])

        if self.wz > 0.2:
            active.add("left_side")

        if self.wz < -0.2:
            active.add("right_side")

        return active

    # ================= SAFE DISTANCE READ =================

    def read_distance(self, echo_pin):
        # Trigger pulse
        lgpio.gpio_write(self.chip, TRIG, 1)
        time.sleep(TRIG_PULSE)
        lgpio.gpio_write(self.chip, TRIG, 0)

        t0 = time.perf_counter()

        # Wait for HIGH
        while lgpio.gpio_read(self.chip, echo_pin) == 0:
            if time.perf_counter() - t0 > TIMEOUT:
                return float('nan')

        t1 = time.perf_counter()

        # Wait for LOW
        while lgpio.gpio_read(self.chip, echo_pin) == 1:
            if time.perf_counter() - t1 > TIMEOUT:
                return float('nan')

        t2 = time.perf_counter()

        duration = t2 - t1

        distance_cm = (duration * 1_000_000 * SPEED_CM_PER_US) / 2
        distance_m = distance_cm / 100

        if distance_m < 0.02 or distance_m > 4:
            return float('nan')

        return distance_m

    # ================= MAIN LOOP =================

    def read_selected(self):

        active = self.get_active_sensors()

        ranges = []
        detected = []

        for name in SENSOR_ORDER:
            pin = SENSORS[name]

            if name in active:
                d = self.read_distance(pin)

                if not math.isnan(d) and d < THRESHOLD:
                    detected.append(name)

                time.sleep(0.04)  # prevent cross-talk

            else:
                d = float('nan')

            ranges.append(d)

        # Publish ranges
        msg = Float32MultiArray()
        msg.data = ranges
        self.range_pub.publish(msg)

        # Publish detections
        det = String()
        det.data = "none" if not detected else ",".join(detected)
        self.detect_pub.publish(det)


# ================= MAIN =================

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()