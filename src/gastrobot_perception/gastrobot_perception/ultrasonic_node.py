import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import time
import math
import lgpio

SENSORS = {
    "front_left":  {"pins": (17, 27), "frame": "ultrasonic_front_left_link"},
    "front_right": {"pins": (22, 5),  "frame": "ultrasonic_front_right_link"},
    "back_left":   {"pins": (6, 13),  "frame": "ultrasonic_back_left_link"},
    "back_right":  {"pins": (19, 26), "frame": "ultrasonic_back_right_link"},
    "left_side":   {"pins": (21, 20), "frame": "ultrasonic_left_side_link"},
    "right_side":  {"pins": (16, 12), "frame": "ultrasonic_right_side_link"}
}

class UltrasonicFusionNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_fusion_node')
        
        # Dictionary to hold publishers for each sensor
        self.pubs = {}
        for name, info in SENSORS.items():
            self.pubs[name] = self.create_publisher(Range, f'/ultrasonic/{name}', 10)

        self.chip = lgpio.gpiochip_open(0)
        for name, info in SENSORS.items():
            lgpio.gpio_claim_output(self.chip, info["pins"][0])
            lgpio.gpio_claim_input(self.chip, info["pins"][1])

        self.timer = self.create_timer(0.1, self.read_sequence) # 10Hz total
        self.get_logger().info("Ultrasonic Fusion Node: Range Messages ACTIVE")

    def read_sequence(self):
        now = self.get_clock().now().to_msg()
        for name, info in SENSORS.items():
            dist = self.get_distance(info["pins"][0], info["pins"][1])
            
            # Create the standard Range message Nav2 expects
            msg = Range()
            msg.header.stamp = now
            msg.header.frame_id = info["frame"]
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = 0.26 # ~15 degrees in radians
            msg.min_range = 0.02
            msg.max_range = 4.0
            
            if math.isnan(dist) or dist > 4.0:
                msg.range = float('inf') # Nav2 treats inf as "clear"
            else:
                msg.range = dist
            
            self.pubs[name].publish(msg)
            time.sleep(0.02) # Prevent cross-talk

    def get_distance(self, trig, echo):
        lgpio.gpio_write(self.chip, trig, 1)
        time.sleep(0.00001)
        lgpio.gpio_write(self.chip, trig, 0)

        start = time.perf_counter()
        while lgpio.gpio_read(self.chip, echo) == 0:
            if time.perf_counter() - start > 0.02: return float('nan')
        
        t_start = time.perf_counter()
        while lgpio.gpio_read(self.chip, echo) == 1:
            if time.perf_counter() - t_start > 0.02: return float('nan')
        
        return ((time.perf_counter() - t_start) * 343.0) / 2

def main():
    rclpy.init()
    node = UltrasonicFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        lgpio.gpiochip_close(node.chip)
        node.destroy_node()
        rclpy.shutdown()
