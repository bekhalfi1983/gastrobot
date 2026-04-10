import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import serial
import threading

class GastrobotBridgeNode(Node):

    def __init__(self):
        super().__init__('lift_node')

        # 1. SERIAL PORT - Connecting to your Nano
        # Ensure your Nano is actually on ACM0 (run 'ls /dev/ttyACM*')
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        except Exception as e:
            self.get_logger().error(f"Failed to open Serial Port: {e}")

        # 2. ROS TOPICS - LIFT
        self.command_sub = self.create_subscription(
            String,
            '/lift_cmd',
            self.command_callback,
            10
        )
        self.lift_state_pub = self.create_publisher(String, '/lift/state', 10)

        # 3. ROS TOPICS - BATTERY
        self.volt_pub = self.create_publisher(Float32, '/battery_voltage', 10)
        self.perc_pub = self.create_publisher(Float32, '/battery_percentage', 10)

        # 4. START SERIAL THREAD
        self.serial_thread = threading.Thread(target=self.read_serial)
        self.serial_thread.daemon = True
        self.serial_thread.start()

        self.get_logger().info("Gastrobot Hardware Bridge (Lift + Battery) Started")

    def command_callback(self, msg):
        """
        Maps ROS string commands to your new Arduino single-char commands:
        'up'    -> 'u'
        'down'  -> 'd'
        'stop'  -> 's'
        'home'  -> 'h'
        """
        raw_cmd = msg.data.lower().strip()
        
        mapping = {
            "up": "u",
            "down": "d",
            "stop": "s",
            "home": "h"
        }

        arduino_cmd = mapping.get(raw_cmd, raw_cmd) # Use map, fallback to raw

        try:
            self.ser.write((arduino_cmd + "\n").encode())
            self.get_logger().info(f"Sent to Arduino: {arduino_cmd}")
        except Exception as e:
            self.get_logger().error(f"Serial write error: {e}")

    def read_serial(self):
        while rclpy.ok():
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8').strip()
                    
                    if not line:
                        continue

                    # PARSE BATTERY DATA: "BAT,12.45,85.0"
                    if line.startswith("BAT,"):
                        parts = line.split(",")
                        if len(parts) == 3:
                            voltage = float(parts[1])
                            percentage = float(parts[2])

                            # Publish as floats for ROS 2 logic/UI
                            v_msg = Float32()
                            v_msg.data = voltage
                            self.volt_pub.publish(v_msg)

                            p_msg = Float32()
                            p_msg.data = percentage
                            self.perc_pub.publish(p_msg)

                    # PARSE MESSAGES: "MSG,HOME_COMPLETE"
                    elif line.startswith("MSG,") or "LIMIT" in line:
                        msg = String()
                        msg.data = line
                        self.lift_state_pub.publish(msg)
                        self.get_logger().info(f"Arduino Status: {line}")
                    
                    else:
                        # Fallback for any other debug lines
                        self.get_logger().debug(f"Arduino Debug: {line}")

            except Exception as e:
                # Silently catch decode errors from serial noise
                pass

def main(args=None):
    rclpy.init(args=args)
    node = GastrobotBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
