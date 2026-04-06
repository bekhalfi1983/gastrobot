import threading
import rclpy
import uvicorn
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, JSONResponse
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool

class GastrobotUiBridge(Node):
    def __init__(self):
        super().__init__("gastrobot_ui_bridge")

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.navigate_pub = self.create_publisher(String, "/navigate_to_table", 10)
        self.go_home_pub = self.create_publisher(Bool, "/go_home", 10)
        self.cancel_nav_pub = self.create_publisher(Bool, "/cancel_navigation", 10)
        self.estop_pub = self.create_publisher(Bool, "/estop", 10)
        self.lift_pub = self.create_publisher(String, "/lift_cmd", 10)
        self.mock_pub = self.create_publisher(String, "/gastrobot/mock_mode", 10)
        self.slam_pub = self.create_publisher(String, "/gastrobot/slam_mode", 10)
        self.ui_command_pub = self.create_publisher(String, "/gastrobot/ui_command", 10)

        self.state = {
            "nav_status": "Autonomous Ready",
            "last_response": "Saved map loaded. Ready for travel.",
            "selected_table": "Table 1",
            "goal_name": "--",
            "distance_m": 0.0,
            "progress_pct": 0,
            "current_location": "--",
            "manual_mode": False,
            "mock_simulation": False,
            "connect_to_slam": False,
            "robot_state": "IDLE",
            "current_table": "-",
        }

        self.create_subscription(String, "/robot_state", self.robot_state_callback, 10)
        self.create_subscription(String, "/current_table", self.current_table_callback, 10)

    def robot_state_callback(self, msg):
        raw = msg.data.strip()
        low = raw.lower()
        self.state["robot_state"] = raw
        names = {
            "idle": "Autonomous Ready",
            "navigating": "Navigating",
            "at_table": "At Table",
            "serving": "Serving",
            "error": "Error",
            "estop": "E-Stop",
        }
        self.state["nav_status"] = names.get(low, raw)

        if low == "idle":
            self.state["progress_pct"] = 0
            self.state["distance_m"] = 0.0
            self.state["last_response"] = "Robot idle and ready"
        elif low == "navigating":
            self.state["progress_pct"] = max(self.state["progress_pct"], 5)
            self.state["last_response"] = f"Navigating to {self.state['goal_name']}"
        elif low == "at_table":
            self.state["progress_pct"] = 100
            self.state["distance_m"] = 0.0
            self.state["last_response"] = f"Arrived at {self.state['goal_name']}"
        elif low == "serving":
            self.state["progress_pct"] = 100
            self.state["distance_m"] = 0.0
            self.state["last_response"] = "Serving at destination"
        elif low == "error":
            self.state["last_response"] = "Navigation error"
        elif low == "estop":
            self.state["last_response"] = "Emergency stop active"

    def current_table_callback(self, msg):
        self.state["current_table"] = msg.data
        self.state["current_location"] = msg.data

    def publish_cmd_vel(self, x=0.0, z=0.0):
        msg = Twist()
        msg.linear.x = x
        msg.angular.z = z
        self.cmd_vel_pub.publish(msg)

    def publish_string(self, pub, text):
        msg = String()
        msg.data = text
        pub.publish(msg)

    def publish_bool(self, pub, value=True):
        msg = Bool()
        msg.data = value
        pub.publish(msg)

    def stop_robot(self):
        self.publish_cmd_vel(0.0, 0.0)
        self.publish_bool(self.cancel_nav_pub, True)

    def destination_to_backend_name(self, name):
        return {
            "Table 1": "table_1",
            "Table 2": "table_2",
            "Table 3": "table_3",
            "Return to Kitchen": "home",
        }.get(name, "table_1")

app = FastAPI()
ros_node = None

@app.on_event("startup")
def startup():
    global ros_node
    # Note: rclpy.init is now handled in main()
    ros_node = GastrobotUiBridge()
    threading.Thread(target=lambda: rclpy.spin(ros_node), daemon=True).start()

@app.on_event("shutdown")
def shutdown():
    global ros_node
    if ros_node:
        ros_node.destroy_node()

@app.get("/", response_class=HTMLResponse)
def home():
    # ... (Keep your existing HTML string here) ...
    return HTMLResponse("<html>...</html>") # Truncated for brevity

@app.get("/api/status")
def get_status():
    return JSONResponse(content=ros_node.state)

# ... (Include all your existing FastAPI @app.post routes here) ...

# ==========================================
# THE CRITICAL FIX: THE MAIN ENTRY POINT
# ==========================================
def main(args=None):
    rclpy.init(args=args)
    # Use uvicorn to run the app
    # host 0.0.0.0 allows access from other devices on the network
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")

if __name__ == "__main__":
    main()
