from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([

        # ================================
        # DISPLAY (FOR GUI)
        # ================================
        SetEnvironmentVariable('DISPLAY', ':0'),

        # ================================
        # DIFF DRIVE CONTROLLER
        # ================================
        Node(
            package='gastrobot_control',
            executable='diff_drive_controller',
            name='diff_drive_controller',
            output='screen'
        ),

        # ================================
        # ESP32 MOTOR BRIDGE
        # ================================
        Node(
            package='gastrobot_control',
            executable='esp32_bridge_node',
            name='esp32_bridge_node',
            output='screen',
            parameters=[{
                "port": "/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_a67f795ee971f0119cc3f99e1045c30f-if00-port0",
                "baud": 115200
            }]
        ),

        # ================================
        # LIFT (ARDUINO)
        # ================================
        Node(
            package='gastrobot_control',
            executable='lift_node',
            name='lift_node',
            output='screen',
            parameters=[{
                "port": "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_9503230373735171A2E1-if00",
                "baud": 115200
            }]
        ),

        # ================================
        # GUI NODE (FULLSCREEN)
        # ================================
        Node(
            package='gastrobot_gui',
            executable='gui_node',
            name='gui_node',
            output='screen'
        ),

        # ================================
        # JOYSTICK INPUT
	# ================================
	Node(
	    package='joy',
	    executable='joy_node',
	    name='joy_node',
	    output='screen'
	),

	# ================================
	# YOUR CUSTOM NODE
	# ================================
	Node(
	    package='gastrobot_control',
	    executable='joystick_control_node',
	    name='joystick_control_node',
	    output='screen'
	),
	Node(
	    package='gastrobot_control',
	    executable='imu_node',
	    name='imu_node',
	    output='screen'
	),

	Node(
	    package='gastrobot_control',
	    executable='wheel_odometry_node',
	    name='wheel_odometry_node',
	    output='screen'
	),
	# ================= EKF (DELAYED) =================
	        TimerAction(
	            period=3.0,
	            actions=[
	                Node(
	                    package='robot_localization',
	                    executable='ekf_node',
	                    name='ekf_node',
	                    output='screen',
	                    parameters=[
	                        '/home/gastrobot/gastrobot_ws/src/gastrobot_control/config/ekf.yaml'
	                    ]
	                )
	            ]
	        ),

    ])
