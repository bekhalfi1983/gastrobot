import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode # Added LifecycleNode
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    bringup_share = get_package_share_directory('gastrobot_bringup')
    
    # Paths
    joystick_config = os.path.join(bringup_share, 'config', 'joystick.yaml')
    ekf_params_file = os.path.join(bringup_share, 'config', 'ekf.yaml')
    urdf_file = os.path.join(bringup_share, 'urdf', 'gastrobot.urdf')
    
    with open(urdf_file, 'r') as infp:
        robot_description_config = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        SetEnvironmentVariable('DISPLAY', ':0'),

        # 1. Skeleton & Localization
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description_config, 'use_sim_time': use_sim_time}]),
#        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
 #            parameters=[ekf_params_file, {'use_sim_time': use_sim_time}]),

        # 2. Hardware (BNO08X & ESP32) - KEPT YOUR ORIGINAL PORTS
        Node(package='gastrobot_imu', executable='bno08x_uart_node', name='imu_node',
             parameters=[{'port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
                          'baudrate': 115200, 'frame_id': 'imu_link'}]),
        Node(package='gastrobot_control', executable='esp32_bridge_node', name='esp32_bridge_node',
             parameters=[{'port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_a67f795ee971f0119cc3f99e1045c30f-if00-port0',
                          'baudrate': 115200}]),

        # 3. Motor Controller - (Listening to SMOOTHED /cmd_vel)
        Node(package='gastrobot_control', executable='diff_drive_controller', name='diff_drive_controller'),


        # 4. Joystick & Teleop - (Publishing to /cmd_vel_joy)
        Node(package='joy', executable='joy_node', name='joy_node', parameters=[joystick_config]),
        Node(package='teleop_twist_joy', executable='teleop_node', name='teleop_node',
             parameters=[joystick_config], remappings=[('/cmd_vel', '/cmd_vel_joy')]),

        # 5. VELOCITY SMOOTHER (The "Smooth" Fix)
        LifecycleNode(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            namespace='',
            output='screen',
            parameters=[{
                'max_velocity': [0.45, 0.0, 1.2],
                'min_velocity': [-0.45, 0.0, -1.2],
                'max_accel': [0.25, 0.0, 1.0],
                'max_decel': [-0.35, 0.0, -1.5],
                'deadband_velocity': [0.03, 0.0, 0.03],
                'velocity_timeout': 0.3,
                'feedback': 'OPEN_LOOP',
                'use_sim_time': use_sim_time
            }],
            remappings=[
                ('cmd_vel', '/cmd_vel_joy'),      # Input from Teleop
                ('cmd_vel_smoothed', '/cmd_vel')  # Output to Motors
            ]
        ),

        # 6. UI & Extra Nodes
        Node(package='gastrobot_control', executable='lift_node', name='lift_node'),
        Node(package='gastrobot_control', executable='ui_node', name='ui_node'),
        Node(package='gastrobot_gui', executable='gui_node', name='gui_node'),
    ])
