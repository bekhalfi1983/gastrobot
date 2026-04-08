from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    bringup_share = get_package_share_directory('gastrobot_bringup')

    joystick_config = os.path.join(bringup_share, 'config', 'joystick.yaml')
    slam_params_file = os.path.join(bringup_share, 'config', 'slam_params.yaml')
    ekf_params_file = os.path.join(bringup_share, 'config', 'ekf.yaml')

    # ================================
    # LIDAR (RPLIDAR C1)
    # ================================
    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar_c1_launch.py'
            )
        ),
        launch_arguments={
            'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_a09f9f7feb73ef11b0c8ca8c8fcc3fa0-if00-port0',
            'serial_baudrate': '460800'
        }.items()
    )

    # ================================
    # IMU NODE (BNO08X UART)
    # ================================
    imu_node = Node(
        package='gastrobot_imu',
        executable='bno08x_uart_node',
        name='imu_node',
        output='screen',
        parameters=[{
            'port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
            'baudrate': 115200,
            'frame_id': 'imu_link'
        }]
    )

    return LaunchDescription([

        # ================================
        # ARGUMENTS
        # ================================
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false'
        ),

        # ================================
        # DISPLAY
        # ================================
        SetEnvironmentVariable('DISPLAY', ':0'),

        # ================================
        # IMU NODE
        # ================================
        imu_node,

        # ================================
        # ESP32 MOTOR BRIDGE
        # ================================
        Node(
            package='gastrobot_control',
            executable='esp32_bridge_node',
            name='esp32_bridge_node',
            output='screen',
            parameters=[{
                'port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_a67f795ee971f0119cc3f99e1045c30f-if00-port0',
                'baudrate': 115200
            }]
        ),
        Node(
            package='gastrobot_control',
            executable='diff_drive_controller',
            name='diff_drive_controller',
            output='screen',
            remappings=[
                    ('/cmd_vel', '/cmd_vel_joy')
                ]
        ),

        # ================================
        # EKF
        # ================================
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params_file]
        ),

        # ================================
        # JOYSTICK INPUT
        # ================================
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[joystick_config, {'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # ================================
        # TELEOP
        # ================================
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joystick_config, {'use_sim_time': use_sim_time}],
            remappings=[
                ('/cmd_vel', '/cmd_vel_joy')
            ],
            output='screen'
        ),

        # ================================
        # STATIC TF (BASE -> LIDAR)
        # ================================
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
        ),

        # ================================
        # LIDAR
        # ================================
        rplidar,

        # ================================
        # SLAM TOOLBOX
        # ================================
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # ================================
        # VELOCITY SMOOTHER
        # ================================
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
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
                ('cmd_vel', '/cmd_vel_joy'),
                ('cmd_vel_smoothed', '/cmd_vel')
            ],
            output='screen'
        ),

        # ================================
        # LIFT NODE
        # ================================
        Node(
            package='gastrobot_control',
            executable='lift_node',
            name='lift_node',
            output='screen'
        ),

        # ================================
        # UI (WEB)
        # ================================
        Node(
            package='gastrobot_control',
            executable='ui_node',
            name='ui_node',
            output='screen'
        ),

        # ================================
        # GUI (LOCAL DISPLAY)
        # ================================
        Node(
            package='gastrobot_gui',
            executable='gui_node',
            name='gui_node',
            output='screen'
        ),
    ])
