from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    bringup_share = get_package_share_directory('gastrobot_bringup')
    slam_params_file = os.path.join(bringup_share, 'config', 'slam_params.yaml')

    # ================================
    # LIDAR
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
            'serial_port': '/dev/ttyUSB1',
            'serial_baudrate': '460800'
        }.items()
    )

    return LaunchDescription([

        # ================================
        # DISPLAY (FOR GUI ON PI)
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
                "port": "/dev/ttyUSB0",
                "baud": 115200
            }]
        ),

        # ================================
        # LIFT NODE
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
        # GUI
        # ================================
        Node(
            package='gastrobot_gui',
            executable='gui_node',
            name='gui_node',
            output='screen'
        ),

        # ================================
        # JOYSTICK
        # ================================
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        # ================================
        # JOYSTICK CONTROL
        # ================================
        Node(
            package='gastrobot_control',
            executable='joystick_control_node',
            name='joystick_control_node',
            output='screen'
        ),

        # ================================
        # WHEEL ODOMETRY
        # publishes /odom and odom->base_link TF
        # ================================
        Node(
            package='gastrobot_control',
            executable='wheel_odometry_node',
            name='wheel_odometry_node',
            output='screen'
        ),

        # ================================
        # STATIC TF: base_link -> laser
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
            output='screen',
            parameters=[slam_params_file]
        ),
    ])
