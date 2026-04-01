from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gastrobot_control',
            executable='wheel_odometry_node',
            name='wheel_odometry_node',
            output='screen'
        )
    ])
