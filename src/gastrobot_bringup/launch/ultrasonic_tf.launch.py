from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Arguments: [x, y, z, yaw, pitch, roll, parent, child]
    # Coordinates in meters, angles in radians
    
    return LaunchDescription([
        # Front Left (10 deg)
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='tf_fl', arguments=['0.37', '0.14', '0.1', '0.174', '0', '0', 'base_link', 'ultrasonic_front_left_link']),
        
        # Front Right (-10 deg)
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='tf_fr', arguments=['0.37', '-0.13', '0.1', '-0.174', '0', '0', 'base_link', 'ultrasonic_front_right_link']),
        
        # Back Left (165 deg)
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='tf_bl', arguments=['-0.14', '0.14', '0.1', '2.880', '0', '0', 'base_link', 'ultrasonic_back_left_link']),

        # Back Right (-160 deg)
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='tf_br', arguments=['-0.14', '-0.13', '0.1', '-2.793', '0', '0', 'base_link', 'ultrasonic_back_right_link']),

        # Left Side (70 deg)
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='tf_l', arguments=['0.15', '0.24', '0.1', '1.222', '0', '0', 'base_link', 'ultrasonic_left_side_link']),

        # Right Side (-75 deg)
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='tf_r', arguments=['0.16', '-0.24', '0.1', '-1.309', '0', '0', 'base_link', 'ultrasonic_right_side_link']),
    ])
