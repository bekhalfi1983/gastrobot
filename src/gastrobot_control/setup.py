from setuptools import find_packages, setup

package_name = 'gastrobot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='youness',
    maintainer_email='youness@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
       'console_scripts': [
       'diff_drive_controller = gastrobot_control.diff_drive_controller:main',
       'motor_drive_node = gastrobot_control.motor_drive_node:main',
       'lift_node = gastrobot_control.lift_node:main',
       'wheel_odometry_node = gastrobot_control.wheel_odometry_node:main',
       'imu_node = gastrobot_control.imu_node:main',
       'lift_serial_node = gastrobot_control.lift_serial_node:main',
       'ultrasonic_node = gastrobot_control.ultrasonic_node:main',
       'esp32_bridge_node = gastrobot_control.esp32_bridge_node:main',
       'joystick_control_node = gastrobot_control.joystick_control_node:main',   
       'gui_node = gastrobot_control.gui_node:main',
       'pid_tuner_node = gastrobot_control.pid_tuner_node:main',
       'pid_test_node = gastrobot_control.pid_test_node:main',
       ],
    },

)
