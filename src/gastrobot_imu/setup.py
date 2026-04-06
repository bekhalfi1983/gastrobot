from setuptools import setup

package_name = 'gastrobot_imu'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'bno08x_node = gastrobot_imu.bno08x_node:main',
            'bno08x_uart_node = gastrobot_imu.bno08x_uart_node:main',
        ],
    },
)
