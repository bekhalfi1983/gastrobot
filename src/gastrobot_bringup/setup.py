from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gastrobot_bringup'

def get_data_files():
    # 1. Start with the standard ROS 2 files
    data_files = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ]
    
    # 2. Automatically handle the config folder and its subfolders
    for root, dirs, files in os.walk('config'):
        # Calculate the destination path in the install directory
        # e.g., 'share/gastrobot_bringup/config/nav2'
        install_path = os.path.join('share', package_name, root)
        
        # Find all .yaml files in the current root
        yaml_files = glob(os.path.join(root, '*.yaml'))
        
        if yaml_files:
            data_files.append((install_path, yaml_files))
            
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']), # Better for multiple sub-packages
    data_files=get_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gastrobot',
    maintainer_email='gastrobot@todo.todo',
    description='Gastrobot bringup package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
