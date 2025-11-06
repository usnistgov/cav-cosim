from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'carla_traffic_light_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hnh21',
    maintainer_email='hnh21@example.com',
    description='ROS2 Traffic Light Controller for CARLA simulation with V2I communication',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'traffic_light_controller_node = carla_traffic_light_controller.traffic_light_controller_node:main',
            'perception_controller_node = carla_traffic_light_controller.perception_controller_node:main',
        ],
    },
    package_dir={'': '.'},
)