"""
Setup for carla_waypoint_publisher
"""
import os
from glob import glob
ROS_VERSION = int(os.environ['ROS_VERSION'])

from setuptools import setup

package_name = 'carla_waypoint_publisher'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CARLA Simulator Team',
    maintainer_email='carla.simulator@gmail.com',
    description='CARLA waypoint publisher for ROS2 bridge',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['carla_waypoint_publisher = carla_waypoint_publisher.carla_waypoint_publisher:main'],
    },
    package_dir={'': 'src'},
)
