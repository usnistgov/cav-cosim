from setuptools import find_packages, setup

package_name = 'carla_traffic_monitor'

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
    maintainer='iotav',
    maintainer_email='hajjaj.hadhoum@gmail.com',
    description='onitor and process traffic lights in CARLA simulation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'traffic_light_monitor = carla_traffic_monitor.traffic_light_monitor:main',
            'traffic_light_sender = carla_traffic_monitor.ros2_ns3_tcp_sender:main',
        ],
    },
)
