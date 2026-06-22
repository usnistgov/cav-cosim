from setuptools import find_packages, setup
import os
from glob import glob

package_name = "carla_camera_lidar_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="VRU Research",
    maintainer_email="research@iotav.org",
    description="Camera + LiDAR fusion perception for VRU detection in CARLA",
    license="MIT",
    entry_points={
        "console_scripts": [
            "camera_lidar_perception = carla_camera_lidar_perception.camera_lidar_perception_node:main",
        ],
    },
)
