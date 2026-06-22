from setuptools import find_packages, setup
import os
from glob import glob

package_name = "carla_aeb_v2x_agent"

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
    description="V2X-aware AEB agent (lidar + CAM fusion) for CARLA VRU scenarios",
    license="MIT",
    entry_points={
        "console_scripts": [
            "carla_aeb_v2x_agent = carla_aeb_v2x_agent.aeb_v2x_node:main",
        ],
    },
)
