from setuptools import find_packages, setup
import os
from glob import glob

package_name = "carla_vru_demo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="VRU Research",
    maintainer_email="research@iotav.org",
    description="Bringup / launch files for CARLA VRU safety experiments",
    license="MIT",
    entry_points={"console_scripts": []},
)
