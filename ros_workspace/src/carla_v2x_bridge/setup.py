from setuptools import find_packages, setup

package_name = "carla_v2x_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="VRU Research",
    maintainer_email="research@iotav.org",
    description="CARLA-to-ns-3 V2P cosim bridge",
    license="MIT",
    entry_points={
        "console_scripts": [
            "carla_v2x_bridge = carla_v2x_bridge.v2x_bridge_node:main",
        ],
    },
)
