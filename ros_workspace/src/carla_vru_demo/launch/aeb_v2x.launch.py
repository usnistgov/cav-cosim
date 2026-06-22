"""
Launch the V2X stack — V2X-aware AEB + bridge + ns-3 gateway in one command.

Usage:
    ros2 launch carla_vru_demo aeb_v2x.launch.py
    ros2 launch carla_vru_demo aeb_v2x.launch.py ego_actor_id:=42
    ros2 launch carla_vru_demo aeb_v2x.launch.py fusion_mode:=v2x_only
    ros2 launch carla_vru_demo aeb_v2x.launch.py ns3_duration:=300

This is the V2X equivalent of `aeb.launch.py`. The baseline launch file is left
untouched so the no-V2X comparison stays valid.

Startup order (one shot):
    1. carla_v2x_bridge      — opens TCP server on :8100 and waits for ns-3
    2. carla_aeb_v2x_agent   — subscribes to /v2x/cam_received and CARLA topics
    3. ns-3 gateway          — connects in to the bridge (started after a 2 s
                               delay so the bridge has time to listen)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, TimerAction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("carla_aeb_v2x_agent")
    default_params = os.path.join(pkg_dir, "config", "aeb_v2x_params.yaml")

    ego_actor_id = LaunchConfiguration("ego_actor_id")
    fusion_mode  = LaunchConfiguration("fusion_mode")
    ns3_dir      = LaunchConfiguration("ns3_dir")
    ns3_duration = LaunchConfiguration("ns3_duration")
    bridge_port  = LaunchConfiguration("bridge_port")
    ego_role     = LaunchConfiguration("ego_role_name")

    return LaunchDescription([
        DeclareLaunchArgument("ego_actor_id", default_value="0",
            description="CARLA actor ID of ego (printed by run_scenario.py)"),
        DeclareLaunchArgument("fusion_mode", default_value="both",
            description="lidar_only | v2x_only | both"),
        DeclareLaunchArgument("ego_role_name", default_value="hero",
            description="role_name attribute used to find ego in CARLA"),
        DeclareLaunchArgument("bridge_port", default_value="8100",
            description="TCP port for ns-3 ↔ bridge"),
        DeclareLaunchArgument("ns3_dir",
            default_value=os.path.expanduser("~/iotav/ns-3-dev"),
            description="Path to ns-3-dev (where ./ns3 lives)"),
        DeclareLaunchArgument("ns3_duration", default_value="300",
            description="ns-3 simulation duration in seconds"),

        # Bridge — must start first so ns-3 has something to connect to
        Node(
            package="carla_v2x_bridge",
            executable="carla_v2x_bridge",
            name="v2x_bridge",
            output="screen",
            parameters=[{
                "ego_role_name": ego_role,
                "bridge_port":   bridge_port,
            }],
        ),

        # V2X-aware AEB
        Node(
            package="carla_aeb_v2x_agent",
            executable="carla_aeb_v2x_agent",
            name="aeb_node_v2x",
            output="screen",
            parameters=[
                default_params,
                {"ego_actor_id": ego_actor_id,
                 "fusion_mode":  fusion_mode},
            ],
        ),

        # ns-3 gateway — delayed 2 s so the bridge has time to bind :8100
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=["./ns3", "run",
                         ["gateway-v2p-wifi --verbose --duration=", ns3_duration]],
                    cwd=ns3_dir,
                    output="screen",
                ),
            ],
        ),
    ])
