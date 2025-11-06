#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'role_name',
            default_value='hero',
            description='Name of the ego vehicle in CARLA'
        ),
        DeclareLaunchArgument(
            'intermediate_server_host',
            default_value='localhost',
            description='Hostname of the intermediate server (NS3)'
        ),
        DeclareLaunchArgument(
            'intermediate_server_port',
            default_value='9000',
            description='Port of the intermediate server (NS3)'
        ),
        DeclareLaunchArgument(
            'max_speed',
            default_value='13.41',
            description='Maximum speed in m/s (default: 30 mph)'
        ),
        DeclareLaunchArgument(
            'max_comfort_decel',
            default_value='2.8',
            description='Maximum comfortable deceleration in m/s²'
        ),
        DeclareLaunchArgument(
            'max_decel',
            default_value='8.0',
            description='Maximum emergency deceleration in m/s²'
        ),
        DeclareLaunchArgument(
            'stopline_offset',
            default_value='27.0',
            description='Distance offset to stopline in meters'
        ),
        DeclareLaunchArgument(
            'control_dt',
            default_value='0.01',
            description='Control loop frequency in seconds'
        ),
        
        # Traffic Light Controller Node
        Node(
            package='carla_traffic_light_controller',
            executable='traffic_light_controller_node',
            name='traffic_light_controller',
            output='screen',
            parameters=[{
                'role_name': LaunchConfiguration('role_name'),
                'intermediate_server_host': LaunchConfiguration('intermediate_server_host'),
                'intermediate_server_port': LaunchConfiguration('intermediate_server_port'),
                'max_speed': LaunchConfiguration('max_speed'),
                'max_comfort_decel': LaunchConfiguration('max_comfort_decel'),
                'max_decel': LaunchConfiguration('max_decel'),
                'stopline_offset': LaunchConfiguration('stopline_offset'),
                'control_dt': LaunchConfiguration('control_dt'),
            }]
        )
    ])