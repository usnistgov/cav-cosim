"""/*
 * NIST-developed software is provided by NIST as a public service. You may use,
 * copy, and distribute copies of the software in any medium, provided that you
 * keep intact this entire notice. You may improve, modify, and create
 * derivative works of the software or any portion of the software, and you may
 * copy and distribute such modifications or works. Modified works should carry
 * a notice stating that you changed the software and should note the date and
 * nature of any such change. Please explicitly acknowledge the National
 * Institute of Standards and Technology as the source of the software. 
 *
 * NIST-developed software is expressly provided "AS IS." NIST MAKES NO WARRANTY
 * OF ANY KIND, EXPRESS, IMPLIED, IN FACT, OR ARISING BY OPERATION OF LAW,
 * INCLUDING, WITHOUT LIMITATION, THE IMPLIED WARRANTY OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, NON-INFRINGEMENT, AND DATA ACCURACY. NIST
 * NEITHER REPRESENTS NOR WARRANTS THAT THE OPERATION OF THE SOFTWARE WILL BE
 * UNINTERRUPTED OR ERROR-FREE, OR THAT ANY DEFECTS WILL BE CORRECTED. NIST DOES
 * NOT WARRANT OR MAKE ANY REPRESENTATIONS REGARDING THE USE OF THE SOFTWARE OR
 * THE RESULTS THEREOF, INCLUDING BUT NOT LIMITED TO THE CORRECTNESS, ACCURACY,
 * RELIABILITY, OR USEFULNESS OF THE SOFTWARE.
 * 
 * You are solely responsible for determining the appropriateness of using and
 * distributing the software and you assume all risks associated with its use,
 * including but not limited to the risks and costs of program errors,
 * compliance with applicable laws, damage to or loss of data, programs or
 * equipment, and the unavailability or interruption of operation. This software 
 * is not intended to be used in any situation where a failure could cause risk
 * of injury or damage to property. The software developed by NIST employees is
 * not subject to copyright protection within the United States.
 *
 * Author: Hadhoum Hajjaj <hadhoum.hajjaj@nist.gov>
*/"""

import os

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

# Goal Pose for the Ego Vehicle (Town01) for scenarios: Stopped Vehicle Ahead
# Uncomment and configure if needed

# Goal position for first example of intersection
# Spawn_point:"spawn_point": {"x": -145.52, "y": -100.15, "z": 1.0, "roll": 0.0, "pitch": 0.0, "yaw": 90.0}
"""default_goal_pose = {
    'px': float(-145.235),
    'py': float(46.6293),
    'pz': float(0.0),
    'ox': float(0.0),
    'oy': float(0.0),
    'oz': float(0.0),
    'ow': float(1.0),
}"""

# Goal position for second example of intersection
# "spawn_point": {"x": 3.76259, "y": 39.6256, "z": 1.0, "roll": 0.0, "pitch": 0.0, "yaw": 90.0},
"""default_goal_pose = {
    'px': float(7.4879),
    'py': float(168.359),
    'pz': float(0.0),
    'ox': float(0.0),
    'oy': float(0.0),
    'oz': float(0.0),
    'ow': float(1.0),
}"""

# Goal position for third example of intersection
#"spawn_point": {"x": -84.9835, "y": -28.3443, "z": 1.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
default_goal_pose = {
    'px': float(-84.7606),
    'py': float(-150.62),
    'pz': float(0.0),
    'ox': float(0.0),
    'oy': float(0.0),
    'oz': float(0.0),
    'ow': float(1.0),
}

# String with message to publish on topic /carla/available/scenarios
#ros_topic_msg_string = "{{ 'scenarios': [{{ 'name': 'Slow Vehicle Ahead', 'scenario_file': '{}'}}] }}".format(
#    os.path.join(get_package_share_directory('carla_ad_demo'), 'config/SlowVehicleAhead.xosc'))

def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='host',
            default_value='localhost'
        ),
        launch.actions.DeclareLaunchArgument(
            name='port',
            default_value='2000'
        ),
        launch.actions.DeclareLaunchArgument(
            name='town',
            default_value='Town03'
        ),
        launch.actions.DeclareLaunchArgument(
            name='timeout',
            default_value='30'
        ),
        launch.actions.DeclareLaunchArgument(
            name='synchronous_mode_wait_for_vehicle_control_command',
            default_value='False'
        ),
        launch.actions.DeclareLaunchArgument(
            name='fixed_delta_seconds',
            default_value='0.05'
        ),
        launch.actions.DeclareLaunchArgument(
            name='role_name',
            default_value='hero'
        ),
        # Include Carla ROS Bridge Launch Description
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('carla_ros_bridge'), 'carla_ros_bridge.launch.py')
            ),
            launch_arguments={
                'host': launch.substitutions.LaunchConfiguration('host'),
                'port': launch.substitutions.LaunchConfiguration('port'),
                'town': launch.substitutions.LaunchConfiguration('town'),
                'timeout': launch.substitutions.LaunchConfiguration('timeout'),
                'synchronous_mode_wait_for_vehicle_control_command': launch.substitutions.LaunchConfiguration('synchronous_mode_wait_for_vehicle_control_command'),
                'fixed_delta_seconds': launch.substitutions.LaunchConfiguration('fixed_delta_seconds'),
            }.items()
        ),
        # Include Carla Example Ego Vehicle Launch Description
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('carla_spawn_objects'), 'carla_example_ego_vehicle.launch.py')
            ),
            launch_arguments={
                'object_definition_file': os.path.join(get_package_share_directory('carla_spawn_objects'), 'config/objects.json'),
                'role_name': launch.substitutions.LaunchConfiguration('role_name')
            }.items()
        ),
        launch.actions.ExecuteProcess(
            cmd=[
                "ros2", "topic", "pub", "--once", "/carla/hero/goal_pose", "geometry_msgs/msg/PoseStamped",
                '{{ "pose": {{ \
                    "position": {{"x": {px}, "y": {py}, "z": {pz}}}, \
                    "orientation": {{"x": {ox}, "y": {oy}, "z": {oz}, "w": {ow}\
                }}}}}}'.format(
                    px=default_goal_pose['px'],
                    py=default_goal_pose['py'],
                    pz=default_goal_pose['pz'],
                    ox=default_goal_pose['ox'],
                    oy=default_goal_pose['oy'],
                    oz=default_goal_pose['oz'],
                    ow=default_goal_pose['ow'])
            ],
            name='execute_topic_pub_goal_pose',
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_waypoint_publisher'), 'carla_waypoint_publisher.launch.py')
            ),
            launch_arguments={
                'host': launch.substitutions.LaunchConfiguration('host'),
                'port': launch.substitutions.LaunchConfiguration('port'),
                'timeout': launch.substitutions.LaunchConfiguration('timeout'),
                'role_name': launch.substitutions.LaunchConfiguration('role_name')
            }.items()
        ),
        # RViz2 Node
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            remappings=[
                (
                    "carla/hero/spectator_pose",
                    "/carla/hero/rgb_view/control/set_transform"
                )
            ],
            arguments=[
                '-d', os.path.join(get_package_share_directory('carla_ad_demo'), 'config/carla_ad_demo_ros2.rviz')],
            on_exit=launch.actions.Shutdown()
        )

    ])
    return ld

if __name__ == '__main__':
    generate_launch_description()
