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
 * Author: Modified by Hadhoum Hajjaj <hadhoum.hajjaj@nist.gov>
*/"""
import launch
import launch_ros.actions

def generate_launch_description():
    ld = launch.LaunchDescription([
        
        launch.actions.DeclareLaunchArgument(
            name='carla_host',
            default_value='localhost',
            description='IP address of the CARLA Simulator'
        ),
        launch.actions.DeclareLaunchArgument(
            name='carla_port',
            default_value='2000',
            description='TCP port number of the CARLA Simulator'
        ),
        launch.actions.DeclareLaunchArgument(
            name='delay_ms',
            default_value='0',
            description='Constant amount of additional delay to add to messages received from ns-3 (in milliseconds)'
        ),
        launch_ros.actions.Node(
            package="ns3_ros_bridge",
            executable="bridge",
            name="ns3_bridge",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    'carla_host': launch.substitutions.LaunchConfiguration('carla_host')
                },
                {
                    'carla_port': launch.substitutions.LaunchConfiguration('carla_port')
                },
                {
                    'delay_ms': launch.substitutions.LaunchConfiguration('delay_ms')
                }
            ]
        )
    ])
    return ld

if __name__ == '__main__':
    generate_launch_description()
