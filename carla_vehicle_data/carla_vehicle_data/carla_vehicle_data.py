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
import rclpy
from rclpy.node import Node
import carla
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaActorList, CarlaActorInfo
from derived_object_msgs.msg import ObjectArray
from cmath import sqrt

class CarlaVehicleDataPublisher(Node):
    def __init__(self):
        super().__init__('carla_leadVehicle_data_publisher')
        self.vehicle_roles = {}

        # Initialize CARLA client
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)

        # Publisher for vehicle list
        self.publisher = self.create_publisher(CarlaActorList, '/carla/vehicles', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Subscriber for vehicles (including 'hero')
        self.objects_sub = self.create_subscription(ObjectArray, '/carla/objects', self.objects_callback, 10)

    def timer_callback(self):
        world = self.client.get_world()
        vehicles = world.get_actors().filter('vehicle.*')

        actor_list_msg = CarlaActorList()
        for vehicle in vehicles:
            actor_info = CarlaActorInfo()
            actor_info.id = vehicle.id
            actor_info.type = vehicle.type_id
            actor_info.rolename = vehicle.attributes.get('role_name', 'unknown')
            actor_list_msg.actors.append(actor_info)

            # Update vehicle_roles for object callback use
            self.vehicle_roles[vehicle.id] = actor_info.rolename

        self.publisher.publish(actor_list_msg)

    def objects_callback(self, msg):
        for object in msg.objects:
            role_name = self.vehicle_roles.get(object.id, 'unknown')
            if role_name in ['hero', 'FollowCar', 'LeadCar']:
                x = object.pose.position.x
                y = object.pose.position.y
                z = object.pose.position.z
                accel_x = object.accel.linear.x
                accel_y = object.accel.linear.y
                accel_z = object.accel.linear.z
                accel = sqrt(accel_x*2 + accel_y*2 + accel_z*2)
                vehicle_data = f'[{role_name}] [x={x}, y={y}, z={z}, accel_y={accel_y}]'
                Vehicle_acceleration = f'[{role_name}] [accel_x={accel_x}, accel_y={accel_y}, accel_z={accel_z}, accel={accel}]'
                if role_name == 'LeadCar':
                    self.get_logger().info(Vehicle_acceleration)

def main(args=None):
    rclpy.init(args=args)
    node = CarlaVehicleDataPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
