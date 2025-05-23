import rclpy
from derived_object_msgs.msg import ObjectArray
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64

import carla
import socket

class NetworkSimulatorBridge(Node):
    def __init__(self):
        super().__init__('network_simulator_bridge')

        # Declare Parameters
        self.declare_parameter('carla_host', 'localhost')
        self.declare_parameter('carla_port', 2000)
        self.declare_parameter('delay_ms', 0)
        self.add_on_set_parameters_callback(self.callback_parameter)

        # Create Subscriptions
        self.create_subscription(Clock, 'clock', self.callback_clock, 10)
        self.create_subscription(ObjectArray, '/carla/objects', self.callback_carla_objects, 10)

        # Create Publications
        self.publish_target_speed = self.create_publisher(Float64, '/carla/hero/target_speed', QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        # Setup the CARLA Client
        host = self.get_parameter('carla_host').value
        port = self.get_parameter('carla_port').value

        self.get_logger().debug("trying to connect to CARLA server at {}:{}...".format(host, port))
        self.carla_client = carla.Client(host, port)
        self.carla_client.set_timeout(2.0)
        self.get_logger().info("connected to CARLA server running version {}".format(self.carla_client.get_server_version()))

        # Setup the TCP Server for ns-3
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # allow immediate re-use of address if code restarted
        self.server_socket.bind(('127.0.0.1', 1111))
        self.server_socket.listen(1)
        self.get_logger().info("TCP/IP server at 127.0.0.1:1111 waiting for client connection...")
        self.client_socket, client_address = self.server_socket.accept()
        self.get_logger().info("accepted client with address {}".format(client_address))

        # Initialize Variables
        self.delay_ms = self.get_parameter('delay_ms').value

        self.clock_sec = 0
        self.clock_nanosec = 0

        self.tracked_roles = ['hero', 'FollowCar', 'LeadCar']
        self.vehicle_id_to_role = {}

        self.last_received_bsm = 0
        self.bsm_received_time = 0

        self.received_clock = False
        self.received_objects = False

        self.lead_car_speed = 0
        self.ego_state = 'FOLLOWING'  # Possible states: FOLLOWING, STOPPED

    def callback_parameter(self, params):
        success = False

        for param in params:
            if param.name == 'delay_ms':
                self.delay_ms = param.value
                self.get_logger().info("PARAMETER delay_ms set as {}".format(self.delay_ms))
                success = True
            if param.name in ['carla_host', 'carla_port']:
                self.get_logger().warn("PARAMETER {} cannot be modified".format(param.name))
        return SetParametersResult(successful=success)

    def callback_clock(self, message):
        if self.received_clock:
            self.get_logger().warn("received multiple /clock values between updates")
        self.received_clock = True

        self.clock_sec = message.clock.sec
        self.clock_nanosec = message.clock.nanosec
        self.get_logger().debug("received /clock: {} s + {} ns".format(self.clock_sec, self.clock_nanosec))

        self.update()

    def callback_carla_objects(self, message):
        if self.received_objects:
            self.get_logger().warn("received multiple /carla/objects values between updates")
        self.received_objects = True

        self.objects = message.objects

        self.update()

    def update(self):
        if not self.received_clock or not self.received_objects:
            return  # require both subscriptions to proceed

        self.get_world_state()

        # Require all tracked roles to be defined exactly once in CARLA
        if sorted(list(self.vehicle_id_to_role.values())) != sorted(self.tracked_roles):
            self.get_logger().warn("the CARLA world does not contain the expected vehicle roles: {}".format(self.tracked_roles))
            return

        clock_ms = (self.clock_sec * 1000) + (self.clock_nanosec // 1000000)
        self.get_logger().info("current time = {} ms".format(clock_ms))

        num_objects = 0
        packet_string = str(self.clock_sec) + ',' + str(self.clock_nanosec)
        for obj in self.objects:
            if obj.id in self.vehicle_id_to_role:
                role = self.vehicle_id_to_role[obj.id]
                data = [
                    obj.pose.position.x,
                    obj.pose.position.y,
                    obj.pose.position.z,
                    obj.accel.linear.x,
                    obj.accel.linear.y,
                    obj.accel.linear.z,
                    obj.twist.linear.x,  # Adding the speed of the object
                    obj.twist.linear.y,
                    obj.twist.linear.z
                ]
                data_string = ','.join(str(d) for d in data)
                packet_string += '\n' + role + ',' + data_string
                num_objects += 1
                if role == 'LeadCar':
                    self.lead_car_speed = (obj.twist.linear.x**2 + obj.twist.linear.y**2 + obj.twist.linear.z**2)**0.5
        packet_string += '\r\n'
        self.get_logger().debug("constructed packet: {}".format(repr(packet_string)))

        if num_objects != len(self.tracked_roles):  # will happen one tick during initialization
            self.get_logger().warn("skipped update: received {} out of {} vehicle objects".format(str(num_objects), str(len(self.tracked_roles))))
            return

        # Send data to ns-3 and receive the response
        self.client_socket.send(packet_string.encode())
        response = self.client_socket.recv(4096).decode()
        self.get_logger().debug("received response: {}".format(response))

        # Extract the BSM timestamp for the hero vehicle (first value in the response)
        hero_bsm_timestamp = float(response.split(',')[0])  # assuming the format: "ts_hero,ts_FollowCar,ts_LeadCar"

        # Update the state if a new BSM is received
        if hero_bsm_timestamp > self.last_received_bsm:
            self.get_logger().info("Received new BSM with timestamp {}".format(hero_bsm_timestamp))
            self.set_target_speed(0)
            self.last_received_bsm = hero_bsm_timestamp
            self.bsm_received_time = clock_ms
            self.ego_state = 'STOPPED'

        # Check if the lead car has started moving again and update the ego vehicle speed
        if self.ego_state == 'STOPPED' and self.lead_car_speed > 1.0 and (clock_ms - self.bsm_received_time) > 1000:
            self.get_logger().info("Lead car is moving again, setting target speed to follow")
            self.set_target_speed(20)
            self.ego_state = 'FOLLOWING'

        self.received_clock = False
        self.received_objects = False

    def get_world_state(self):
        carla_world = self.carla_client.get_world()

        self.vehicle_id_to_role.clear()
        for vehicle in carla_world.get_actors().filter('vehicle.*'):
            role_name = vehicle.attributes.get('role_name')  # can return None
            if role_name in self.tracked_roles:
                self.vehicle_id_to_role[vehicle.id] = role_name

    def set_target_speed(self, speed):
        message = Float64()
        message.data = float(speed)
        self.publish_target_speed.publish(message)
        self.get_logger().debug("Published target_speed: {}".format(message.data))

def main(args=None):
    rclpy.init(args=args)
    node = NetworkSimulatorBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
