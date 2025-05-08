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
from carla_msgs.msg import CarlaTrafficLightInfoList, CarlaTrafficLightStatusList, TrafficLightInfo
from nav_msgs.msg import Odometry
import math
import carla
from transitions import Machine
from rosgraph_msgs.msg import Clock
import socket


class TrafficLightMonitor(Node):
    """ ROS2 Node that detects the nearest traffic light, calculates the distance to it, determines the remaining time for each state, and identifies transitions between traffic light states using finite state machine (fsm). """

    states = ["Red", "Yellow", "Green"]

    def __init__(self):
        super().__init__("traffic_light_monitor")
        
        # ROS2 Variables
        self.traffic_lights = {}
        self.traffic_light_status = {}
        self.ego_vehicle_position = None
        self.ego_vehicle_orientation = None
        self.vehicle_velocity = (0.0, 0.0, 0.0)
        self.sim_time_sec = 0
        self.sim_time_nanosec = 0
        self.previous_state = "Red"
        self.nearest_traffic_light_id = None

        # TCP socket connection to intermediate server
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(('localhost', 7000))
        self.get_logger().info("✅ Connected to intermediate server on port 7000")

        # Connect to CARLA
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()

        # Publisher
        self.traffic_light_in_front_pub = self.create_publisher(
            TrafficLightInfo, '/carla/hero/traffic_light_in_front', 10)

        # FSM Initialization
        self.machine = Machine(model=self, states=TrafficLightMonitor.states, initial="Red")
        self.machine.add_transition("turn_green", "Red", "Green")
        self.machine.add_transition("turn_yellow", "Green", "Yellow")
        self.machine.add_transition("turn_red", "Yellow", "Red")

        # ROS2 Subscriptions
        self.create_subscription(CarlaTrafficLightInfoList, "/carla/traffic_lights/info", self.info_callback, 10)
        self.create_subscription(CarlaTrafficLightStatusList, "/carla/traffic_lights/status", self.status_callback, 10)
        self.create_subscription(Odometry, "/carla/hero/odometry", self.odometry_callback, 10)
        self.create_subscription(Clock, '/clock', self.clock_callback, 10)


        # Timer to Process Traffic LIght
        self.timer = self.create_timer(0.1, self.process_nearest_traffic_light)
        self.get_logger().info("🚦 TrafficLightMonitor Node Initialized!")

    def info_callback(self, msg):
        """ Callback function for handling incoming traffic light information messages"""
        if not msg.traffic_lights:
            self.get_logger().warn("No traffic lights received in info message.")
            return

        # Store traffic lights in a dictionary
        self.traffic_lights = {light.id: light for light in msg.traffic_lights}

    def status_callback(self, msg):
        """ Updates traffic light states """
        if not msg.traffic_lights:
            self.get_logger().warn("No traffic lights received in status message.")
            return

        # Store traffic light statuses in a dictionary
        for light in msg.traffic_lights:
            self.traffic_light_status[light.id] = light.state

    def odometry_callback(self, msg):
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        self.ego_vehicle_position = (pos.x, pos.y)
        self.vehicle_velocity = (vel.x, vel.y, vel.z)

        orientation = msg.pose.pose.orientation
        self.ego_vehicle_orientation = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y**2 + orientation.z**2)
        )

    def process_nearest_traffic_light(self):
        """ Detect and process the nearest traffic light. """
        if not self.traffic_lights or not self.ego_vehicle_position:
            return

        nearest_light = self.find_nearest_traffic_light()
        if not nearest_light:
            return

        light_id, distance_to_light = nearest_light
        traffic_light_actor = self.get_carla_traffic_light_actor(light_id)
        if not traffic_light_actor:
            return

        state = traffic_light_actor.get_state()
        elapsed_time = traffic_light_actor.get_elapsed_time()
        current_state = self.traffic_light_status.get(light_id, "Unknown")
        reset_red_timer = self.handle_state_transition(self.previous_state, current_state, light_id)
        self.previous_state = current_state

        # Calculate remaining time, log traffic ligh info and publish to the topic
        red_duration = self.calculate_red_light_duration(traffic_light_actor)
        time_remaining = self.calculate_light_remaining_time(state, elapsed_time, traffic_light_actor, red_duration, reset_red_timer)
        self.log_traffic_light_info(light_id, state, distance_to_light, time_remaining, traffic_light_actor.get_transform().location)
        self.publish_traffic_light_info(light_id, state, distance_to_light, time_remaining, traffic_light_actor.get_transform().location)


    def find_nearest_traffic_light(self):
        """Find the traffic light that impact the vehicle. Return its id and the distance to its stop line."""

        candidate_lights = []

        for light_id, light_info in self.traffic_lights.items():
            light_position = light_info.transform.position
            light_x, light_y = light_position.x, light_position.y
            distance = self.calculate_distance(self.ego_vehicle_position[0], self.ego_vehicle_position[1], light_x, light_y)

            if self.is_in_front_of_vehicle(light_x, light_y) and distance < 300:
                candidate_lights.append((light_id, light_info))

        nearest_light_id = None
        min_avg_distance = float('inf')

        for light_id, light_info in candidate_lights:
            traffic_light_actor = self.get_carla_traffic_light_actor(light_id)
            if traffic_light_actor:
                affected_waypoints = traffic_light_actor.get_affected_lane_waypoints()
                if affected_waypoints:
                    avg_distance = self.calculate_average_distance_to_waypoints(affected_waypoints)
                    if avg_distance < min_avg_distance:
                        min_avg_distance = avg_distance
                        nearest_light_id = light_id

        if nearest_light_id:
            # Retrieve the distance to stop line
            nearest_light_info = self.traffic_lights[nearest_light_id]
            nearest_light_position = nearest_light_info.transform.position
            exact_distance = self.ego_vehicle_position[1] - (-120.03)

            return (nearest_light_id, exact_distance)
        else:
            return None

    def is_in_front_of_vehicle(self, light_x, light_y):
        """Check if the traffic light is in front of the vehicle based on orientation."""
        ego_x, ego_y = self.ego_vehicle_position
        dx = light_x - ego_x
        dy = light_y - ego_y

        # Calculate angle to traffic light relative to the vehicle's orientation
        angle_to_light = math.atan2(dy, dx)
        angle_diff = angle_to_light - self.ego_vehicle_orientation

        # Normalize angle difference to [-pi, pi]
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        # Consider traffic lights within a certain angle threshold in front of the vehicle
        return 0 > angle_diff > -math.radians(30)

    def calculate_light_remaining_time(self, state, elapsed_time, traffic_light_actor, red_duration, reset_red_timer):
        """ Calculate the remaining time for the current traffic light state. """
        yellow_time= traffic_light_actor.get_yellow_time()
        green_time = traffic_light_actor.get_green_time()

        if state == carla.TrafficLightState.Red:
            time_remaining = self.calculate_red_light_remaining_time(traffic_light_actor, red_duration, reset_red_timer)
        elif state == carla.TrafficLightState.Yellow:
            time_remaining = yellow_time - elapsed_time
        elif state == carla.TrafficLightState.Green:
            time_remaining = green_time - elapsed_time
        else:
            time_remaining = 0.0  # Default for unknown/off state

        # Avoid negative remaining time
        if time_remaining < 0:
            time_remaining = 0.0
        return time_remaining

    def calculate_red_light_remaining_time(self, nearest_traffic_light, red_duration, reset_red_timer):
        """
        Calculate the time remaining for the red light of the nearest traffic light,
        while handling state transitions externally.
        """
        try:
            # Persistent storage for traffic light states
            if not hasattr(self, 'traffic_light_states'):
                self.traffic_light_states = {}

            # Get the traffic light ID
            traffic_light_id = nearest_traffic_light.id

            # Initialize or retrieve the state for this traffic light
            if traffic_light_id not in self.traffic_light_states:
                self.traffic_light_states[traffic_light_id] = {
                    'remaining_time': red_duration,
                    'last_elapsed_time': 0.0
                }

            state = self.traffic_light_states[traffic_light_id]
            remaining_time = state['remaining_time']

            # Get all traffic lights in the world
            all_traffic_lights = self.world.get_actors().filter('traffic.traffic_light')

            # Identify traffic lights in the same intersection by proximity
            intersection_traffic_lights = [
                tl for tl in all_traffic_lights
                if self.calculate_distance(
                    nearest_traffic_light.get_transform().location.x,
                    nearest_traffic_light.get_transform().location.y,
                    tl.get_transform().location.x,
                    tl.get_transform().location.y
                ) < 50 and tl.id != traffic_light_id
            ]

            # Calculate total elapsed time of other traffic lights
            total_elapsed_time = sum(tl.get_elapsed_time() for tl in intersection_traffic_lights) + nearest_traffic_light.get_elapsed_time()

            # Reset logic: Handle transitions from non-red to red (externally detected via last_state)
            if reset_red_timer:
                remaining_time = red_duration  # Reset to full red duration
            else:
                # Handle countdown logic
                if remaining_time > 0.0:
                    if total_elapsed_time < state['last_elapsed_time']:
                        # Prevent backward jumps in elapsed time
                        remaining_time = max(0.0, remaining_time)
                    else:
                        # Count down the remaining time
                        remaining_time = max(0.0, remaining_time - (total_elapsed_time - state['last_elapsed_time']))
                else:
                    # If remaining_time hits 0, stay there until reset
                    remaining_time = 0.0

            # Update state
            state['remaining_time'] = remaining_time
            state['last_elapsed_time'] = total_elapsed_time
            self.traffic_light_states[traffic_light_id] = state

            return remaining_time
        except Exception as e:
            self.get_logger().error(f"Error calculating red light time: {e}")
            return 0.0

    def calculate_red_light_duration(self, nearest_traffic_light):
        """
        Calculate the red duration for the nearest traffic light by summing up
        the green, yellow, and red durations of other traffic lights in the same intersection.
        """
        try:
            # Get the nearest traffic light ID and its position
            traffic_light_id = nearest_traffic_light.id
            nearest_light_position = nearest_traffic_light.get_transform().location

            # Get all traffic lights in the world
            all_traffic_lights = self.world.get_actors().filter('traffic.traffic_light')

            # Identify traffic lights in the same intersection by proximity (exclude the nearest one)
            intersection_traffic_lights = [
                tl for tl in all_traffic_lights
                if self.calculate_distance(
                    nearest_light_position.x, nearest_light_position.y,
                    tl.get_transform().location.x, tl.get_transform().location.y
                ) < 50 and tl.id != traffic_light_id
            ]

            # Calculate the red duration as the sum of green, yellow, and red durations of other traffic lights
            red_duration = 0.0
            for traffic_light in intersection_traffic_lights:
                green_time = traffic_light.get_green_time()
                yellow_time = traffic_light.get_yellow_time()
                red_time = traffic_light.get_red_time()

                red_duration += green_time + yellow_time + red_time

            return red_duration

        except Exception as e:
            self.get_logger().error(f"Error calculating red duration: {e}")
            return 0.0

    def handle_state_transition(self, previous, current, light_id):
        """ Handles FSM state transitions and logs the change """
        reset_red_timer = False  # Initialize flag as False

        try:
            if previous == 0 and current == 2:  # Red → Green
                self.to_Green()
                reset_red_timer = False  # Reset flag when Red → Green

            elif previous == 2 and current == 1:  # Green → Yellow
                self.to_Yellow()
                reset_red_timer = False  # Reset flag when Green → Yellow

            elif previous == 1 and current == 0:  # Yellow → Red (Reset Timer Here!)
                if self.state != "Red":  # Ensure we're not already in Red
                    self.to_Red()
                    reset_red_timer = True  # 🚨 Set flag when transitioning to Red
                else:
                    self.get_logger().warn(f"⚠️ Traffic Light {light_id} already in Red, ignoring transition.")

            self.get_logger().info(f"🚦 Traffic Light {light_id}: {previous} ➝ {current}, Reset: {reset_red_timer}")

        except Exception as e:
            self.get_logger().warn(f"⚠️ Invalid transition for Traffic Light {light_id}: {previous} ➝ {current} ({e})")

        return reset_red_timer

    def get_carla_traffic_light_actor(self, light_id):
        for actor in self.world.get_actors().filter('traffic.traffic_light'):
            if actor.id == light_id:
                return actor
        return None

    def calculate_distance(self, ego_x, ego_y, light_x, light_y):
        return math.sqrt((ego_x - light_x) ** 2 + (ego_y - light_y) ** 2)

    def calculate_average_distance_to_waypoints(self, waypoints):
        total_distance = 0.0
        for wp in waypoints:
            wp_location = wp.transform.location
            distance = self.calculate_distance(
                self.ego_vehicle_position[0],
                self.ego_vehicle_position[1],
                wp_location.x,
                wp_location.y
            )
            total_distance += distance
        return total_distance / len(waypoints) if waypoints else float('inf')

    def publish_traffic_light_info(self, light_id, state, distance, time_remaining, position):
        msg = TrafficLightInfo()
        msg.id = light_id
        msg.distance = distance
        msg.status = str(state)
        msg.time_remaining = time_remaining
        self.traffic_light_in_front_pub.publish(msg)

        # === Send over TCP to intermediate server ===
        vx, vy, vz = self.vehicle_velocity
        x, y = self.ego_vehicle_position
        z = 0.0  # dummy if needed
        send_flag = 0

        if 'Red' in str(state):
            state_code = 3
        elif 'Yellow' in str(state):
            state_code = 2
        elif 'Green' in str(state):
            state_code = 1
        else:
            state_code = -1

        tcp_message = f"{self.sim_time_sec},{self.sim_time_nanosec},{x:.2f},{y:.2f},{z:.2f},{vx:.2f},{vy:.2f},{vz:.2f},{send_flag},{state_code},{time_remaining:.2f};"
        try:
            self.sock.sendall(tcp_message.encode())
            self.get_logger().info(f"📤 Sent: {tcp_message}")
        except Exception as e:
            self.get_logger().error(f"❌ TCP send failed: {e}")

    def destroy_node(self):
        self.sock.close()
        self.get_logger().info("✅ Closed TCP connection")
        super().destroy_node()


    def clock_callback(self, msg):
        self.sim_time_sec = msg.clock.sec
        self.sim_time_nanosec = msg.clock.nanosec


    def log_traffic_light_info(self, light_id, state, distance, time_remaining, position):
        vehicle_x, vehicle_y = self.ego_vehicle_position
        sim_time = self.sim_time_sec + self.sim_time_nanosec * 1e-9  # convert to float seconds
        self.get_logger().info(
            f"[🕒 {sim_time:.3f} s] 🚦 Light ID: {light_id}, Distance: {distance:.2f} m, Status: {state}, "
            #f"🚦 Light ID: {light_id}, Distance: {distance:.2f}m, Status: {state}, "
            f"Time Left: {time_remaining:.2f}s | Light Position: ({position.x:.2f}, {position.y:.2f}), "
            f"Vehicle Position: ({vehicle_x:.2f}, {vehicle_y:.2f})"
        )

def main(args=None):
    """ ROS2 Node Execution """
    rclpy.init(args=args)
    node = TrafficLightMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
