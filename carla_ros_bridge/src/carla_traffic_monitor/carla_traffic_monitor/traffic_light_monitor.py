import rclpy
from rclpy.node import Node
from carla_msgs.msg import CarlaTrafficLightInfoList, CarlaTrafficLightStatusList, TrafficLightInfo
from nav_msgs.msg import Odometry
import math
import carla
import csv
from datetime import datetime


class TrafficLightDistanceLogger(Node):
    def __init__(self):
        super().__init__('traffic_light_distance_logger')
        self.traffic_lights = {}
        self.traffic_light_status = {}
        self.ego_vehicle_position = None
        self.ego_vehicle_orientation = None

        # CSV file setup
        self.csv_file = "traffic_light_log.csv"
        with open(self.csv_file, mode="w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["Timestamp", "Traffic Light ID", "Remaining Time (s)", "State"])

        # Connect to CARLA
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()

        # Publisher
        self.traffic_light_in_front_pub = self.create_publisher(
            TrafficLightInfo, '/carla/hero/traffic_light_in_front', 10)

        # Subscriptions
        self.create_subscription(
            CarlaTrafficLightInfoList,
            '/carla/traffic_lights/info',
            self.info_callback,
            10
        )
        self.create_subscription(
            CarlaTrafficLightStatusList,
            '/carla/traffic_lights/status',
            self.status_callback,
            10
        )
        self.create_subscription(
            Odometry,
            '/carla/hero/odometry',
            self.odometry_callback,
            10
        )

        # Timer to log distances periodically and find the nearest traffic light
        self.timer = self.create_timer(0.5, self.log_and_publish_nearest_traffic_light)
        self.get_logger().info("TrafficLightDistanceLogger node initialized.")

    def info_callback(self, msg):
        if not msg.traffic_lights:
            self.get_logger().warn("No traffic lights received in info message.")
            return

        # Store traffic lights in a dictionary
        self.traffic_lights = {light.id: light for light in msg.traffic_lights}

    def status_callback(self, msg):
        if not msg.traffic_lights:
            self.get_logger().warn("No traffic lights received in status message.")
            return

        # Store traffic light statuses in a dictionary
        self.traffic_light_status = {light.id: light for light in msg.traffic_lights}

    def odometry_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.ego_vehicle_position = (position.x, position.y)
        self.ego_vehicle_orientation = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y**2 + orientation.z**2)
        )

    def status_callback(self, msg):
        if not msg.traffic_lights:
            self.get_logger().warn("No traffic lights received in status message.")
            return

        # Store traffic light statuses as dictionaries, not CarlaTrafficLightStatus objects
        self.traffic_light_status = {
            light.id: {
                'last_state': None,
                'remaining_time': 29.5  # Default initial red duration
            }
            for light in msg.traffic_lights
        }

    def log_and_publish_nearest_traffic_light(self):
        if not self.traffic_lights:
            self.get_logger().warn("No traffic light data available.")
            return

        if not self.ego_vehicle_position or self.ego_vehicle_orientation is None:
            self.get_logger().warn("Ego vehicle position or orientation is not set.")
            return

        nearest_light = self.find_nearest_traffic_light_in_front()
        if nearest_light:
            light_id, distance_to_traffic_light = nearest_light
            traffic_light_actor = self.get_carla_traffic_light_actor(light_id)
            if traffic_light_actor:
                state = traffic_light_actor.get_state()
                elapsed_time = traffic_light_actor.get_elapsed_time()

                # Fetch durations for each state
                yellow_time = traffic_light_actor.get_yellow_time()
                green_time = traffic_light_actor.get_green_time()

                # Initialize or retrieve last_state and remaining_time
                if light_id not in self.traffic_light_status:
                    self.traffic_light_status[light_id] = {
                        'last_state': None,
                        'remaining_time': 29.0  # Default initial red duration
                    }

                light_state = self.traffic_light_status[light_id]

                # Handle state transitions and calculate time_remaining
                if state == carla.TrafficLightState.Red:
                    if light_state['last_state'] == None:
                        # Transition from Yellow to Red: Reset to 45 seconds
                        time_remaining = self.calculate_red_light_remaining_time(traffic_light_actor, 29.0)
                    else:
                        # Calculate remaining time with the current duration
                        time_remaining = self.calculate_red_light_remaining_time(traffic_light_actor, 60.0)

                    # Log red light time to CSV
                    self.log_red_light_data(light_id, time_remaining, state)

                elif state == carla.TrafficLightState.Yellow:
                    time_remaining = yellow_time - elapsed_time
                elif state == carla.TrafficLightState.Green:
                    time_remaining = green_time - elapsed_time
                else:
                    time_remaining = 0.0  # Default for unknown/off state

                # Avoid negative remaining time
                if time_remaining < 0:
                    time_remaining = 0.0

                # Update last_state
                light_state['last_state'] = state
                light_state['remaining_time'] = time_remaining

                # Log the information
                position = traffic_light_actor.get_transform().location
                vehicle_x, vehicle_y = self.ego_vehicle_position
                self.get_logger().info(
                    f"Nearest Traffic Light ID: {light_id}, Distance to Traffic Light: {distance_to_traffic_light:.2f} meters, "
                    f"Status: {state}, Time Remaining: {time_remaining:.2f}s\n"
                    f"Vehicle Position: ({vehicle_x:.2f}, {vehicle_y:.2f})\n"
                    f"Traffic Light Position: ({position.x:.2f}, {position.y:.2f})"
                )

                # Publish the nearest traffic light info
                traffic_light_info = TrafficLightInfo()
                traffic_light_info.id = light_id
                traffic_light_info.distance = distance_to_traffic_light
                traffic_light_info.status = str(state)
                traffic_light_info.time_remaining = time_remaining
                self.traffic_light_in_front_pub.publish(traffic_light_info)
            else:
                self.get_logger().warn(f"Could not fetch CARLA actor for traffic light ID: {light_id}")
        else:
            self.get_logger().info("No traffic light is directly in front of the vehicle.")


    def calculate_red_light_remaining_time(self, nearest_traffic_light, red_duration):
        """
        Calculate the time remaining for the red light of the nearest traffic light
        based solely on the given red duration.
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
            total_elapsed_time = sum(tl.get_elapsed_time() for tl in intersection_traffic_lights)

            # Handle remaining time countdown
            if remaining_time > 0.075:
                if total_elapsed_time < state['last_elapsed_time']:
                    remaining_time = max(0.0, remaining_time)
                else:
                    remaining_time = max(0.0, remaining_time - (total_elapsed_time - state['last_elapsed_time']))
            else:
                remaining_time = red_duration

            # Update the state
            state['remaining_time'] = remaining_time
            state['last_elapsed_time'] = total_elapsed_time
            self.traffic_light_states[traffic_light_id] = state

            return remaining_time
        except Exception as e:
            self.get_logger().error(f"Error calculating red light time: {e}")
            return 0.0




    def log_red_light_data(self, light_id, remaining_time, state):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with open(self.csv_file, mode="a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow([timestamp, light_id, remaining_time, state])


    def find_nearest_traffic_light_in_front(self):
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
            # Retrieve the exact distance using calculate_distance
            nearest_light_info = self.traffic_lights[nearest_light_id]
            nearest_light_position = nearest_light_info.transform.position
            exact_distance = self.calculate_distance(
                self.ego_vehicle_position[0],
                self.ego_vehicle_position[1],
                nearest_light_position.x,
                nearest_light_position.y
            )
            return (nearest_light_id, exact_distance)
        else:
            return None


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

    def get_carla_traffic_light_actor(self, light_id):
        for actor in self.world.get_actors().filter('traffic.traffic_light'):
            if actor.id == light_id:
                return actor
        return None

    def calculate_distance(self, ego_x, ego_y, light_x, light_y):
        return math.sqrt((ego_x - light_x) ** 2 + (ego_y - light_y) ** 2)

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



def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDistanceLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

    node = TrafficLightDistanceLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
