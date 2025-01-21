#!/usr/bin/env python

import collections
import math
import threading

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from ros_compatibility.qos import QoSProfile, DurabilityPolicy

from carla_msgs.msg import CarlaEgoVehicleControl  # pylint: disable=import-error
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker


class LocalPlanner(CompatibleNode):
    """
    LocalPlanner implements the basic behavior of following a trajectory of waypoints,
    allowing direct vehicle control commands.
    """

    # minimum distance to target waypoint as a percentage (e.g., within 90% of total distance)
    MIN_DISTANCE_PERCENTAGE = 0.9

    def __init__(self):
        super(LocalPlanner, self).__init__("local_planner")

        role_name = self.get_param("role_name", "hero")
        self.control_time_step = self.get_param("control_time_step", 0.05)

        args_lateral_dict = {}
        args_lateral_dict['K_P'] = self.get_param("Kp_lateral", 0.9)
        args_lateral_dict['K_I'] = self.get_param("Ki_lateral", 0.0)
        args_lateral_dict['K_D'] = self.get_param("Kd_lateral", 0.0)

        args_longitudinal_dict = {}
        args_longitudinal_dict['K_P'] = self.get_param("Kp_longitudinal", 0.206)
        args_longitudinal_dict['K_I'] = self.get_param("Ki_longitudinal", 0.0206)
        args_longitudinal_dict['K_D'] = self.get_param("Kd_longitudinal", 0.515)

        self.data_lock = threading.Lock()
        self._current_pose = None
        self._current_speed = None
        self._waypoints_queue = collections.deque(maxlen=20000)
        self._waypoint_buffer = collections.deque(maxlen=5)

        # subscribers
        self._odometry_subscriber = self.new_subscription(
            Odometry,
            "/carla/{}/odometry".format(role_name),
            self.odometry_cb,
            qos_profile=10)
        self._path_subscriber = self.new_subscription(
            Path,
            "/carla/{}/waypoints".format(role_name),
            self.path_cb,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        # New subscriber for control commands from Simulink
        self._control_cmd_subscriber = self.new_subscription(
            CarlaEgoVehicleControl,
            "/carla/{}/vehicle_control_cmd".format(role_name),
            self.vehicle_control_cb,
            qos_profile=10)

        # publishers
        self._target_pose_publisher = self.new_publisher(
            Marker,
            "/carla/{}/next_target".format(role_name),
            qos_profile=10)
        self._control_cmd_publisher = self.new_publisher(
            CarlaEgoVehicleControl,
            "/carla/{}/vehicle_control_cmd".format(role_name),
            qos_profile=10)

    def odometry_cb(self, odometry_msg):
        with self.data_lock:
            self._current_pose = odometry_msg.pose.pose
            self._current_speed = math.sqrt(odometry_msg.twist.twist.linear.x ** 2 +
                                            odometry_msg.twist.twist.linear.y ** 2 +
                                            odometry_msg.twist.twist.linear.z ** 2) * 3.6

    def path_cb(self, path_msg):
        with self.data_lock:
            self._waypoint_buffer.clear()
            self._waypoints_queue.clear()
            self._waypoints_queue.extend([pose.pose for pose in path_msg.poses])

    def pose_to_marker_msg(self, pose):
        marker_msg = Marker()
        marker_msg.type = 0
        marker_msg.header.frame_id = "map"
        marker_msg.pose = pose
        marker_msg.scale.x = 1.0
        marker_msg.scale.y = 0.2
        marker_msg.scale.z = 0.2
        marker_msg.color.r = 255.0
        marker_msg.color.a = 1.0
        return marker_msg

    def vehicle_control_cb(self, control_cmd_msg):
        """
        Receives control commands directly from Simulink and publishes them
        to the CARLA vehicle control command topic.
        """
        with self.data_lock:
            self._control_cmd_publisher.publish(control_cmd_msg)

    def run_step(self):
        """
        Executes one step of local planning for waypoint following,
        allows direct control via Simulink.
        """
        with self.data_lock:
            if not self._waypoint_buffer and not self._waypoints_queue:
                self.loginfo("Waiting for a route...")
                self.emergency_stop()
                return

            # Buffering the waypoints if buffer is empty
            if not self._waypoint_buffer:
                for i in range(5):
                    if self._waypoints_queue:
                        self._waypoint_buffer.append(self._waypoints_queue.popleft())
                    else:
                        break

            # target waypoint
            target_pose = self._waypoint_buffer[0]
            self._target_pose_publisher.publish(self.pose_to_marker_msg(target_pose))

            # Purge obsolete waypoints
            max_index = -1
            sampling_radius = self._current_speed * 1 / 3.6  # radius in meters
            min_distance = sampling_radius * self.MIN_DISTANCE_PERCENTAGE

            for i, route_point in enumerate(self._waypoint_buffer):
                if distance_vehicle(route_point, self._current_pose.position) < min_distance:
                    max_index = i
            if max_index >= 0:
                for i in range(max_index + 1):
                    self._waypoint_buffer.popleft()

    def emergency_stop(self):
        control_msg = CarlaEgoVehicleControl()
        control_msg.steer = 0.0
        control_msg.throttle = 0.0
        control_msg.brake = 1.0
        control_msg.hand_brake = False
        control_msg.manual_gear_shift = False
        self._control_cmd_publisher.publish(control_msg)


def main(args=None):
    """
    Main function to initialize ROS node and run the local planner.
    """
    roscomp.init("local_planner", args=args)
    local_planner = None
    update_timer = None
    try:
        local_planner = LocalPlanner()
        roscomp.on_shutdown(local_planner.emergency_stop)

        update_timer = local_planner.new_timer(
            local_planner.control_time_step, lambda timer_event=None: local_planner.run_step())

        local_planner.spin()

    except KeyboardInterrupt:
        pass

    finally:
        roscomp.loginfo('Local planner shutting down.')
        roscomp.shutdown()

if __name__ == "__main__":
    main()
