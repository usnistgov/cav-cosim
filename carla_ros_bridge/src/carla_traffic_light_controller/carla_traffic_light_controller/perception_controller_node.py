#!/usr/bin/env python3
"""
ROS2 Perception Traffic Light Controller Node for CARLA
Same as original but uses traffic light topic and simplified braking logic
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# ROS2 message types
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from carla_msgs.msg import TrafficLightInfo

import time
import math
import json
from collections import deque
import os
import signal
import atexit


class PerceptionTrafficLightControllerNode(Node):
    def __init__(self):
        super().__init__('perception_traffic_light_controller')
        
        # Declare parameters with defaults matching your python controller
        self.declare_parameter('role_name', 'hero')
        self.declare_parameter('max_speed', 13.41)  # 30 mph in m/s
        self.declare_parameter('max_comfort_decel', 2.8)
        self.declare_parameter('max_decel', 8.0)
        self.declare_parameter('stopline_offset', 27.0)
        self.declare_parameter('control_dt', 0.01)
        
        # Get parameters
        self.role_name = self.get_parameter('role_name').get_parameter_value().string_value
        self.MAX_SPEED = self.get_parameter('max_speed').get_parameter_value().double_value
        self.MAX_COMFORT_DECEL = self.get_parameter('max_comfort_decel').get_parameter_value().double_value
        self.MAX_DECEL = self.get_parameter('max_decel').get_parameter_value().double_value
        self.STOPLINE_OFFSET = self.get_parameter('stopline_offset').get_parameter_value().double_value
        self.dt = self.get_parameter('control_dt').get_parameter_value().double_value
        
        self.get_logger().info(f'Initialized Perception Traffic Light Controller for vehicle: {self.role_name}')
        
        # Vehicle state variables
        self.current_velocity = 0.0
        self.previous_velocity = 0.0
        self.current_position = None
        self.previous_time = time.time()
        
        # Traffic light data from topic
        self.tl_status = 1  # Default: Green (1=Green, 2=Yellow, 3=Red)
        self.tl_distance = 100.0  # Default distance
        self.tl_time_remaining = 0.0
        self.tl_id = 0
        self.have_tl_data = False
        self.have_pose = False
        
        # Control state
        self.committed_action = 0
        self.decision_made = False
        self.vehicle_stopped = False
        self.step_counter = 0
        
        # PID controller parameters for cruise control
        self.pid_kp = 0.5  # Proportional gain
        self.pid_ki = 0.08  # Integral gain  
        self.pid_kd = 0.0 # Derivative gain
        self.pid_integral = 0.0
        self.pid_previous_error = 0.0
        
        # Traffic light decision parameters
        self.YELLOW_DURATION = 3.0  # Assumed yellow light duration
        self.A_COMFORT = 2.6  # Comfort deceleration (between 2.5-2.8)
        
        # Constant decision distance calculation: v²/(2*a_comfort) with v=12.45 m/s
        self.DECISION_DISTANCE = (12.45 * 12.45) / (2.0 * 2.6)  # = 29.84 meters
        
        # Decision commitment to avoid oscillations
        self.decision_committed = False
        self.committed_decision = None  # Will be "CRUISE" or "BRAKE"
        
        # Kinematic braking state - calculate once at decision point
        self.initial_braking_decel = None  # Store the initial calculated deceleration
        self.braking_started = False
        
        # Data logging
        self.data_log = []
        self.start_time = time.time()
        self.actual_acceleration_history = deque(maxlen=10)
        self.max_accel_commanded = 0.0
        self.max_decel_commanded = 0.0  # most negative value stored
        
        # IMU data for actual acceleration measurement - FOCUS ON X-AXIS
        self.imu_acceleration_x = 0.0  # X-axis is longitudinal (forward/backward)
        self.imu_data_timestamps = deque(maxlen=100)  # Track IMU data timing
        
        # Enhanced logging for high acceleration detection
        self.high_accel_events = []
        self.HIGH_ACCEL_THRESHOLD = 10.0  # Log events above 10 m/s²
        
        # Setup QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        control_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        
        # ROS2 Publishers
        self.control_publisher = self.create_publisher(
            Twist,
            f'/carla/{self.role_name}/twist',
            control_qos
        )
        
        # ROS2 Subscribers
        self.odometry_subscriber = self.create_subscription(
            Odometry,
            f'/carla/{self.role_name}/odometry',
            self.odometry_callback,
            sensor_qos
        )
        
        self.clock_subscriber = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            sensor_qos
        )
        
        # IMU subscriber for actual acceleration measurement - FOCUS ON X-AXIS
        self.imu_subscriber = self.create_subscription(
            Imu,
            f'/carla/{self.role_name}/imu',
            self.imu_callback,
            sensor_qos
        )
        
        # Traffic light subscriber (CHANGE 1: Use topic instead of intermediate server)
        self.traffic_light_subscriber = self.create_subscription(
            TrafficLightInfo,
            f'/carla/{self.role_name}/traffic_light_in_front',
            self.traffic_light_callback,
            sensor_qos
        )
        
        # Control timer
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        
        # Simulation time
        self.sim_time = None
        
        self.get_logger().info('Perception Traffic Light Controller Node initialized successfully')
        
        # Register graceful shutdown hooks to ensure results are saved on Ctrl+C and termination
        self._cleanup_done = False
        def _signal_handler(signum, frame):
            try:
                self.get_logger().info(f'Received signal {signum}, saving results...')
            except Exception:
                pass
            try:
                self.cleanup()
            finally:
                # Re-raise default behavior
                signal.signal(signum, signal.SIG_DFL)
                os.kill(os.getpid(), signum)
        try:
            signal.signal(signal.SIGINT, _signal_handler)
            signal.signal(signal.SIGTERM, _signal_handler)
        except Exception:
            pass
        
        def _on_exit():
            try:
                self.cleanup()
            except Exception:
                pass
        atexit.register(_on_exit)

    def odometry_callback(self, msg):
        """Callback for vehicle odometry updates"""
        # Extract velocity magnitude from twist
        linear_vel = msg.twist.twist.linear
        self.current_velocity = math.sqrt(
            linear_vel.x**2 + linear_vel.y**2 + linear_vel.z**2
        )
        
        # Store position for distance calculations
        self.current_position = msg.pose.pose.position

    def clock_callback(self, msg):
        """Callback for simulation clock"""
        self.sim_time = msg.clock

    def imu_callback(self, msg):
        """Callback for IMU data - FOCUS ON X-AXIS ACCELERATION"""
        current_time = time.time()
        
        # Store only X-axis acceleration (longitudinal)
        self.imu_acceleration_x = msg.linear_acceleration.x
        
        # Log timestamp for analysis
        self.imu_data_timestamps.append(current_time)
        
        # Detect and log high acceleration events immediately
        if abs(self.imu_acceleration_x) > self.HIGH_ACCEL_THRESHOLD:
            high_accel_event = {
                'timestamp': current_time,
                'step': self.step_counter,
                'imu_x_acceleration': self.imu_acceleration_x,
                'velocity': self.current_velocity,
                'ros2_time': self.get_ros2_time_seconds()
            }
            self.high_accel_events.append(high_accel_event)
            
            self.get_logger().warn(
                f'HIGH X-AXIS ACCELERATION DETECTED: {self.imu_acceleration_x:.2f} m/s² '
                f'at step {self.step_counter}, velocity {self.current_velocity:.2f} m/s'
            )

    def traffic_light_callback(self, msg):
        """Callback for traffic light data from topic (CHANGE 1)"""
        # Extract traffic light information directly from message
        self.tl_id = msg.id
        self.tl_distance = msg.distance
        status_str = msg.status
        self.tl_time_remaining = msg.time_remaining
        
        # Convert status string to numeric value
        if status_str == "Red":
            self.tl_status = 3
        elif status_str == "Yellow":
            self.tl_status = 2
        else:  # Green or any other status
            self.tl_status = 1
        
        self.have_tl_data = True

    def get_ros2_time_seconds(self):
        """Get current ROS2 simulation time in seconds"""
        if self.sim_time:
            return self.sim_time.sec + self.sim_time.nanosec / 1e9
        return None

    def get_distance_to_stopline(self):
        """Get distance to stopline from traffic light topic (CHANGE 1)"""
        if self.have_tl_data:
            return self.tl_distance
        else:
            return 100.0  # Default safe distance

    def calculate_actual_acceleration(self):
        """Calculate actual acceleration from velocity change"""
        current_time = time.time()
        dt_actual = current_time - self.previous_time
        
        if dt_actual > 0:
            acceleration = (self.current_velocity - self.previous_velocity) / dt_actual
            self.actual_acceleration_history.append(acceleration)
        else:
            acceleration = 0.0
            
        self.previous_velocity = self.current_velocity
        self.previous_time = current_time
        
        return acceleration

    def apply_vehicle_control(self, acceleration=0.0):
        """Publish acceleration command as Twist message"""
        msg = Twist()
        msg.linear.x = float(acceleration)  # Set acceleration in m/s²
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0  # No steering for traffic light control
        
        self.control_publisher.publish(msg)

    def control_loop(self):
        """Main control loop with simplified braking logic (CHANGE 2)"""
        self.step_counter += 1
        current_time = time.time()
        
        # Wait for vehicle data before starting control
        if not self.current_position or self.step_counter < 10:
            self.apply_vehicle_control(acceleration=0.0)
            return
        
        # Get current state
        velocity = self.current_velocity
        distance_to_stopline = self.get_distance_to_stopline()
        
        # Calculate actual acceleration for logging
        acceleration_actual = self.calculate_actual_acceleration()
        
        # SIMPLIFIED CONTROL LOGIC (CHANGE 2): Only brake when distance ≤ decision distance AND light is red
        if distance_to_stopline <= self.DECISION_DISTANCE and self.tl_status == 3:  # Red light
            # Apply braking for red light
            acceleration_commanded = self.kinematic_brake_control(velocity, distance_to_stopline)
            state = f"BRAKE_RED_LIGHT"
            action = 2
        else:
            # Cruise in all other cases (before decision point, or green/yellow light)
            acceleration_commanded = self.pid_cruise_control(velocity)
            if distance_to_stopline > self.DECISION_DISTANCE:
                state = "CRUISE_BEFORE_DECISION"
            else:
                state = f"CRUISE_TL{self.tl_status}"
            action = 0
        
        # Apply control
        self.apply_vehicle_control(acceleration=acceleration_commanded)
        
        # Check if stopped
        if velocity < 0.1 and distance_to_stopline < 2.0:
            self.vehicle_stopped = True
        
        # ENHANCED LOGGING: Focus on requested metrics
        if self.step_counter % 10 == 0 or distance_to_stopline < 50.0 or abs(self.imu_acceleration_x) > 5.0:
            self.get_logger().info(
                f'[{self.step_counter:4d}] DISTANCE: {distance_to_stopline:.2f}m | '
                f'VELOCITY: {velocity:.2f}m/s | '
                f'ACCEL_COMMANDED: {acceleration_commanded:+.2f}m/s² | '
                f'IMU_X_ACCEL: {self.imu_acceleration_x:+.2f}m/s² | '
                f'TL_STATUS: {self.tl_status} | STATE: {state}'
            )
        
        # Log data point with focus on requested metrics
        self.log_enhanced_data_point(
            velocity=velocity,
            acceleration_commanded=acceleration_commanded,
            acceleration_actual=self.imu_acceleration_x,
            distance_to_stopline=distance_to_stopline,
            ros2_time=self.get_ros2_time_seconds(),
            system_time=current_time,
            tl_status=self.tl_status,
            tl_id=self.tl_id,
            action=action,
            state=state
        )

    def log_enhanced_data_point(self, velocity, acceleration_commanded, acceleration_actual, 
                               distance_to_stopline, ros2_time, system_time, tl_status, tl_id, action, state):
        """Enhanced logging focusing on distance, velocity, applied acceleration, and IMU X-axis"""
        
        # Update running maxima
        if acceleration_commanded > self.max_accel_commanded:
            self.max_accel_commanded = acceleration_commanded
        if acceleration_commanded < self.max_decel_commanded:
            self.max_decel_commanded = acceleration_commanded  # negative values desired
        
        data_point = {
            "step": self.step_counter,
            "ros2_time": ros2_time,
            "system_time": system_time,
            "velocity": velocity,
            "acceleration_commanded": acceleration_commanded,
            "acceleration_actual": acceleration_actual,
            "max_acceleration": self.max_accel_commanded,
            "max_deceleration": self.max_decel_commanded,
            "distance_to_stopline": distance_to_stopline,
            "traffic_light_status": tl_status,
            "traffic_light_id": tl_id,
            "debug_state": state,
        }
        
        self.data_log.append(data_point)
        
        # Additional warning for extremely high values
        if abs(self.imu_acceleration_x) > 20.0:
            self.get_logger().error(
                f'EXTREMELY HIGH IMU X-AXIS ACCELERATION: {self.imu_acceleration_x:.2f} m/s² '
                f'- This appears to be sensor noise or calculation error!'
            )

    def save_results(self):
        """Save enhanced logged data with focus on IMU X-axis analysis"""
        if not self.data_log:
            return
            
        # Ensure output directory exists
        results_dir = "/home/hnh21/iotav/cosim/python_controller/results/perception"
        try:
            os.makedirs(results_dir, exist_ok=True)
        except Exception as e:
            self.get_logger().error(f'Failed to create results directory: {e}')
            return
        
        # Determine next index N for filename results_perc_N.json
        next_index = 1
        try:
            existing = [f for f in os.listdir(results_dir) if f.startswith('results_perc_') and f.endswith('.json')]
            indices = []
            for name in existing:
                try:
                    num_part = name[len('results_perc_'):-len('.json')]
                    indices.append(int(num_part))
                except Exception:
                    pass
            if indices:
                next_index = max(indices) + 1
        except Exception:
            pass
        
        filename = os.path.join(results_dir, f"results_perc_{next_index}.json")
        
        result_data = {
            "data": self.data_log
        }
        
        try:
            with open(filename, 'w') as f:
                json.dump(result_data, f, indent=2)
            self.get_logger().info(f'Results saved to: {filename}')
        except Exception as e:
            self.get_logger().error(f'Failed to save results: {e}')

    def cleanup(self):
        """Cleanup before shutdown"""
        if getattr(self, '_cleanup_done', False):
            return
        self._cleanup_done = True
        self.get_logger().info('Cleaning up Perception Traffic Light Controller Node')
        
        # Stop vehicle
        self.apply_vehicle_control(acceleration=0.0)
                
        # Save results
        self.save_results()

    def pid_cruise_control(self, current_velocity):
        """PID controller to reach target speed of 13.41 m/s"""
        error = self.MAX_SPEED - current_velocity
        
        # PID calculations
        self.pid_integral += error * self.dt
        derivative = (error - self.pid_previous_error) / self.dt
        
        # PID output (acceleration command)
        acceleration = (self.pid_kp * error + 
                       self.pid_ki * self.pid_integral + 
                       self.pid_kd * derivative)
        
        # Store error for next iteration
        self.pid_previous_error = error
        
        # Limit acceleration
        acceleration = max(-2.0, min(3.0, acceleration))
        
        return acceleration

    def kinematic_brake_control(self, velocity, distance_to_stopline):
        """Calculate braking acceleration using kinematic equation: a = -v²/(4d), recomputed each control step"""
        if distance_to_stopline <= 0.5:
            decel = self.MAX_DECEL  # Emergency
        elif velocity <= 0.1:
            decel = 0.001  # Light holding brake
        elif distance_to_stopline > 0.0:
            # Calculate deceleration needed to stop at stopline using current velocity and distance (comfortable braking)
            required_decel = (velocity * velocity) / (4.0 * distance_to_stopline)
            # Limit to comfort deceleration for passenger comfort
            decel = min(required_decel, self.MAX_DECEL)
        else:
            decel = self.MAX_DECEL

        # Periodic debug log to visualize recomputed braking demand
        if self.step_counter % 20 == 0:
            try:
                required_dbg = (velocity * velocity) / (4.0 * max(distance_to_stopline, 1e-6)) if velocity > 0.1 else 0.0
                self.get_logger().info(
                    f'[KINEMATIC] v={velocity:.2f} d={distance_to_stopline:.2f} required={required_dbg:.2f} using={decel:.2f}')
            except Exception:
                pass

        return -decel


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PerceptionTrafficLightControllerNode()
        
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info('Received shutdown signal')
        finally:
            node.cleanup()
            
    except Exception as e:
        print(f"Error starting node: {e}")
    finally:
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
