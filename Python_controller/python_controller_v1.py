#!/usr/bin/env python3
"""
Improved Python Traffic Light Controller (AV-style braking)
- Keeps STOPLINE_OFFSET = 27.0 (TL head → stop line)
- Uses NEGATED vehicle Y in distance calc (per your convention)
- Starts CRUISING immediately (no blocking wait for V2I)
- Jerk-limited braking, low-speed creep controller (correct sign)
- Latency-aware time-to-reach in decision; smoother eco logic
- Tightened stop latch so we finish at the line, not short
- Smooth nonlinear brake mapping + throttle/brake deadband
- Kinematic equation for low speeds (< 5 m/s) without jerk limiter
- ROS2 simulation time logging

Drop-in replacement for your previous script.
"""

import carla
import socket
import time
import math
import sys
from collections import deque
import numpy as np

# ROS2 imports
try:
    import rclpy
    from rclpy.node import Node
    from rosgraph_msgs.msg import Clock
    ROS2_AVAILABLE = True
    print("[ROS2] ROS2 libraries found - simulation time will be used")
except ImportError:
    ROS2_AVAILABLE = False
    print("[ROS2] ROS2 libraries not found - using system time")


class JerkLimiter:
    def __init__(self, max_jerk_up=1.5, max_jerk_down=1.8, dt=0.01):
        self.max_jerk_up = float(max_jerk_up)
        self.max_jerk_down = float(max_jerk_down)
        self.dt = float(dt)
        self.a_cmd_prev = 0.0

    def step(self, a_target: float) -> float:
        """Slew-limit the commanded acceleration (jerk limiting)."""
        if a_target > self.a_cmd_prev:
            a_cmd = min(self.a_cmd_prev + self.max_jerk_up * self.dt, a_target)
        else:
            a_cmd = max(self.a_cmd_prev - self.max_jerk_down * self.dt, a_target)
        self.a_cmd_prev = a_cmd
        return a_cmd


class ImprovedTrafficLightController:
    def __init__(self, host='localhost', port=2000):
        print("[INIT] Starting Improved Traffic Light Controller")

        # --- CARLA connection ---
        self.client = carla.Client(host, port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        print("[INIT] Connected to CARLA")

        # Get the vehicle
        self.vehicle = None
        self.get_vehicle()

        # --- NS3 connection via intermediate server ---
        self.ns3_socket = None
        self.tl_buffer = ""
        self.connect_to_intermediate_server()
        self.v2i_seen = False  # for one-time log when first in-range packet arrives

        # --- Traffic light data ---
        self.tl_status = 0   # 0=unknown, 1=green, 2=yellow, 3=red
        self.tl_time_remaining = 0.0
        self.tl_position = None
        self.tl_timestamp = 0.0
        self.tl_generation_time = 0.0  # NEW: When the data was generated in NS3
        self.tl_receive_time = 0.0     # NEW: When we received it
        self.v2i_msg = 0     # 1 when recv_time>0 (in range), else 0
        self.have_pose = False  # NEW: True once ANY TL packet seen (even recv_time==0)

        # --- Synchronization parameters ---
        self.MAX_TL_DATA_AGE = 0.5      # Maximum age of traffic light data (500ms)
        self.EXPECTED_V2I_RATE = 10.0   # Expected V2I message rate (10 Hz)
        self.last_v2i_time = 0.0        # Track V2I communication health
        self.v2i_connection_healthy = False

        # --- Vehicle/Control parameters ---
        self.TARGET_SPEED = 13.41  # m/s (cruise speed)
        self.MAX_COMFORT_DECEL = 2.8  # m/s^2 (tunable 2.5–3.5)
        self.MAX_DECEL = 8.0          # m/s^2 (emergency cap)
        self.MAX_ACCEL = 3.0          # m/s^2 (realistic accel)
        self.YELLOW_DURATION = 3.0    # s

        # Map geometry assumptions
        self.STOPLINE_OFFSET = 27.0   # m from TL head to stop line (as requested)
        self.STOP_BIAS = 0.0          # m; keep 0 since we use 27 m offset

        # Timesteps
        self.dt = 0.01                # 10 ms
        self.TIMESTEP = self.dt
        self.latency_tau = 0.05       # Reduce from 120ms to 50ms - was over-compensating

        # State management
        self.committed_action = 0
        self.committed_deceleration = 0.0
        self.committed_velocity = self.TARGET_SPEED
        self.decision_made = False
        self.vehicle_stopped = False
        self.stop_velocity_threshold = 0.05
        self.hold_stop_deceleration = -0.05

        # Tracking
        self.step_counter = 0
        self.previous_velocity = 0.0
        self.previous_distance = 999.0
        self.actual_deceleration = 0.0

        # Decision parameters
        self.SMALL_ACCELERATION = 0.15  # m/s^2
        self.ECO_DECELERATION = 0.5     # m/s^2

        # Velocity smoothing - reduce lag
        self.velocity_history = deque(maxlen=3)  # Reduce from 5 to 3 for less lag

        # PID controller for cruise control
        self.pid_kp = 1.0
        self.pid_ki = 0.2
        self.pid_kd = 0.1
        self.pid_integral = 0.0
        self.pid_previous_error = 0.0
        self.pid_integral_max = 5.0

        # Jerk limiter for accel commands - make more responsive for stopping
        self.jerk_limiter = JerkLimiter(max_jerk_up=1.5, max_jerk_down=3.0, dt=self.dt)  # Increase jerk_down

        # --- ROS2 time integration ---
        self.ros_node = None
        self.sim_time = None
        if ROS2_AVAILABLE:
            self.init_ros2_clock()

        # --- Data logging for paper figures ---
        self.data_log = []
        self.previous_a_cmd = 0.0  # For jerk calculation
        self.log_counter = 0
        self.setup_data_logging()

    def init_ros2_clock(self):
        """Initialize ROS2 node and clock subscription"""
        try:
            rclpy.init()
            self.ros_node = rclpy.create_node('traffic_light_controller_clock')
            self.sim_time = None
            self.clock_subscriber = self.ros_node.create_subscription(
                Clock, '/clock', self.clock_callback, 10)
            print("[ROS2] Clock subscriber initialized - using simulation time")
        except Exception as e:
            print(f"[ROS2-ERROR] Failed to initialize: {e}")
            self.ros_node = None

    def clock_callback(self, msg):
        """Callback for /clock topic"""
        self.sim_time = msg.clock

    def get_ros2_time_seconds(self):
        """Get current ROS2 simulation time in seconds"""
        if self.sim_time:
            return self.sim_time.sec + self.sim_time.nanosec / 1e9
        return None

    def spin_ros2_once(self):
        """Process ROS2 callbacks once (non-blocking)"""
        if self.ros_node:
            rclpy.spin_once(self.ros_node, timeout_sec=0.0)

    def get_current_time(self):
        """Get current time - system time"""
        return time.time()

    def setup_data_logging(self):
        """Setup data logging for paper figures"""
        import os
        
        # Create results directory if it doesn't exist
        self.results_dir = "/home/hnh21/iotav/cosim/matlab_simulink_model/results_python/com"
        os.makedirs(self.results_dir, exist_ok=True)
        
        # Find next available result number
        self.result_number = 1
        while os.path.exists(f"{self.results_dir}/result_com_{self.result_number}.json"):
            self.result_number += 1
        
        self.log_filename = f"{self.results_dir}/result_com_{self.result_number}.json"
        print(f"[DATA-LOG] Will save results to: {self.log_filename}")
        
        # Initialize metadata
        self.log_metadata = {
            'controller_version': 'improved_with_v2i',
            'target_speed': self.TARGET_SPEED,
            'max_comfort_decel': self.MAX_COMFORT_DECEL,
            'max_decel': self.MAX_DECEL,
            'stopline_offset': self.STOPLINE_OFFSET,
            'dt': self.dt,
            'start_time': time.time(),
            'description': 'V2I-enabled controller with kinematic low-speed control'
        }

    def log_data_point(self, velocity, a_cmd, distance_to_stopline, ros2_time, current_time, 
                       tl_status, v2i_msg, final_action, debug_state):
        """Log a single data point for analysis"""
        
        # Calculate jerk from acceleration change
        jerk = (a_cmd - self.previous_a_cmd) / self.dt if self.step_counter > 1 else 0.0
        
        data_point = {
            'step': self.step_counter,
            'ros2_time': ros2_time,
            'system_time': current_time,
            'velocity': velocity,
            'acceleration': a_cmd,
            'deceleration': -a_cmd if a_cmd < 0 else 0.0,  # Positive deceleration for plotting
            'jerk': jerk,
            'distance_to_stopline': distance_to_stopline,
            'traffic_light_status': tl_status,
            'v2i_msg': v2i_msg,
            'final_action': final_action,
            'debug_state': debug_state,
            'vehicle_stopped': self.vehicle_stopped,
            'decision_made': self.decision_made,
            'committed_action': self.committed_action
        }
        
        self.data_log.append(data_point)
        self.log_counter += 1
        
        # Log progress every 1000 steps (10 seconds)
        if self.log_counter % 1000 == 0:
            print(f"[DATA-LOG] Logged {self.log_counter} data points")

    def save_results(self):
        """Save logged data to JSON file"""
        import json
        
        results = {
            'metadata': self.log_metadata,
            'data': self.data_log,
            'summary': {
                'total_steps': len(self.data_log),
                'duration_seconds': len(self.data_log) * self.dt,
                'final_distance_to_stopline': self.data_log[-1]['distance_to_stopline'] if self.data_log else None,
                'min_distance_achieved': min([d['distance_to_stopline'] for d in self.data_log]) if self.data_log else None,
                'max_deceleration': max([d['deceleration'] for d in self.data_log]) if self.data_log else None,
                'max_jerk': max([abs(d['jerk']) for d in self.data_log]) if self.data_log else None,
                'vehicle_stopped': self.vehicle_stopped
            }
        }
        
        try:
            with open(self.log_filename, 'w') as f:
                json.dump(results, f, indent=2)
            print(f"[DATA-LOG] Results saved to: {self.log_filename}")
            print(f"[DATA-LOG] Total data points: {len(self.data_log)}")
            print(f"[DATA-LOG] Duration: {len(self.data_log) * self.dt:.1f} seconds")
            if results['summary']['final_distance_to_stopline'] is not None:
                print(f"[DATA-LOG] Final distance to stop line: {results['summary']['final_distance_to_stopline']:.3f} m")
        except Exception as e:
            print(f"[DATA-LOG-ERROR] Failed to save results: {e}")

    # ------------------- Perception/Geometry Helpers -------------------
    def get_vehicle(self, timeout_s: float = 10.0):
        """Find (or spawn) the ego vehicle and cache its front bumper extent."""
        start = time.time()
        hero_names = {"hero", "ego", "vehicle_ego"}

        def find_existing():
            actors = self.world.get_actors()
            vehicles = actors.filter('vehicle.*')
            heroes = [v for v in vehicles if v.attributes.get('role_name', '').lower() in hero_names]
            if heroes:
                return heroes[0]
            return vehicles[0] if len(vehicles) > 0 else None

        veh = None
        while time.time() - start < timeout_s:
            veh = find_existing()
            if veh is not None:
                break
            try:
                self.world.wait_for_tick(timeout=1.0)
            except:  # world may be paused; just sleep briefly
                time.sleep(0.2)

        if veh is None:
            print("[INIT] No vehicle found; spawning an ego vehicle...")
            veh = self._spawn_ego_vehicle()
            if veh is None:
                print("[ERROR] Failed to spawn an ego vehicle; world has no free spawn points.")
                sys.exit(1)

        self.vehicle = veh
        print(f"[INIT] Using vehicle: {self.vehicle.type_id} (ID: {self.vehicle.id}, role_name={self.vehicle.attributes.get('role_name','')})")
        self.vehicle_extent = float(self.vehicle.bounding_box.extent.x)
        print(f"[INIT] Vehicle front extent: {self.vehicle_extent:.2f} m")


    def connect_to_intermediate_server(self):
        try:
            self.ns3_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.ns3_socket.connect(('localhost', 9000))
            self.ns3_socket.setblocking(False)
            print("[INIT] Connected to intermediate server on port 9000")
        except Exception as e:
            print(f"[ERROR] Failed to connect to intermediate server: {e}")
            sys.exit(1)

    def receive_traffic_light_data(self):
        """Read any available bytes from the intermediate server and parse complete lines.
        Line format: recv_time,status,time_remaining,tl_x,tl_y,tl_z\\n
        We ALWAYS store tl_position (even if recv_time==0), set have_pose=True.
        We set v2i_msg=1 only when recv_time>0 (in-range).
        """
        current_time = time.time()
        
        try:
            chunk = self.ns3_socket.recv(1024).decode()
            if chunk:
                self.tl_buffer += chunk
                while '\n' in self.tl_buffer:
                    line, self.tl_buffer = self.tl_buffer.split('\n', 1)
                    line = line.strip()
                    if not line:
                        continue
                    parts = line.split(',')
                    if len(parts) == 6:
                        recv_time = float(parts[0])
                        self.tl_status = int(parts[1])
                        self.tl_time_remaining = float(parts[2])
                        tl_x = float(parts[3]); tl_y = float(parts[4]); tl_z = float(parts[5])
                        
                        # Enhanced timestamping for synchronization
                        self.tl_position = carla.Location(x=tl_x, y=tl_y, z=tl_z)
                        self.have_pose = True
                        self.tl_receive_time = current_time  # When WE received it
                        self.tl_generation_time = current_time - recv_time  # Estimate when NS3 generated it
                        self.tl_timestamp = current_time  # Legacy compatibility
                        
                        # V2I communication health tracking
                        if recv_time > 0.0:
                            self.v2i_msg = 1
                            self.last_v2i_time = current_time
                            self.v2i_connection_healthy = True
                            
                            if not self.v2i_seen:
                                print(f"[V2I] First in-range TL packet: pos=({tl_x:.2f},{tl_y:.2f},{tl_z:.2f}) status={self.tl_status} t_rem={self.tl_time_remaining:.2f}s")
                                self.v2i_seen = True
                        else:
                            self.v2i_msg = 0
                            
        except BlockingIOError:
            pass
        except Exception as e:
            if 'Resource temporarily unavailable' not in str(e):
                print(f"[ERROR] Receiving data: {e}")
        
        # Check V2I connection health
        if self.v2i_connection_healthy:
            time_since_last_v2i = current_time - self.last_v2i_time
            if time_since_last_v2i > (2.0 / self.EXPECTED_V2I_RATE):  # 2x expected period
                self.v2i_connection_healthy = False
                print(f"[WARNING] V2I connection lost - no data for {time_since_last_v2i:.1f}s")

    def is_traffic_light_data_valid(self):
        """Check if current traffic light data is fresh enough to trust."""
        if not self.have_pose:
            return False
            
        current_time = time.time()
        data_age = current_time - self.tl_receive_time
        
        if data_age > self.MAX_TL_DATA_AGE:
            if self.step_counter % 100 == 0:  # Log every second
                print(f"[SYNC-WARNING] TL data too old: {data_age:.3f}s > {self.MAX_TL_DATA_AGE:.3f}s")
            return False
            
        return True
    
    def get_synchronized_data_snapshot(self):
        """Get a synchronized snapshot of all sensor data with timestamps."""
        current_time = time.time()
        
        # Get fresh vehicle data
        velocity = self.get_smoothed_velocity()
        distance_to_stopline = self.calculate_distance_to_stopline()
        
        # Check traffic light data validity
        tl_data_valid = self.is_traffic_light_data_valid()
        tl_data_age = current_time - self.tl_receive_time if self.have_pose else 999.0
        
        # TEMPORAL COMPENSATION: Adjust traffic light data for age
        compensated_tl_time_remaining = self.tl_time_remaining
        if tl_data_valid and tl_data_age > 0.01:  # Only compensate if age > 10ms
            # Subtract the data age from remaining time (it's been counting down)
            compensated_tl_time_remaining = max(0.0, self.tl_time_remaining - tl_data_age)
            
            if self.step_counter % 100 == 0:  # Log every second
                print(f"[TEMPORAL-COMP] TL data age: {tl_data_age:.3f}s")
                print(f"  → Original time_remaining: {self.tl_time_remaining:.3f}s")
                print(f"  → Compensated time_remaining: {compensated_tl_time_remaining:.3f}s")
        
        snapshot = {
            'timestamp': current_time,
            'velocity': velocity,
            'distance': distance_to_stopline,
            'tl_status': self.tl_status if tl_data_valid else 0,  # Unknown if stale
            'tl_time_remaining': compensated_tl_time_remaining,  # COMPENSATED VALUE
            'tl_time_remaining_raw': self.tl_time_remaining,     # Original for debugging
            'v2i_msg': self.v2i_msg if tl_data_valid else 0,
            'tl_data_age': tl_data_age,
            'v2i_healthy': self.v2i_connection_healthy,
            'data_valid': tl_data_valid,
            'temporal_compensation_applied': tl_data_age > 0.01 and tl_data_valid
        }
        
        return snapshot

    def predict_future_vehicle_state(self, current_velocity, current_distance, prediction_time):
        """Predict where the vehicle will be after prediction_time seconds.
        Uses current acceleration trend to improve prediction accuracy.
        """
        if prediction_time <= 0:
            return current_velocity, current_distance
        
        # Estimate current acceleration from recent velocity history
        if hasattr(self, 'previous_velocity') and hasattr(self, 'dt'):
            estimated_accel = (current_velocity - self.previous_velocity) / self.dt
            # Clamp to reasonable bounds
            estimated_accel = max(-self.MAX_DECEL, min(self.MAX_ACCEL, estimated_accel))
        else:
            estimated_accel = 0.0  # No acceleration data available
        
        # Predict future state using kinematics
        # v_future = v_current + a*t
        # d_future = d_current - (v_current*t + 0.5*a*t²)  # negative because distance decreases
        
        future_velocity = current_velocity + estimated_accel * prediction_time
        future_velocity = max(0.0, future_velocity)  # Can't go backwards
        
        distance_traveled = current_velocity * prediction_time + 0.5 * estimated_accel * prediction_time**2
        future_distance = current_distance - distance_traveled
        
        return future_velocity, future_distance

    def get_temporally_aligned_data(self):
        """Get data aligned to the same temporal reference point.
        Compensates for NS3 data latency by predicting current vehicle state
        at the time when the NS3 data was actually generated.
        """
        snapshot = self.get_synchronized_data_snapshot()
        
        if not snapshot['data_valid'] or snapshot['tl_data_age'] < 0.02:
            # No compensation needed for fresh data
            return snapshot
        
        # The NS3 data is old - predict where we WERE when that data was generated
        data_age = snapshot['tl_data_age']
        
        # Predict vehicle state backwards in time to match NS3 data timestamp
        past_velocity, past_distance = self.predict_future_vehicle_state(
            snapshot['velocity'], 
            snapshot['distance'], 
            -data_age  # Negative time = go backwards
        )
        
        # Create aligned snapshot using past vehicle state with current NS3 data
        aligned_snapshot = snapshot.copy()
        aligned_snapshot.update({
            'velocity': past_velocity,
            'distance': past_distance,
            'tl_time_remaining': self.tl_time_remaining,  # Use original NS3 time (not compensated)
            'temporal_alignment_applied': True,
            'alignment_offset': data_age
        })
        
        if self.step_counter % 100 == 0:
            print(f"[TEMPORAL-ALIGN] Data age: {data_age:.3f}s")
            print(f"  → Current: v={snapshot['velocity']:.2f}, d={snapshot['distance']:.2f}")
            print(f"  → Aligned: v={past_velocity:.2f}, d={past_distance:.2f}")
        
        return aligned_snapshot

    def get_smoothed_velocity(self):
        v = self.vehicle.get_velocity()
        speed_raw = math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
        
        # Use RAW velocity - no filtering to avoid lag and inconsistencies
        # CARLA provides the actual vehicle velocity, which is what we need for physics
        return float(speed_raw)

    def calculate_distance_to_stopline(self):
        """Euclidean distance to TL, minus 27m offset only.
        Uses STOPLINE_OFFSET = 27.0 and NEGATES vehicle.y as per your convention.
        Works even before in-range, as long as we've received at least one packet.
        """
        vehicle_location = self.vehicle.get_location()

        if not self.have_pose or not self.tl_position:
            # No TL pose seen yet at all
            return 999.0

        dx = self.tl_position.x - vehicle_location.x
        dy = self.tl_position.y - (-vehicle_location.y)  # negated Y by request
        distance_to_tl = math.hypot(dx, dy)
        
        # Only subtract the 27m offset (TL head to stop line)
        # Don't subtract vehicle_extent here - that's handled in the control logic
        distance_to_stopline = distance_to_tl - self.STOPLINE_OFFSET

        return max(distance_to_stopline, 0.0)

    def calculate_jerk_aware_stopping_decel(self, velocity, distance_to_stopline):
        """Calculate deceleration needed to stop at the line, accounting for jerk limiter.
        This ensures we can actually achieve the calculated deceleration given jerk constraints.
        """
        if distance_to_stopline <= 0.1:
            return -1.0  # Emergency braking when very close
        
        # Standard physics calculation
        a_physics = -(velocity * velocity) / (2.0 * distance_to_stopline)
        
        # Check if we can achieve this deceleration given jerk limits
        a_current = self.jerk_limiter.a_cmd_prev
        max_jerk_change = self.jerk_limiter.max_jerk_down * self.dt
        a_achievable = a_current - max_jerk_change
        
        # If physics target is too aggressive for jerk limits, use achievable deceleration
        if a_physics < a_achievable:
            a_target = a_achievable
        else:
            a_target = a_physics
        
        # Apply comfort and monotonic constraints
        a_target = max(a_target, -self.MAX_COMFORT_DECEL)
        a_target = min(a_target, -0.1)  # Always decelerating
        
        return a_target

    # ------------------- Planning/Decision -------------------
    def traffic_light_decision(self, vehicle_velocity, distance_to_stopline,
                               traffic_status, remaining_time, v2i_msg):
        """Implements your exact decision rules, with latency-aware t_to_reach.
        Returns (action, target_decel, target_velocity):
          action: 0=cruise, 1=eco slow, 2=stop-at-line
        """
        action = 0; target_deceleration = 0.0; target_velocity = self.TARGET_SPEED

        # Launch guard: if we start near the line and almost stopped, go.
        if distance_to_stopline < 1.0 and vehicle_velocity < 0.5:
            return 0, 0.0, self.TARGET_SPEED

        decision_point_distance = (vehicle_velocity ** 2) / (2.0 * self.MAX_COMFORT_DECEL)

        # NEW: require we at least have a pose; then require in-range to commit braking
        if (not self.have_pose) or (distance_to_stopline > decision_point_distance) or (v2i_msg == 0):
            return 0, 0.0, self.TARGET_SPEED  # cruise

        # Latency-aware time to reach
        v_eps = 0.10
        tau = self.latency_tau
        t_to_reach = max((distance_to_stopline - vehicle_velocity * tau), 0.0) / max(vehicle_velocity, v_eps)

        # --- RED ---
        if traffic_status == 3:
            if remaining_time < t_to_reach:
                # Will turn GREEN before arrival → eco slow (no full stop)
                a_eco = -self.ECO_DECELERATION
                v_arrive = distance_to_stopline / max(remaining_time + 0.5, 0.5)
                v_min_abs = 2.0
                v_min = max(v_min_abs, 0.5 * max(vehicle_velocity, v_eps))
                target_velocity = max(min(v_arrive, vehicle_velocity), v_min)
                return 1, a_eco, target_velocity
            else:
                # Still RED at arrival → stop at the line - use jerk-aware physics
                a_req = -(vehicle_velocity ** 2) / (2.0 * max(distance_to_stopline, 0.05))
                a_req = max(a_req, -self.MAX_DECEL)
                return 2, a_req, 0.0

        # --- YELLOW ---
        if traffic_status == 2:
            if remaining_time < t_to_reach:
                a_req = -(vehicle_velocity ** 2) / (2.0 * max(distance_to_stopline, 0.05))
                a_req = max(a_req, -self.MAX_DECEL)
                return 2, a_req, 0.0
            else:
                return 0, 0.0, self.TARGET_SPEED

        # --- GREEN ---
        if traffic_status == 1:
            if (remaining_time + self.YELLOW_DURATION) < t_to_reach:
                a_req = -(vehicle_velocity ** 2) / (2.0 * max(distance_to_stopline, 0.05))
                a_req = max(a_req, -self.MAX_DECEL)
                return 2, a_req, 0.0
            else:
                return 0, 0.0, self.TARGET_SPEED

        # Unknown status → cruise
        return 0, 0.0, self.TARGET_SPEED

    # ------------------- AV-style Braking Helpers -------------------
    def required_decel_to_stop(self, v, d):
        if d <= 0.05:
            return -self.MAX_DECEL
        # Use same physics as decision layer for consistency
        a_req = -(v * v) / (2.0 * max(d, 0.05))  # Changed from 4.0 to 2.0
        return max(a_req, -self.MAX_COMFORT_DECEL)

    def low_speed_distance_controller(self, v, d):
        """PD on distance for the last ~1–12 m. Returns accel (m/s^2).
        Sign chosen so positive distance → positive accel (creep forward).
        """
        Kp_d = 0.8
        Kd_d = 1.2
        a = +Kp_d * d - Kd_d * v
        return max(min(a, 0.8), -2.5)

    def decel_to_brake_cmd(self, a_des):
        """Smooth nonlinear map from desired decel (negative) to brake [0..1]."""
        if a_des >= -0.05:
            return 0.0
        a = abs(a_des)
        if a <= 1.0:
            b = 0.08 + 0.12 * (a / 1.0)            # 0.08 → 0.20
        elif a <= 3.0:
            b = 0.20 + 0.35 * ((a - 1.0) / 2.0)    # 0.20 → 0.55
        elif a <= 6.0:
            b = 0.55 + 0.35 * ((a - 3.0) / 3.0)    # 0.55 → 0.90
        else:
            b = 1.0
        return max(0.0, min(1.0, b))

    # ------------------- Controllers -------------------
    def cruise_control(self, current_velocity, brake_cmd=0.0):
        """Speed PID with basic anti-windup and derivative on measurement."""
        error = self.TARGET_SPEED - current_velocity
        # anti-windup: decay integral when braking or near target
        if brake_cmd > 0.02 or abs(error) < 0.1 or current_velocity < 0.5:
            self.pid_integral *= 0.9
        else:
            self.pid_integral += error * self.dt
            self.pid_integral = max(-self.pid_integral_max, min(self.pid_integral_max, self.pid_integral))
        p_term = self.pid_kp * error
        i_term = self.pid_ki * self.pid_integral
        d_term = self.pid_kd * ((error - self.pid_previous_error) / self.dt)
        self.pid_previous_error = error
        target_accel = p_term + i_term + d_term
        # comfort bounds during cruise
        if target_accel > 0:
            target_accel = min(target_accel, self.MAX_ACCEL)
        else:
            target_accel = max(target_accel, -1.5)
        return target_accel

    def acceleration_to_throttle_brake(self, a_cmd):
        """Deadbanded split: throttle for accel, brake for decel."""
        throttle = 0.0; brake = 0.0
        
        if a_cmd > 0.05:
            throttle = max(0.0, min(a_cmd / self.MAX_ACCEL, 1.0))
            brake = 0.0
        elif a_cmd < -0.05:
            throttle = 0.0
            brake = self.decel_to_brake_cmd(a_cmd)
        else:
            throttle = 0.0; brake = 0.0
        
        return throttle, brake

    def reset_state(self):
        self.committed_action = 0
        self.committed_deceleration = 0.0
        self.committed_velocity = self.TARGET_SPEED
        self.decision_made = False
        self.vehicle_stopped = False
        self.pid_integral = 0.0
        self.pid_previous_error = 0.0

    def try_manual_vehicle_kick(self):
        """Try to force the vehicle to respond by applying a strong manual control command."""
        print("[MANUAL-KICK] Attempting to force vehicle response...")
        
        # Try setting the vehicle to manual gear and apply strong throttle
        control = carla.VehicleControl()
        control.throttle = 1.0
        control.brake = 0.0
        control.steer = 0.0
        control.hand_brake = False
        control.reverse = False
        control.manual_gear_shift = True
        control.gear = 1
        
        # Apply for several frames
        for i in range(10):
            self.vehicle.apply_control(control)
            time.sleep(0.02)
            v = self.vehicle.get_velocity()
            speed = math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
            print(f"[MANUAL-KICK] Frame {i+1}: speed={speed:.3f} m/s")
            if speed > 0.5:
                print("[MANUAL-KICK] Vehicle responding! Switching back to automatic.")
                break
        
        # Reset to automatic gear
        control.manual_gear_shift = False
        self.vehicle.apply_control(control)

    # ------------------- State Manager -------------------
    def state_manager_step(self, decision_action, decision_decel, decision_vel,
                           vehicle_velocity, traffic_status, remaining_time, distance_to_stopline):
        self.step_counter += 1
        if self.step_counter > 1:
            self.actual_deceleration = (vehicle_velocity - self.previous_velocity) / self.dt
        final_action = 0; final_decel = 0.0; final_vel = vehicle_velocity; debug_state = 1
        decision_point_distance = (vehicle_velocity ** 2) / (2 * self.MAX_COMFORT_DECEL)
        at_decision_point = distance_to_stopline <= decision_point_distance
        if (not self.decision_made) and at_decision_point and decision_action != 0:
            self.committed_action = decision_action
            self.committed_deceleration = decision_decel
            self.committed_velocity = decision_vel
            self.decision_made = True
            debug_state = 10 + self.committed_action
            print(f"[COMMIT] act={self.committed_action} decel={self.committed_deceleration:.3f} v_tgt={self.committed_velocity:.3f}")
        if self.decision_made and self.committed_action == 2:
            # Latch to full stop only when we're truly at the line
            if vehicle_velocity <= self.stop_velocity_threshold and distance_to_stopline <= 0.5:
                if not self.vehicle_stopped:
                    print(f"[STOPPED] d={distance_to_stopline:.3f} v={vehicle_velocity:.3f}")
                self.vehicle_stopped = True
                final_action = 2; final_decel = self.hold_stop_deceleration; final_vel = 0.0; debug_state = 20
            else:
                final_action = 2; final_decel = 0.0; final_vel = vehicle_velocity; debug_state = 21
        elif self.decision_made and self.committed_action == 1:
            target_reached = abs(vehicle_velocity - self.committed_velocity) < 0.5
            conditions_changed = (traffic_status != 1)
            if target_reached or conditions_changed:
                print("[ACCEL-END] Target reached or conditions changed")
                self.reset_state()
                final_action = 0; final_decel = 0.0; final_vel = vehicle_velocity; debug_state = 31
            else:
                final_action = 1; final_decel = self.committed_deceleration; final_vel = self.committed_velocity; debug_state = 22
        else:
            final_action = 0; final_decel = 0.0; final_vel = vehicle_velocity; debug_state = 1
        self.previous_velocity = vehicle_velocity
        self.previous_distance = distance_to_stopline
        return final_action, final_decel, final_vel, debug_state

    # ------------------- Main Loop -------------------
    def control_loop(self):
        print("\n[START] Improved Traffic Light Controller with Enhanced Synchronization")
        print(f"[CONFIG] v_target={self.TARGET_SPEED:.2f} m/s, a_comf={self.MAX_COMFORT_DECEL:.1f}, a_max={self.MAX_DECEL:.1f}")
        print(f"[SYNC] Max TL data age: {self.MAX_TL_DATA_AGE:.1f}s, Expected V2I rate: {self.EXPECTED_V2I_RATE:.1f}Hz\n")
        
        # Start cruising immediately; just probe briefly for V2I and then continue polling while driving
        print("Probing for traffic light data (2s max), then cruising while polling...")
        t0 = time.time()
        while (time.time() - t0) < 2.0:
            self.receive_traffic_light_data()
            time.sleep(0.05)
        
        # Try to kick-start the vehicle if it seems stuck
        initial_velocity = self.get_smoothed_velocity()
        if initial_velocity < 0.1:
            print("[INIT] Vehicle appears stationary, attempting manual kick-start...")
            self.try_manual_vehicle_kick()
        
        print("\n--- Control Loop: Distance | Speed | Accel | CARLA_Speed ---")
        
        try:
            while True:
                # Keep polling V2I in the background
                self.receive_traffic_light_data()
                
                # Process ROS2 callbacks to get latest simulation time
                if ROS2_AVAILABLE:
                    self.spin_ros2_once()
                
                # GET TEMPORALLY ALIGNED DATA - compensates for NS3 latency
                data_snapshot = self.get_temporally_aligned_data()
                
                # Extract synchronized values
                velocity = data_snapshot['velocity']
                distance_to_stopline = data_snapshot['distance']
                tl_status = data_snapshot['tl_status']
                tl_time_remaining = data_snapshot['tl_time_remaining']
                v2i_msg = data_snapshot['v2i_msg']
                tl_data_age = data_snapshot['tl_data_age']
                data_valid = data_snapshot['data_valid']
                
                # Get current time (system time)
                current_time = self.get_current_time()
                ros2_time = self.get_ros2_time_seconds()
                
                # Planner-level decision (pass/eco/stop) - using synchronized data
                decision_action, decision_decel, decision_vel = self.traffic_light_decision(
                    velocity, distance_to_stopline, tl_status, tl_time_remaining, v2i_msg)
                
                # State manager
                final_action, final_decel, final_vel, debug_state = self.state_manager_step(
                    decision_action, decision_decel, decision_vel,
                    velocity, tl_status, tl_time_remaining, distance_to_stopline)
                
                # --- Inner control: AV-style braking & cruise ---
                if final_action == 2:  # Stopping path
                    if velocity <= 0.01:
                        # Vehicle has stopped
                        if not self.vehicle_stopped:
                            self.vehicle_stopped = True
                        a_cmd = 0.0  # No acceleration command when stopped
                        # Reset jerk limiter to avoid buildup
                        self.jerk_limiter.a_cmd_prev = 0.0
                    elif velocity < 5.0:
                        # LOW SPEED: Apply kinematic equation directly without jerk limiter
                        if distance_to_stopline <= 0.1:
                            a_cmd = -1.0  # Emergency when very close
                            control_mode = "EMERGENCY"
                        else:
                            # Pure kinematic: a = -v²/(2*d)
                            a_kinematic = -(velocity * velocity) / (2.0 * distance_to_stopline)
                            # Apply reasonable bounds
                            a_cmd = max(a_kinematic, -self.MAX_COMFORT_DECEL)
                            a_cmd = min(a_cmd, -0.05)  # Always some deceleration
                            control_mode = "KINEMATIC"
                        
                        # Update jerk limiter state for consistency when switching back to high speed
                        self.jerk_limiter.a_cmd_prev = a_cmd
                        
                        # LOG DIRECT KINEMATIC CONTROL - EVERY TIMESTEP WHEN v < 5 m/s
                        print(f"[LOW-SPEED-KINEMATIC] Step {self.step_counter:5d}: d={distance_to_stopline:6.3f}m | v={velocity:5.3f}m/s | a_direct={a_cmd:+6.3f}m/s² | mode={control_mode}")
                    else:
                        # HIGH SPEED: Use jerk-aware stopping calculation
                        a_tgt = self.calculate_jerk_aware_stopping_decel(velocity, distance_to_stopline)
                        # Apply jerk limiting
                        a_cmd = self.jerk_limiter.step(a_tgt)
                        
                        # LOG HIGH SPEED JERK-LIMITED CONTROL (less frequent)
                        if self.step_counter % 10 == 0:  # Every 100ms for high speed
                            print(f"[HIGH-SPEED-JERK] Step {self.step_counter:5d}: d={distance_to_stopline:6.3f}m | v={velocity:5.3f}m/s | a_target={a_tgt:+6.3f}m/s² | a_applied={a_cmd:+6.3f}m/s²")
                
                elif final_action == 1:  # Eco/accel path
                    if final_decel != 0.0:
                        a_cmd = self.jerk_limiter.step(final_decel)
                    else:
                        vel_error = final_vel - velocity
                        a_target = 0.5 * vel_error
                        a_cmd = self.jerk_limiter.step(a_target)
                
                else:  # Cruise
                    cruise_accel = self.cruise_control(velocity)
                    a_cmd = self.jerk_limiter.step(cruise_accel)
                    # anti-stiction nudge to ensure we roll off from rest
                    if velocity < 0.2:
                        a_cmd = max(a_cmd, 0.3)
                
                throttle, brake = self.acceleration_to_throttle_brake(a_cmd)
                
                control = carla.VehicleControl()
                control.throttle = float(throttle)
                control.brake = float(brake)
                control.manual_gear_shift = False
                control.hand_brake = bool(self.vehicle_stopped)
                
                self.vehicle.apply_control(control)
                
                # Get CARLA-reported speed for comparison
                carla_velocity = self.vehicle.get_velocity()
                carla_speed = math.sqrt(carla_velocity.x**2 + carla_velocity.y**2 + carla_velocity.z**2)
                
                # SIMPLIFIED LOGGING - Show ROS2 time when available
                if ros2_time is not None:
                    print(f"Step {self.step_counter:5d}: d={distance_to_stopline:6.2f}m | v={velocity:5.2f}m/s | a={a_cmd:+6.3f}m/s² | carla_v={carla_speed:5.2f}m/s | ros2_t={ros2_time:8.3f}s")
                else:
                    print(f"Step {self.step_counter:5d}: d={distance_to_stopline:6.2f}m | v={velocity:5.2f}m/s | a={a_cmd:+6.3f}m/s² | carla_v={carla_speed:5.2f}m/s | sys_t={current_time:8.3f}s")
                
                # Log data point for paper analysis
                self.log_data_point(velocity, a_cmd, distance_to_stopline, ros2_time, current_time, 
                                    tl_status, v2i_msg, final_action, debug_state)
                
                # Store for next iteration
                self.previous_a_cmd = a_cmd
                
                # time.sleep to maintain loop rate
                time.sleep(self.dt)
                
        except KeyboardInterrupt:
            print("\n[STOP] Shutting down controller...")
        except Exception as e:
            print(f"[ERROR] Control loop: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.save_results()

    def cleanup(self):
        # Save results before cleanup
        self.save_results()
        
        if self.ns3_socket:
            self.ns3_socket.close()
        if self.vehicle:
            control = carla.VehicleControl()
            control.throttle = 0.0
            control.brake = 0.0
            control.hand_brake = False
            self.vehicle.apply_control(control)
        
        # Cleanup ROS2 if initialized
        if self.ros_node:
            try:
                self.ros_node.destroy_node()
                rclpy.shutdown()
            except:
                pass
                
        print("[CLEANUP] Controller cleanup complete")


def main():
    controller = None
    try:
        controller = ImprovedTrafficLightController()
        controller.control_loop()
    except Exception as e:
        print(f"[FATAL] {e}")
        import traceback
        traceback.print_exc()
    finally:
        if controller:
            controller.cleanup()


if __name__ == "__main__":
    main()