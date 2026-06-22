#!/usr/bin/env python3
"""
V2X-aware Autonomous Emergency Braking (AEB) Node.

This is the V2X variant of `aeb_node.py`. The baseline `aeb_node.py` is left
untouched so that the no-V2X comparison stays valid — they run in different
configurations and produce independent results.

What this node adds vs. the baseline:
  - Subscribes to /v2x/cam_received (nav_msgs/Odometry) — the most-recent CAM
    delivered by the ns-3 V2P gateway (ped pose+vel as transmitted by the ped).
  - Subscribes to ego's CARLA native odometry to know ego's current pose.
  - Computes a V2X-side TTC and triggers AEB independently of LiDAR.
  - Supports three fusion modes for ablation:
      * 'lidar_only' — V2X is logged but never triggers brake (matches baseline behaviour).
      * 'v2x_only'   — only V2X triggers brake (LiDAR detection is logged but not acted on).
      * 'both'       — either source can trigger brake (default V2X production mode).

Parameters:
    ego_actor_id          (int)               CARLA actor ID of ego vehicle
    cruise_speed_kmh      (double, 40.0)      target cruise speed
    ttc_brake             (double, 1.5)       TTC threshold (s) — applied to BOTH lidar and V2X
    lane_half_width       (double, 1.5)       lateral detection zone for lidar (m)
    min_obstacle_z        (double, -2.5)      lidar z-min (sensor frame)
    max_obstacle_z        (double, 1.0)       lidar z-max (sensor frame)
    fusion_mode           (string, 'both')    'lidar_only' | 'v2x_only' | 'both'
    v2x_lateral_corridor  (double, 2.0)       V2X path-conflict half-width (m)
    v2x_max_age           (double, 0.5)       drop CAMs older than this (s)
"""

import math
import struct
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64, String
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaEgoVehicleControl


class AEBNodeV2X(Node):
    """LiDAR + V2X fused AEB. Logs every brake event with the source that triggered it."""

    def __init__(self):
        super().__init__("aeb_node_v2x")

        # Parameters (mirror baseline)
        self.declare_parameter("ego_actor_id", 0)
        self.declare_parameter("cruise_speed_kmh", 40.0)
        self.declare_parameter("ttc_brake", 1.5)
        self.declare_parameter("lane_half_width", 1.5)
        self.declare_parameter("min_obstacle_z", -2.5)
        self.declare_parameter("max_obstacle_z", 1.0)

        # V2X-specific parameters
        self.declare_parameter("fusion_mode", "both")
        # Corridor wide enough to include both curbs of the crosswalk
        # (~2.9 m curb half-width + ~0.9 m RSU lateral perception bias →
        # ~3.8 m needed; 5.0 m gives margin). This is intentionally
        # generous: V2X's value proposition is "yield to any ped detected
        # near the crosswalk", including ones still waiting on the curb.
        # The scenario uses a time-fallback trigger so a yielding ego
        # doesn't deadlock the ped's distance-based start.
        self.declare_parameter("v2x_lateral_corridor", 5.0)
        # Driving-lane half-width (~2 m). Used only by the release rule
        # in `_v2x_corridor_occupied`: once the ped has crossed past this
        # threshold on the OPPOSITE side from where the brake fired, the
        # ped has "finished crossing the lane" and the ego may resume.
        # Narrower than v2x_lateral_corridor (5 m, the threat threshold)
        # so the ego doesn't sit until the ped is fully off the road —
        # past the lane is enough.
        self.declare_parameter("v2x_lane_half_width", 2.0)
        # Monotonic-progress release threshold (signed body-frame meters,
        # `lat_now * brake_side`). If the ped has at ANY point been
        # observed past this threshold during the brake hold, release —
        # even if the most recent CAM has them back inside the lane.
        # This is the fallback for "RSU lost the ped right as they
        # exited the lane": their last cached position freezes inside
        # the corridor (e.g. lat≈-0.2 m) while the bridge keeps
        # republishing fresh-age CAMs. Without this, the AEB sits until
        # max_hold timeout. 1.5 m places the threshold halfway between
        # ego centerline and the opposite lane edge — the ped is then
        # committed to leaving the lane and won't reverse direction.
        self.declare_parameter("v2x_progress_release_m", 1.5)
        self.declare_parameter("v2x_max_age", 0.5)
        # Yield-until-clear release (mirrors perception AEB). After a
        # full stop triggered by V2X, hold the brake until the V2X CAM
        # has reported the ped clearly outside the corridor (or past the
        # ego) for `clear_persistence_s` consecutive seconds. Minimum
        # dwell `hold_after_stop_s` settles the stop; `max_hold_s` caps
        # a stuck-forever case where the CAM never updates.
        self.declare_parameter("hold_after_stop_s", 2.0)
        self.declare_parameter("clear_persistence_s", 1.0)
        # IMPORTANT: max_hold_s is measured in WALL-CLOCK seconds (the
        # control timer fires on ROS wall time, not sim time). The V2X
        # stack drops CARLA's real-time factor to ~30 %, so 1 sim s
        # takes ~3.3 wall s. The wait we need to cover:
        #   ~2 sim s hold_after_stop_s settle
        # + ≤10 sim s waiting for the scenario time-fallback to fire
        # + ~5.6 sim s ped walking across the 5 m corridor
        # ≈ 18 sim s ≈ 60 wall s in the worst case. 40 wall s ≈ 12 sim s
        # is enough for the smoke-test cell (ped starts at sim t=10,
        # brake at sim t≈8) but bump if a higher-speed cell shows
        # release on max_hold_s instead of "corridor clear".
        self.declare_parameter("max_hold_s", 40.0)
        # Post-release cooldown — after AEB releases following a stop,
        # ignore further V2X threats for this many seconds. This lets
        # the ego drive through the crosswalk cleanly after waiting for
        # the ped instead of ping-ponging between brake and creep when
        # the RSU's stale ped position keeps re-tripping the threat.
        # Matches the human-driver pattern: once you've yielded and the
        # path is clear enough to proceed, you don't re-brake mid-way.
        self.declare_parameter("post_release_cooldown_s", 10.0)

        ego_id = self.get_parameter("ego_actor_id").value
        self._target_speed = self.get_parameter("cruise_speed_kmh").value / 3.6
        self._ttc_brake = self.get_parameter("ttc_brake").value
        self._lane_hw = self.get_parameter("lane_half_width").value
        self._min_z = self.get_parameter("min_obstacle_z").value
        self._max_z = self.get_parameter("max_obstacle_z").value
        self._fusion_mode = self.get_parameter("fusion_mode").value
        self._v2x_lateral = self.get_parameter("v2x_lateral_corridor").value
        self._v2x_lane_half = float(self.get_parameter("v2x_lane_half_width").value)
        self._v2x_progress_release = float(self.get_parameter("v2x_progress_release_m").value)
        self._v2x_max_age = self.get_parameter("v2x_max_age").value
        self._hold_after_stop = float(self.get_parameter("hold_after_stop_s").value)
        self._clear_persistence = float(self.get_parameter("clear_persistence_s").value)
        self._max_hold = float(self.get_parameter("max_hold_s").value)
        self._post_release_cooldown = float(self.get_parameter("post_release_cooldown_s").value)

        if self._fusion_mode not in ("lidar_only", "v2x_only", "both"):
            raise ValueError(f"Invalid fusion_mode: {self._fusion_mode!r}")

        # Speed controller (PI with conditional-integration anti-windup —
        # I-term eliminates the steady-state cruise droop that pure-P leaves).
        self._kp = 0.3
        self._ki = 0.2
        self._max_throttle = 0.9
        self._integral = 0.0
        self._integral_max = 3.0

        # State (lidar side — copied from baseline)
        self._ego_speed = 0.0
        self._min_front_distance = float("inf")
        self._braking = False
        self._lidar_received = False
        self._stopped_time = 0.0
        self._history_size = 30
        self._dist_history = deque(maxlen=self._history_size)
        self._lidar_threat = False
        self._brake_source = None  # 'lidar' or 'v2x'
        # Sign of the ped's lateral position (in ego body frame) when the
        # V2X brake first fires. Locked on the first tick during braking
        # where |lat_now| > 1.5 m (i.e. the ped is clearly on one side).
        # Used by `_v2x_corridor_occupied` to release as soon as the ped
        # has crossed to the OPPOSITE side past the driving lane, rather
        # than waiting for the ped to fully exit the wide 5 m corridor.
        # Reset to None on every AEB release.
        self._ped_brake_side = None
        # Most-negative `lat * brake_side` (i.e. furthest progress toward
        # the opposite side) the ped has reached during this brake hold.
        # The release rule looks at this monotonic minimum rather than the
        # instantaneous CAM, because the RSU sometimes loses track of the
        # ped JUST as they exit the lane — its last cached position then
        # freezes inside the corridor (e.g. at lat=-0.2 m) while the
        # bridge keeps republishing it with fresh delivery age. Without
        # monotonic tracking the corridor would stay "occupied" forever
        # until max_hold fires.
        self._ped_max_progress = float("inf")
        # Yield-until-clear: consecutive seconds the V2X corridor has
        # been clear of the ped since coming to a stop.
        self._clear_time = 0.0
        # Post-release cooldown deadline (wall-clock seconds). Until then,
        # V2X threats are ignored — we've already yielded, the ped is
        # presumed to be clearing, and we want a clean drive-through.
        self._cooldown_until = 0.0

        # Brake-hold wall-clock timing. Set on the tick the brake first
        # fires (`_brake_trigger_wall`) and the tick ego first reaches
        # full stop (`_full_stop_wall`). Used to print a wait-time
        # breakdown on release: time-to-stop, hold time, total wait.
        self._brake_trigger_wall = None
        self._full_stop_wall = None
        self._brake_diag_counter = 0

        # State (V2X side)
        self._ego_x = 0.0
        self._ego_y = 0.0
        self._ego_yaw = 0.0
        self._ego_pose_received = False
        self._last_cam = None  # latest CAM dict
        self._v2x_threat = False
        self._v2x_n_received = 0
        self._diag_log_counter = 0  # log V2X eval state ~once per second

        # CARLA native sensor QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        lidar_topic   = f"/carla/actor{ego_id}/lidar/point_cloud"
        control_topic = f"/carla/actor{ego_id}/vehicle_control_cmd"

        self._lidar_sub = self.create_subscription(
            PointCloud2, lidar_topic, self._on_lidar, sensor_qos)
        self._speed_sub = self.create_subscription(
            Float64, "/ego/speed", self._on_speed, 10)
        # Subscribe to /v2x/ego_pose published by v2x_bridge_node — this works
        # regardless of CARLA version since the bridge reads ego pose via the
        # CARLA Python API, not via CARLA's native ROS2 (which is 0.10-only).
        self._ego_odom_sub = self.create_subscription(
            Odometry, "/v2x/ego_pose", self._on_ego_odom, 10)
        self._cam_sub = self.create_subscription(
            Odometry, "/v2x/cam_received", self._on_cam, 10)
        self._ctrl_pub = self.create_publisher(
            CarlaEgoVehicleControl, control_topic, 10)
        self._link_state_pub = self.create_publisher(
            String, "/metrics/v2x_link_state", 10)

        self._timer = self.create_timer(0.05, self._control_loop)

        self.get_logger().info(
            f"AEB-V2X started — fusion={self._fusion_mode}, "
            f"target={self._target_speed*3.6:.0f} km/h, TTC_brake={self._ttc_brake}s, "
            f"v2x_corridor={self._v2x_lateral}m, v2x_max_age={self._v2x_max_age}s"
        )

    # ----------------------------------------------------- subscribers

    def _on_speed(self, msg: Float64):
        self._ego_speed = msg.data

    def _on_ego_odom(self, msg: Odometry):
        if not self._ego_pose_received:
            self.get_logger().info(
                f"FIRST ego odom received at "
                f"({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f})")
        self._ego_x = msg.pose.pose.position.x
        self._ego_y = msg.pose.pose.position.y
        # yaw from quaternion (z-axis rotation)
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._ego_yaw = math.atan2(siny, cosy)
        self._ego_pose_received = True

    def _on_cam(self, msg: Odometry):
        # Bridge encodes msg_age_s in twist.angular.z
        self._v2x_n_received += 1
        self._last_cam = {
            "ped_x":   msg.pose.pose.position.x,
            "ped_y":   msg.pose.pose.position.y,
            "ped_vx":  msg.twist.twist.linear.x,
            "ped_vy":  msg.twist.twist.linear.y,
            "age_s":   msg.twist.twist.angular.z,
            "stamp":   self.get_clock().now(),
        }

    def _on_lidar(self, msg: PointCloud2):
        if not self._lidar_received:
            self._lidar_received = True
            self.get_logger().info(
                f"First lidar: {msg.width}x{msg.height} pts, "
                f"fields={[f.name for f in msg.fields]}")

        x_off = y_off = z_off = None
        for field in msg.fields:
            if field.name == "x":   x_off = field.offset
            elif field.name == "y": y_off = field.offset
            elif field.name == "z": z_off = field.offset
        if x_off is None or y_off is None or z_off is None:
            return

        point_step = msg.point_step
        data = bytes(msg.data)
        min_dist = float("inf")
        for i in range(0, len(data) - point_step + 1, point_step):
            x = struct.unpack_from("f", data, i + x_off)[0]
            y = struct.unpack_from("f", data, i + y_off)[0]
            z = struct.unpack_from("f", data, i + z_off)[0]
            if x <= 1.0 or abs(y) > self._lane_hw or z < self._min_z or z > self._max_z:
                continue
            dist = math.sqrt(x * x + y * y)
            if dist < min_dist:
                min_dist = dist

        self._min_front_distance = min_dist
        self._dist_history.append(min_dist)

    # ----------------------------------------------------- threat logic

    def _is_sudden_obstacle(self):
        """LiDAR sudden-obstacle detector — same logic as baseline."""
        if len(self._dist_history) < 2:
            return False
        current = self._dist_history[-1]
        if current == float("inf"):
            return False
        prev = self._dist_history[-2]
        if (prev == float("inf") or prev > 20.0) and current < 12.0:
            return True
        lookback = min(5, len(self._dist_history) - 1)
        past = self._dist_history[-1 - lookback]
        if (past == float("inf") or past > 20.0) and current < 12.0:
            return True
        if past != float("inf") and current < 15.0 and (past - current) > 8.0:
            return True
        return False

    def _compute_lidar_ttc(self):
        if len(self._dist_history) < 5:
            return float("inf")
        current = self._dist_history[-1]
        if current == float("inf"):
            return float("inf")
        past = self._dist_history[-5]
        if past == float("inf"):
            return float("inf")
        dt = 5 * (1.0 / 60.0)
        closing = (past - current) / dt
        if closing < 0.5:
            return float("inf")
        return current / closing

    def _v2x_threat_eval(self):
        """Decide if the latest CAM indicates a collision risk.

        Two-stage check:
        (1) Exit gate — if the ped is already off the ego centerline AND its
            lateral velocity points further outward (same sign as lat_now), the
            ped is leaving the path. No threat. This catches the "ped just
            finished crossing in front" case where a pure CPA snapshot at small
            TTC would still see the ped inside a generous corridor.
        (2) Closest-point-of-approach — ego reaches ped's longitudinal position
            at t_cpa = long_now / closing. Threat only if the lateral position
            extrapolated to t_cpa is still within the corridor.

        Returns (is_threat, long_now_m, ttc_s, age_s).
        """
        if self._last_cam is None or not self._ego_pose_received:
            return False, float("inf"), float("inf"), float("inf")
        cam = self._last_cam
        if cam["age_s"] > self._v2x_max_age:
            return False, float("inf"), float("inf"), cam["age_s"]
        # Post-release cooldown: we've already yielded once for this ped;
        # don't ping-pong on stale RSU CAMs while driving through.
        if time.time() < self._cooldown_until:
            return False, float("inf"), float("inf"), cam["age_s"]

        c, s = math.cos(self._ego_yaw), math.sin(self._ego_yaw)

        # Ped position in ego frame
        dx = cam["ped_x"] - self._ego_x
        dy = cam["ped_y"] - self._ego_y
        long_now =  c * dx + s * dy
        lat_now  = -s * dx + c * dy

        if long_now <= 0:
            return False, float("inf"), float("inf"), cam["age_s"]

        # Ped velocity in ego frame
        vx_ped =  c * cam["ped_vx"] + s * cam["ped_vy"]
        vy_ped = -s * cam["ped_vx"] + c * cam["ped_vy"]

        # (1) Exit gate: ped is off-centerline and moving further outward.
        # `lat_now * vy_ped > 0` means same sign → diverging from path.
        ped_exiting_lane = (
            lat_now * vy_ped > 0
            and abs(vy_ped) > 0.3
            and abs(lat_now) > 1.0
        )
        if ped_exiting_lane:
            return False, long_now, float("inf"), cam["age_s"]

        # Longitudinal closing rate (ego catching up to ped). If non-positive,
        # ego is not approaching → cannot collide via straight-line dynamics.
        closing = self._ego_speed - vx_ped
        if closing <= 0.1:
            return False, long_now, float("inf"), cam["age_s"]

        # (2) CPA: where will ped be when ego reaches its longitudinal position?
        ttc = long_now / closing
        lat_cpa = lat_now + vy_ped * ttc
        if abs(lat_cpa) > self._v2x_lateral:
            return False, long_now, ttc, cam["age_s"]

        is_threat = ttc < self._ttc_brake or long_now < 10.0
        return is_threat, long_now, ttc, cam["age_s"]

    def _v2x_corridor_occupied(self):
        """Is the V2X-reported ped currently in the ego's forward corridor?

        Used by the yield-until-clear release check — we hold the brake
        until the V2X-reported ped has moved out of the ego's lane.

        Release rules (any one fires "clear"):
          1. Ped behind ego front (long_now < -1 m)
          2. Ped past the wide V2X corridor edge (|lat_now| > v2x_lateral)
          3. Ped past the LANE on the OPPOSITE SIDE from where they were
             when brake fired. This is the natural "ped finished crossing"
             signal — the wide corridor (5 m) is the threat zone (catches
             curb-waiting peds), the lane half-width (~2 m, derived from
             v2x_lateral_lane_half) is the release zone. Without rule 3
             the ego waits another ~3 sim s for the ped to leave the curb
             on the far side, which is fine for safety but slow as a demo.

        Conservative fallback: missing or stale CAM → treat corridor as
        occupied (do not release without information).
        """
        if self._last_cam is None or not self._ego_pose_received:
            return True
        cam = self._last_cam
        if cam["age_s"] > self._v2x_max_age:
            return True
        c, s = math.cos(self._ego_yaw), math.sin(self._ego_yaw)
        dx = cam["ped_x"] - self._ego_x
        dy = cam["ped_y"] - self._ego_y
        long_now =  c * dx + s * dy
        lat_now  = -s * dx + c * dy
        # Ped already behind ego front (1 m back-of-bumper margin) — clear.
        if long_now < -1.0:
            return False
        # Ped laterally past the wide corridor edge — clear in the ego's path.
        if abs(lat_now) > self._v2x_lateral:
            return False
        # Lock the brake-time side on the first call where the ped is
        # clearly on one side of the lane (>1.5 m lateral). Reset on
        # release.
        if self._ped_brake_side is None and abs(lat_now) > 1.5:
            self._ped_brake_side = 1 if lat_now > 0 else -1
        if self._ped_brake_side is not None:
            # Track furthest progress toward the opposite side. If the ped
            # was EVER observed past the progress-release threshold, treat
            # as cleared — guards against the RSU losing the ped just as
            # they exit the lane and freezing the cached position inside
            # the corridor.
            progress = lat_now * self._ped_brake_side
            if progress < self._ped_max_progress:
                self._ped_max_progress = progress
            if self._ped_max_progress < -self._v2x_progress_release:
                return False
        return True

    # ----------------------------------------------------- control loop

    def _speed_control(self):
        """PI cruise controller with conditional-integration anti-windup.
        Integral only updates when the throttle isn't saturated against the
        error direction — prevents wind-up during the initial accel-from-zero
        phase where throttle is pinned at max for several seconds."""
        error = self._target_speed - self._ego_speed
        throttle = self._kp * error + self._ki * self._integral
        saturated_up   = throttle >= self._max_throttle and error > 0
        saturated_down = throttle <= 0.0           and error < 0
        if not saturated_up and not saturated_down:
            self._integral += error * 0.05
            if self._integral > self._integral_max:
                self._integral = self._integral_max
            elif self._integral < -self._integral_max:
                self._integral = -self._integral_max
            throttle = self._kp * error + self._ki * self._integral
        return max(0.0, min(throttle, self._max_throttle))

    def _control_loop(self):
        d_lidar = self._min_front_distance
        ttc_lidar = self._compute_lidar_ttc()
        v2x_threat, d_v2x, ttc_v2x, age_v2x = self._v2x_threat_eval()

        if self._last_cam is not None and self._last_cam["age_s"] <= self._v2x_max_age:
            self._link_state_pub.publish(String(data="RECEIVED"))
        else:
            self._link_state_pub.publish(String(data="NO_MESSAGE"))

        ctrl = CarlaEgoVehicleControl()
        ctrl.header.stamp = self.get_clock().now().to_msg()

        if self._braking:
            if self._ego_speed < 0.3:
                ctrl.throttle = 0.0
                ctrl.brake = 0.5
                # First-tick-at-stop: stamp the full-stop wall time so we
                # can report time-to-stop on release.
                if self._full_stop_wall is None:
                    self._full_stop_wall = time.time()
                    ttstop = self._full_stop_wall - (self._brake_trigger_wall or self._full_stop_wall)
                    self.get_logger().info(
                        f"[v2x hold] FULL STOP reached — time-to-stop {ttstop:.2f} s")
                self._stopped_time += 0.05
                # Yield-until-clear: hold the brake until the V2X CAM
                # reports the ped clearly outside the ego corridor (or
                # behind the ego) for `clear_persistence_s` consecutive
                # seconds, after a `hold_after_stop_s` settle. The
                # `max_hold_s` cap covers the case where the RSU loses
                # the ped track and stops reporting motion (the CAM ped
                # position freezes inside the corridor) — without the
                # cap the AEB would hold forever.
                occupied = self._v2x_corridor_occupied()
                if occupied:
                    self._clear_time = 0.0
                else:
                    self._clear_time += 0.05

                # Brake-hold timing diag: every ~1 s, log how long we've
                # been waiting, what the latest CAM says about ped pose
                # in body frame, and whether we'd release this tick. This
                # is the trace needed to debug "vehicle sometimes doesn't
                # resume" — shows exactly when clear_time resets and why.
                self._brake_diag_counter += 1
                if self._brake_diag_counter >= 20:  # 20 * 0.05 s = 1 s
                    self._brake_diag_counter = 0
                    waited = time.time() - (self._brake_trigger_wall or time.time())
                    if self._last_cam is not None and self._ego_pose_received:
                        cam = self._last_cam
                        c, s = math.cos(self._ego_yaw), math.sin(self._ego_yaw)
                        dx = cam["ped_x"] - self._ego_x
                        dy = cam["ped_y"] - self._ego_y
                        long_off =  c * dx + s * dy
                        lat_off  = -s * dx + c * dy
                        side = "?" if self._ped_brake_side is None else (
                            "+1" if self._ped_brake_side > 0 else "-1")
                        prog = (f"{self._ped_max_progress:+.2f}"
                                if self._ped_max_progress != float("inf") else "n/a")
                        self.get_logger().info(
                            f"[v2x hold] waited={waited:.1f}s stopped={self._stopped_time:.1f}s "
                            f"clear={self._clear_time:.2f}/{self._clear_persistence:.1f}s "
                            f"occupied={occupied} side={side} "
                            f"long={long_off:+.2f} lat={lat_off:+.2f} "
                            f"max_prog={prog}/{-self._v2x_progress_release:+.1f} "
                            f"age={cam['age_s']*1000:.0f}ms "
                            f"max_hold={self._max_hold:.0f}s")
                    else:
                        self.get_logger().info(
                            f"[v2x hold] waited={waited:.1f}s stopped={self._stopped_time:.1f}s "
                            f"NO_CAM (last_cam={self._last_cam is not None} "
                            f"ego_pose={self._ego_pose_received})")

                cleared = (self._stopped_time > self._hold_after_stop
                           and self._clear_time > self._clear_persistence)
                timed_out = self._stopped_time > self._max_hold
                if cleared or timed_out:
                    reason = "corridor clear" if cleared else "max-hold timeout"
                    now = time.time()
                    total_wait = now - (self._brake_trigger_wall or now)
                    hold = now - (self._full_stop_wall or now)
                    ttstop = (self._full_stop_wall - self._brake_trigger_wall
                              if self._brake_trigger_wall and self._full_stop_wall else 0.0)
                    self._cooldown_until = now + self._post_release_cooldown
                    self.get_logger().info(
                        f"AEB released — {reason} (was triggered by {self._brake_source}); "
                        f"total wait={total_wait:.2f}s "
                        f"(time-to-stop={ttstop:.2f}s + hold={hold:.2f}s); "
                        f"V2X cooldown {self._post_release_cooldown:.0f}s"
                    )
                    self._braking = False
                    self._stopped_time = 0.0
                    self._clear_time = 0.0
                    self._lidar_threat = False
                    self._v2x_threat = False
                    self._ped_brake_side = None  # next encounter relocks
                    self._ped_max_progress = float("inf")  # reset progress tracker
                    self._integral = 0.0  # reset PI integral on resume
                    self._brake_trigger_wall = None
                    self._full_stop_wall = None
                    self._brake_diag_counter = 0
                    self._brake_source = None
            else:
                ctrl.throttle = 0.0
                ctrl.brake = 1.0
                self._stopped_time = 0.0

        elif self._ego_speed > 3.0:
            # ---- diagnostic: every ~1 s, log the V2X eval state regardless of threat ----
            self._diag_log_counter += 1
            if self._diag_log_counter >= 20:  # 20 * 0.05 s = 1 s
                self._diag_log_counter = 0
                if self._last_cam is not None and self._ego_pose_received:
                    cam = self._last_cam
                    dx = cam["ped_x"] - self._ego_x
                    dy = cam["ped_y"] - self._ego_y
                    c, s = math.cos(self._ego_yaw), math.sin(self._ego_yaw)
                    long_off =  c * dx + s * dy
                    lat_off  = -s * dx + c * dy
                    rng = math.hypot(dx, dy)
                    self.get_logger().info(
                        f"[v2x diag] ego=({self._ego_x:.1f},{self._ego_y:.1f}) "
                        f"yaw={math.degrees(self._ego_yaw):.0f}° "
                        f"ped=({cam['ped_x']:.1f},{cam['ped_y']:.1f}) "
                        f"range={rng:.1f}m long={long_off:+.1f} lat={lat_off:+.1f} "
                        f"age={cam['age_s']*1000:.0f}ms "
                        f"ttc_v2x={ttc_v2x:.2f}s threat={v2x_threat}")
                else:
                    self.get_logger().info(
                        f"[v2x diag] last_cam={self._last_cam is not None} "
                        f"ego_pose_received={self._ego_pose_received} "
                        f"n_cams={self._v2x_n_received}")

            # V2X path
            if self._fusion_mode in ("v2x_only", "both") and v2x_threat:
                if not self._v2x_threat:
                    self._v2x_threat = True
                    self.get_logger().warn(
                        f"V2X THREAT — ped {d_v2x:.1f}m ahead, TTC={ttc_v2x:.2f}s, "
                        f"CAM age={age_v2x*1000:.0f}ms, ~{self._ego_speed*3.6:.0f} km/h")
                ctrl.throttle = 0.0
                ctrl.brake = 1.0
                ctrl.hand_brake = ttc_v2x < 0.5
                if not self._braking:
                    self._brake_trigger_wall = time.time()
                    self._full_stop_wall = None
                    self._brake_diag_counter = 0
                self._braking = True
                self._brake_source = "v2x"
                self.get_logger().error(
                    f"AEB BRAKE [v2x] — TTC={ttc_v2x:.2f}s, dist={d_v2x:.1f}m, "
                    f"CAM age={age_v2x*1000:.0f}ms, ~{self._ego_speed*3.6:.0f} km/h")
                self._ctrl_pub.publish(ctrl)
                return

            # LiDAR path
            if self._fusion_mode in ("lidar_only", "both"):
                if not self._lidar_threat and self._is_sudden_obstacle():
                    self._lidar_threat = True
                    self.get_logger().warn(
                        f"LIDAR THREAT — sudden obstacle at {d_lidar:.1f}m, "
                        f"TTC={ttc_lidar:.2f}s, ~{self._ego_speed*3.6:.0f} km/h")
                if self._lidar_threat and (ttc_lidar < self._ttc_brake or d_lidar < 10.0):
                    ctrl.throttle = 0.0
                    ctrl.brake = 1.0
                    ctrl.hand_brake = ttc_lidar < 0.5
                    if not self._braking:
                        self._brake_trigger_wall = time.time()
                        self._full_stop_wall = None
                        self._brake_diag_counter = 0
                    self._braking = True
                    self._brake_source = "lidar"
                    self.get_logger().error(
                        f"AEB BRAKE [lidar] — TTC={ttc_lidar:.2f}s, dist={d_lidar:.1f}m, "
                        f"~{self._ego_speed*3.6:.0f} km/h")
                    self._ctrl_pub.publish(ctrl)
                    return

            # No threat: cruise
            ctrl.throttle = self._speed_control()
            ctrl.brake = 0.0
        else:
            ctrl.throttle = self._speed_control()
            ctrl.brake = 0.0

        self._ctrl_pub.publish(ctrl)


def main(args=None):
    rclpy.init(args=args)
    node = AEBNodeV2X()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
