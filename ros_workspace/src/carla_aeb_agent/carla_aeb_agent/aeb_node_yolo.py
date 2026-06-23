#!/usr/bin/env python3
"""
AEB consuming tracked VRU detections from carla_camera_lidar_perception.

Pipeline:
  /perception/detections  →  this node  →  /carla/actor{id}/vehicle_control_cmd

For each frame:
  1. Read the latest PoseArray of tracked objects.
     (Decode the hijacked Pose.orientation: vx, vy, age, track_id —
      see carla_camera_lidar_perception node docstring for the wire format.)
  2. For each track:
       - check path conflict:  |y_lat| < lane_half_width
       - compute TTC:           x_fwd / |vx|  when vx < 0  (closing); else inf
       - require track age ≥ min_track_frames (anti-flicker)
  3. Take min TTC across in-path tracks.
  4. State machine:
       - cruising + (min_ttc < ttc_brake_threshold  OR
                     x_fwd < stop_dist(v) + min_brake_clearance) → BRAKE
         (TTC trigger catches the closing rate; clearance trigger ensures the
          ego always stops *with margin*, not at the bumper, by computing the
          expected stopping distance under `assumed_brake_decel`.)
       - braking  + ego_speed < 0.3 m/s            → yield-until-clear,
           with ASYMMETRIC clearance gating:
             * BRAKE gating uses fused tracks (camera ∧ LiDAR depth)
               — high confidence to act, avoids false-brakes.
             * RELEASE gating uses sensor diversity (camera ∨ LiDAR
               tracker) — fail-safe-toward-stopped, ISO 21448 SOTIF
               graceful degradation. Stay stopped if EITHER an aged
               in-path track exists OR YOLO sees any person bbox in
               the latest fresh camera frame.
           Brake releases when neither channel sees a ped for
           `clear_persistence_s` consecutive seconds (after a
           `hold_after_stop_s` settle). A `max_hold_s` cap covers the
           case where both channels go blind forever on a phantom.
  5. Speed PI controller during cruise (ported from baseline aeb_node.py).

Parallel to baseline `carla_aeb_agent.aeb_node`. The baseline runs raw LiDAR
corridor detection; this one runs against the perception node's tracked
output. Both consume /ego/speed and publish to the same vehicle_control_cmd
topic, so they can be A/B tested with the same scenario.

Parameters:
    ego_actor_id            (int)
    cruise_speed_kmh        (double, 40.0)
    ttc_brake               (double, 1.5)     TTC threshold to trigger AEB
    min_brake_clearance_m   (double, 1.5)     desired ego-to-ped clearance at stop
    assumed_brake_decel     (double, 25.0)    m/s² used to plan stop_dist
    brake_lateral_margin_m  (double, 0.5)     trigger corridor = lane_hw + this
    lane_half_width         (double, 1.5)     path-conflict gate (metres)
    min_brake_speed_mps     (double, 1.0)     don't brake below this speed
    min_track_frames        (int, 3)          anti-flicker; tracks below this are ignored
    hold_after_stop_s       (double, 2.0)     min dwell time after full stop
    clear_persistence_s     (double, 1.0)     corridor must be clear this long
    max_hold_s              (double, 30.0)    safety cap on yield duration
    yolo_present_topic      (str, "/perception/yolo_person_present")
    yolo_freshness_s        (double, 0.5)     max age of yolo bool before stale
    detection_topic         (str, "/perception/detections")
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Bool, Float64, String
from geometry_msgs.msg import PoseArray
from carla_msgs.msg import CarlaEgoVehicleControl

_NAN = float("nan")


class AEBYoloNode(Node):
    """AEB driven by tracked VRU detections (TTC + path-conflict)."""

    def __init__(self):
        super().__init__("aeb_node_yolo")

        # --- Parameters ---
        self.declare_parameter("ego_actor_id", 0)
        self.declare_parameter("cruise_speed_kmh", 40.0)
        self.declare_parameter("ttc_brake", 1.5)
        # Clearance-margin trigger: in addition to TTC, brake when the ego's
        # expected emergency-brake stopping distance would leave less than
        # `min_brake_clearance_m` between the ego front and the threat. This
        # ensures the AEB stops *with margin*, not just before contact. Use
        # `assumed_brake_decel` as the planning deceleration (m/s²).
        self.declare_parameter("min_brake_clearance_m", 1.5)
        self.declare_parameter("assumed_brake_decel", 25.0)
        # Lateral safety margin added ONLY to the brake-trigger corridor:
        # the AEB treats a track as "in path" if |y_lat| ≤ lane_hw +
        # brake_lateral_margin_m. The release/yield-until-clear check still
        # uses the narrower lane_hw, so the AEB brakes early but doesn't sit
        # stopped forever for peds that pass safely outside the lane.
        self.declare_parameter("brake_lateral_margin_m", 0.5)
        # Near-miss-resume policy: when the ego comes to a full stop, the
        # scenario publishes /scenario/aeb_directive based on ground-truth
        # ego/ped positions:
        #   "STAY"  → there's room for the ped to cross in front of the ego
        #             → AEB stays in held-stop, mutual yield runs, ped crosses.
        #   "CREEP" → no room (ego blocking crosswalk) → AEB drives forward
        #             for `near_miss_resume_s` to clear, ped crosses behind.
        # Ground truth is the right source here: perception's last_x_fwd is
        # unreliable at close range (LiDAR drops out, ground-plane fallback
        # projects to the wrong spot), and the room-check is a high-level
        # policy decision that in real life uses HD-map + V2X + driver attn.
        self.declare_parameter("near_miss_resume_s", 4.0)
        # Realism dwell: ego comes to full stop, holds for this long, then
        # makes the room-check decision. Mirrors the human "driver pauses
        # after panic brake before deciding what to do next."
        self.declare_parameter("near_miss_dwell_s", 0.8)
        self.declare_parameter("lane_half_width", 1.2)
        self.declare_parameter("min_brake_speed_mps", 1.0)
        self.declare_parameter("min_track_frames", 2)
        self.declare_parameter("hold_after_stop_s", 2.0)
        # Yield-until-clear policy: after coming to a stop for a pedestrian,
        # only resume when the ego corridor has been clear of any in-path
        # track for `clear_persistence_s` consecutive seconds. A `max_hold_s`
        # safety cap prevents the ego from getting stuck forever on a phantom.
        self.declare_parameter("clear_persistence_s", 1.0)
        self.declare_parameter("max_hold_s", 30.0)
        # Asymmetric clearance: release also requires the camera-only YOLO
        # person-present bool to be False (and ≤ `yolo_freshness_s` old).
        # When the topic is stale we fall back to the tracker-based check.
        self.declare_parameter("yolo_present_topic", "/perception/yolo_person_present")
        self.declare_parameter("yolo_freshness_s", 0.5)
        self.declare_parameter("detection_topic", "/perception/detections")
        # Scenario metadata for the dashboard "scenario info" panel
        self.declare_parameter("scenario_label", "")
        self.declare_parameter("trigger_distance_m", 0.0)
        # When True, suppress vehicle_control_cmd publishes. Used when running
        # alongside the V2X AEB so this node only feeds dashboard metrics.
        self.declare_parameter("metrics_only", False)

        ego_id = int(self.get_parameter("ego_actor_id").value)
        self._target_speed = float(self.get_parameter("cruise_speed_kmh").value) / 3.6
        self._ttc_brake = float(self.get_parameter("ttc_brake").value)
        self._min_brake_clearance = float(self.get_parameter("min_brake_clearance_m").value)
        self._assumed_brake_decel = float(self.get_parameter("assumed_brake_decel").value)
        self._brake_lat_margin = float(self.get_parameter("brake_lateral_margin_m").value)
        self._near_miss_resume_s = float(self.get_parameter("near_miss_resume_s").value)
        self._near_miss_dwell = float(self.get_parameter("near_miss_dwell_s").value)
        self._lane_hw = float(self.get_parameter("lane_half_width").value)
        self._min_brake_speed = float(self.get_parameter("min_brake_speed_mps").value)
        self._min_track_frames = int(self.get_parameter("min_track_frames").value)
        self._hold_after_stop = float(self.get_parameter("hold_after_stop_s").value)
        self._clear_persistence = float(self.get_parameter("clear_persistence_s").value)
        self._max_hold = float(self.get_parameter("max_hold_s").value)
        self._yolo_topic = str(self.get_parameter("yolo_present_topic").value)
        self._yolo_freshness = float(self.get_parameter("yolo_freshness_s").value)
        det_topic = str(self.get_parameter("detection_topic").value)

        # --- Speed PI controller (ported from baseline) ---
        self._kp = 0.3
        self._ki = 0.2
        self._max_throttle = 0.9
        self._integral = 0.0
        self._integral_max = 3.0

        # --- State ---
        self._ego_speed = 0.0          # m/s
        self._latest_tracks = []        # list of (x_fwd, y_lat, vx, vy, age, tid)
        self._latest_track_stamp = None
        self._braking = False
        self._stopped_time = 0.0
        self._clear_time = 0.0          # consecutive seconds with no in-path track
        self._threat_track_id = None    # id of the track that triggered brake
        self._yolo_person_present = True  # safe default: assume occupied until told otherwise
        self._yolo_present_time = 0.0   # wall-clock of last bool message (for freshness)
        # Near-miss-resume state: when ego stopped too close to ped, drive
        # forward to clear the crosswalk for `near_miss_resume_s` seconds.
        # Decision input is the scenario directive (ground-truth informed)
        # rather than perception's last_x_fwd (which is unreliable at close
        # range — LiDAR drops, GP fallback projects to the wrong point).
        self._scenario_directive = "STAY"  # default: don't creep
        self._near_miss_resume = False
        self._near_miss_resume_until = 0.0

        # --- Topics ---
        ctrl_topic = f"/carla/actor{ego_id}/vehicle_control_cmd"

        det_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(PoseArray, det_topic, self._on_detections, det_qos)
        self.create_subscription(Float64, "/ego/speed", self._on_speed, 10)
        self.create_subscription(Bool, self._yolo_topic, self._on_yolo_present, 10)
        self.create_subscription(String, "/scenario/aeb_directive", self._on_aeb_directive, 10)
        self._metrics_only = bool(self.get_parameter("metrics_only").value)
        self._ctrl_pub = self.create_publisher(
            CarlaEgoVehicleControl, ctrl_topic, 10
        )

        # Demo metrics (Foxglove dashboard)
        self._dist_pub = self.create_publisher(Float64, "/metrics/ped_distance", 10)
        self._ttc_pub = self.create_publisher(Float64, "/metrics/ttc", 10)
        self._ped_speed_pub = self.create_publisher(Float64, "/metrics/ped_speed", 10)
        self._state_pub = self.create_publisher(String, "/metrics/state", 10)
        self._percep_state_pub = self.create_publisher(String, "/metrics/perception_state", 10)
        # Pre-formatted strings (Foxglove RawMessages panels)
        self._dist_str_pub = self.create_publisher(String, "/metrics/ped_distance_str", 10)
        self._ttc_str_pub = self.create_publisher(String, "/metrics/ttc_str", 10)
        self._ped_speed_str_pub = self.create_publisher(String, "/metrics/ped_speed_str", 10)
        self._ego_speed_str_pub = self.create_publisher(String, "/metrics/ego_speed_str", 10)
        # Numeric km/h for the speed gauge
        self._ego_speed_kmh_pub = self.create_publisher(Float64, "/metrics/ego_speed_kmh", 10)
        # Throttle / brake bars
        self._throttle_pub = self.create_publisher(Float64, "/metrics/throttle", 10)
        self._brake_pub = self.create_publisher(Float64, "/metrics/brake", 10)
        # Target speed as a constant line on the speed plot
        self._target_speed_pub = self.create_publisher(Float64, "/metrics/target_speed_kmh", 10)
        # Distance trace for the speed plot — never NaN; falls back to max so
        # the line keeps drawing horizontally when no ped is tracked.
        self._dist_plot_pub = self.create_publisher(Float64, "/metrics/ped_distance_plot", 10)
        # Scenario info — published periodically (default VOLATILE) so the
        # Foxglove bridge always picks it up regardless of when it connects.
        self._scenario_info_pub = self.create_publisher(
            String, "/metrics/scenario_info", 10
        )
        scenario_label = str(self.get_parameter("scenario_label").value)
        trigger_dist = float(self.get_parameter("trigger_distance_m").value)
        info_lines = []
        if scenario_label:
            info_lines.append(f"Scenario: {scenario_label}")
        info_lines.append(f"Target speed: {self._target_speed * 3.6:.0f} km/h")
        if trigger_dist > 0:
            info_lines.append(f"Trigger distance: {trigger_dist:.2f} m")
        self._scenario_info_text = "\n".join(info_lines)
        # Republish every 1 s so Foxglove always shows it.
        self.create_timer(1.0, lambda: self._scenario_info_pub.publish(
            String(data=self._scenario_info_text)))

        # 20 Hz control loop (matches baseline)
        self._timer = self.create_timer(0.05, self._control_loop)

        self.get_logger().info(
            f"aeb_node_yolo started — target={self._target_speed * 3.6:.0f} km/h, "
            f"TTC_brake={self._ttc_brake}s, lane_hw={self._lane_hw}m, "
            f"min_track_frames={self._min_track_frames}, "
            f"detections='{det_topic}'"
        )

    # ----------------------------------------------------------------- callbacks

    def _on_yolo_present(self, msg: Bool):
        self._yolo_person_present = bool(msg.data)
        self._yolo_present_time = time.time()

    def _on_aeb_directive(self, msg: String):
        self._scenario_directive = msg.data

    def _on_speed(self, msg: Float64):
        self._ego_speed = float(msg.data)

    def _on_detections(self, msg: PoseArray):
        """Decode the perception node's hijacked PoseArray format."""
        tracks = []
        for p in msg.poses:
            x_fwd = p.position.x
            y_lat = p.position.y
            vx = p.orientation.x
            vy = p.orientation.y
            age = int(p.orientation.z)
            tid = int(p.orientation.w)
            tracks.append((x_fwd, y_lat, vx, vy, age, tid))
        self._latest_tracks = tracks
        self._latest_track_stamp = msg.header.stamp

    # ----------------------------------------------------------------- logic

    def _evaluate_threats(self):
        """Walk all current tracks, return (min_ttc, threat_id, threat_x, threat_y).

        A track is a threat if:
          - it has been seen at least min_track_frames times
          - it is in the ego's path (|y_lat| < lane_half_width)
          - it is closing  (vx < 0  →  x_fwd will decrease)
          - it will still be in the path at TTC seconds from now
        """
        best_ttc = float("inf")
        best_id = None
        best_x = 0.0
        best_y = 0.0
        for x_fwd, y_lat, vx, vy, age, tid in self._latest_tracks:
            if age < self._min_track_frames:
                continue
            if x_fwd <= 0:
                continue                 # behind us

            # Closing rate. Note: vx is ALREADY the relative velocity of the
            # ped in ego's frame, so it implicitly contains ego's motion.
            # Do NOT add ego_speed again — that double-counts.
            closing = -vx
            if closing <= 0.1:
                continue                 # not closing

            ttc = x_fwd / closing

            # Path conflict check: is the ped in (or imminently entering)
            # ego's path? Trigger corridor is wider than the lane by
            # `brake_lateral_margin_m` — the AEB brakes for any track
            # entering the lane edge plus a safety buffer, not only tracks
            # already in the strict lane. Release/yield-until-clear still
            # uses the strict lane_hw via `_any_track_in_path`.
            lane_hw_trigger = self._lane_hw + self._brake_lat_margin
            in_path_now = abs(y_lat) <= lane_hw_trigger
            # Predicted lateral at TTC:
            y_at_ttc = y_lat + vy * ttc
            in_path_at_ttc = abs(y_at_ttc) <= lane_hw_trigger

            if not (in_path_now or in_path_at_ttc):
                continue                 # ped passes outside the lane

            if ttc < best_ttc:
                best_ttc = ttc
                best_id = tid
                best_x = x_fwd
                best_y = y_lat
        return best_ttc, best_id, best_x, best_y

    def _any_track_in_path(self):
        """True if any sufficiently-aged track sits in the ego corridor ahead.
        Used by the yield-until-clear release check — we hold the brake until
        the same corridor that defines a threat is empty."""
        for x_fwd, y_lat, _vx, _vy, age, _tid in self._latest_tracks:
            if age < self._min_track_frames:
                continue
            if x_fwd <= 0:
                continue
            if abs(y_lat) <= self._lane_hw:
                return True
        return False

    def _corridor_occupied_for_release(self):
        """Asymmetric release gating: stay stopped if EITHER channel sees a ped.

        Brake gating (the BRAKE branch above) requires fused tracks — high
        confidence to act. Release gating below requires BOTH channels to
        agree clearance — fail-safe-toward-stopped:

          * Tracker reports an in-path aged track  → still occupied
          * Camera YOLO bbox present (fresh)       → still occupied
          * Camera bool stale (no recent frame)    → fall back to tracker only

        This catches the close-range LiDAR dropout case: the tracker loses
        the ped (no LiDAR depth), but YOLO still has a bbox, so the camera
        signal keeps us stopped until the ped truly walks out of frame.
        """
        if self._any_track_in_path():
            return True
        cam_age = time.time() - self._yolo_present_time
        cam_fresh = self._yolo_present_time > 0.0 and cam_age <= self._yolo_freshness
        if cam_fresh and self._yolo_person_present:
            return True
        return False

    def _publish_metrics(self, ttc):
        """Publish nearest-ped distance/speed, current TTC, and BRAKE/CRUISE
        state for the Foxglove demo dashboard. NaN when no ped is tracked."""
        nearest_d = float("inf")
        nearest_speed = _NAN
        any_track = False
        any_in_path = False
        for x_fwd, y_lat, vx, vy, age, _tid in self._latest_tracks:
            if age < self._min_track_frames or x_fwd <= 0:
                continue
            any_track = True
            if abs(y_lat) <= self._lane_hw:
                any_in_path = True
            d = math.hypot(x_fwd, y_lat)
            if d < nearest_d:
                nearest_d = d
                nearest_speed = math.hypot(vx, vy)

        self._dist_pub.publish(Float64(data=round(nearest_d, 1) if math.isfinite(nearest_d) else _NAN))
        # ped_distance_plot: always finite, but use an impossible negative value
        # (-5 m) when no ped is tracked. With the plot's minYValue=0, this is
        # clipped below the visible range so the line only appears once a real
        # detection arrives, instead of misleading viewers with a fake "30 m".
        self._dist_plot_pub.publish(Float64(data=round(nearest_d, 1) if math.isfinite(nearest_d) else -5.0))
        self._ped_speed_pub.publish(Float64(data=round(nearest_speed, 2) if math.isfinite(nearest_speed) else _NAN))
        self._ttc_pub.publish(Float64(data=round(ttc, 1) if math.isfinite(ttc) else _NAN))
        self._state_pub.publish(String(data="BRAKE" if self._braking else "CRUISE"))
        if any_in_path:
            percep_state = "IN_PATH"
        elif any_track:
            percep_state = "DETECTED"
        else:
            percep_state = "NO_DETECTION"
        self._percep_state_pub.publish(String(data=percep_state))

        dist_txt = f"{nearest_d:.1f} m" if math.isfinite(nearest_d) else "—"
        speed_txt = f"{nearest_speed:.2f} m/s" if math.isfinite(nearest_speed) else "—"
        ttc_txt = f"{ttc:.1f} s" if math.isfinite(ttc) else "—"
        self._dist_str_pub.publish(String(data=dist_txt))
        self._ped_speed_str_pub.publish(String(data=speed_txt))
        self._ttc_str_pub.publish(String(data=ttc_txt))
        self._ego_speed_str_pub.publish(String(data=f"{self._ego_speed * 3.6:.1f} km/h"))
        self._ego_speed_kmh_pub.publish(Float64(data=round(self._ego_speed * 3.6, 1)))

    def _speed_control(self):
        """PI cruise (ported verbatim from baseline)."""
        error = self._target_speed - self._ego_speed
        throttle = self._kp * error + self._ki * self._integral
        saturated_up   = throttle >= self._max_throttle and error > 0
        saturated_down = throttle <= 0.0 and error < 0
        if not saturated_up and not saturated_down:
            self._integral += error * 0.05
            self._integral = max(-self._integral_max,
                                  min(self._integral, self._integral_max))
            throttle = self._kp * error + self._ki * self._integral
        return max(0.0, min(throttle, self._max_throttle))

    def _control_loop(self):
        ctrl = CarlaEgoVehicleControl()
        ctrl.header.stamp = self.get_clock().now().to_msg()

        ttc, tid, x, y = self._evaluate_threats()

        # NEAR-MISS-RESUME — ego stopped too close, driving forward to clear.
        # Suppresses brake re-trigger so the ego can pass the ped.
        if self._near_miss_resume:
            now = time.time()
            done = now >= self._near_miss_resume_until
            if done:
                self._near_miss_resume = False
                self._braking = False
                self._stopped_time = 0.0
                self._clear_time = 0.0
                self._threat_track_id = None
                self._scenario_directive = "STAY"
                self.get_logger().info("AEB near-miss resume complete")
                # fall through to normal cruise / brake-trigger logic below
            else:
                ctrl.throttle = self._speed_control()
                ctrl.brake = 0.0
                if not self._metrics_only:
                    self._ctrl_pub.publish(ctrl)
                    self._throttle_pub.publish(Float64(data=float(ctrl.throttle)))
                    self._brake_pub.publish(Float64(data=float(ctrl.brake)))
                self._target_speed_pub.publish(Float64(data=self._target_speed * 3.6))
                # Override state to NEAR_MISS for the dashboard
                self._state_pub.publish(String(data="NEAR_MISS"))
                self._publish_metrics(ttc)
                # Re-publish state (publish_metrics overwrote it as BRAKE/CRUISE)
                self._state_pub.publish(String(data="NEAR_MISS"))
                return

        if self._braking:
            # Yield-until-clear: hold the brake until the ego corridor has
            # been empty of any in-path track for `clear_persistence_s`
            # consecutive seconds (after a minimum dwell of `hold_after_stop_s`
            # to settle the stop). `max_hold_s` caps a stuck-forever case.
            if self._ego_speed < 0.3:
                ctrl.throttle = 0.0
                ctrl.brake = 0.5
                # Hold a full stop for `near_miss_dwell_s` before deciding.
                # Visual realism: panic-stop, brief pause, then act. The
                # check fires in the single tick when stopped_time crosses
                # the dwell threshold (window = one control period 0.05s).
                in_decision_tick = (
                    self._stopped_time >= self._near_miss_dwell
                    and self._stopped_time < self._near_miss_dwell + 0.05
                )
                if (not self._near_miss_resume
                        and in_decision_tick
                        and self._scenario_directive == "CREEP"):
                    self._near_miss_resume = True
                    self._near_miss_resume_until = time.time() + self._near_miss_resume_s
                    self.get_logger().warn(
                        "AEB NEAR MISS — scenario directive=CREEP "
                        "(no room for ped); driving forward to clear"
                    )
                self._stopped_time += 0.05
                if self._corridor_occupied_for_release():
                    self._clear_time = 0.0
                else:
                    self._clear_time += 0.05
                cleared = (self._stopped_time > self._hold_after_stop
                           and self._clear_time > self._clear_persistence)
                timed_out = self._stopped_time > self._max_hold
                if cleared or timed_out:
                    self._braking = False
                    self._stopped_time = 0.0
                    self._clear_time = 0.0
                    self._integral = 0.0
                    self._threat_track_id = None
                    reason = "corridor clear" if cleared else "max-hold timeout"
                    self.get_logger().info(f"AEB released — {reason}")
            else:
                ctrl.throttle = 0.0
                ctrl.brake = 1.0
                self._stopped_time = 0.0
                self._clear_time = 0.0
        elif self._ego_speed > self._min_brake_speed:
            # Two triggers — fire if EITHER says brake:
            #   (a) TTC trigger: ttc < ttc_brake_threshold (existing).
            #   (b) Clearance trigger: ego cannot stop at desired clearance
            #       given current speed and assumed brake decel — proactive
            #       margin so the ego stops *with cushion*, not at the bumper.
            #       stop_dist = v² / (2·a_brake); brake when x_fwd is closer
            #       than stop_dist + min_clearance.
            have_threat = tid is not None
            ttc_trigger = have_threat and ttc < self._ttc_brake
            stop_dist = (self._ego_speed ** 2) / (2.0 * self._assumed_brake_decel) \
                        if self._assumed_brake_decel > 0 else 0.0
            clear_trigger = (have_threat and math.isfinite(x) and x > 0
                             and x < stop_dist + self._min_brake_clearance)
            if ttc_trigger or clear_trigger:
                self._braking = True
                self._threat_track_id = tid
                ctrl.throttle = 0.0
                ctrl.brake = 1.0
                ctrl.hand_brake = ttc < 0.5
                reason = "TTC" if ttc_trigger else "CLR"
                self.get_logger().warn(
                    f"AEB BRAKE [{reason}] — T#{tid} TTC={ttc:.2f}s "
                    f"dist={x:.1f}m lat={y:+.1f}m stop_dist={stop_dist:.1f}m "
                    f"@ {self._ego_speed * 3.6:.0f} km/h"
                )
            else:
                ctrl.throttle = self._speed_control()
                ctrl.brake = 0.0
        else:
            ctrl.throttle = self._speed_control()
            ctrl.brake = 0.0

        if not self._metrics_only:
            self._ctrl_pub.publish(ctrl)
            self._throttle_pub.publish(Float64(data=float(ctrl.throttle)))
            self._brake_pub.publish(Float64(data=float(ctrl.brake)))
        self._target_speed_pub.publish(Float64(data=self._target_speed * 3.6))
        self._publish_metrics(ttc)


def main(args=None):
    rclpy.init(args=args)
    node = AEBYoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
