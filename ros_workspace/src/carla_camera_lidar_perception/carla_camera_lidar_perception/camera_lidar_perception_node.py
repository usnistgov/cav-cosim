#!/usr/bin/env python3
"""
Camera + LiDAR fusion perception node for VRU detection, with tracking.

Pipeline per frame:
  1. YOLOv8 on the RGB image → 2D person bboxes.
  2. For each bbox, depth comes from LiDAR fusion (closest credible cluster of
     LiDAR points whose pixels fall inside the bbox); if no cluster qualifies,
     optionally fall back to monocular ground-plane projection.
  3. Greedy nearest-neighbour tracker associates raw detections to persistent
     track IDs across frames. Velocity is estimated per-track from successive
     positions using an exponential moving average.
  4. Confirmed tracks (≥ N consecutive frames seen) are published.

The same node is launched once per agent (ego, RSU) with different sensor
topic prefixes. No hard-coded "ego" or "rsu" anywhere.

----------------------------------------------------------------------------
OUTPUT MESSAGE — IMPORTANT: orientation field is HIJACKED for track data
----------------------------------------------------------------------------
The node publishes geometry_msgs/PoseArray. Each Pose carries:

  pose.position.x = x_fwd  (m) — forward distance in agent's local frame
  pose.position.y = y_lat  (m) — left-positive lateral
  pose.position.z = z_grnd (m) — ground-plane Z (0 = on road)

  pose.orientation.x = vx  (m/s)        — forward velocity component
  pose.orientation.y = vy  (m/s)        — lateral velocity component
  pose.orientation.z = age (frames)     — how many frames this track has been seen
  pose.orientation.w = track_id (float) — integer track ID (cast back to int)

The orientation field is NOT a quaternion in this stream. Downstream consumers
(aeb_node_yolo, cpm_broadcaster) read these four floats directly. This is a
deliberate trade-off — a custom .msg package was avoided to keep the build
simple. A future refactor can introduce a proper TrackedObjectArray message
without changing the producer/consumer logic, only the wire format.
----------------------------------------------------------------------------

Coordinate conventions:
  - LiDAR frame  (ROS REP-105):  X-fwd, Y-left, Z-up
  - Vehicle frame                 X-fwd, Y-left, Z-up (LiDAR is just translated)
  - Camera body frame             same axes as vehicle, rotated by pitch
  - Camera optical frame          X-right, Y-down, Z-fwd

Sensor extrinsics on ego (must match the OSC2 scenario declaration):
  - LiDAR mount in vehicle frame:    (0.0, 0.0, 2.5)
  - Camera mount in vehicle frame:   (0.3, 0.0, 2.0), pitched -5°
For RSU and other agents, override the *_mount_* parameters at launch.

Parameters:
    agent_actor_id          (int)
    camera_ros_name         (str, "rgb")
    lidar_ros_name          (str, "lidar")
    output_topic            (str, "/perception/detections")
    debug_image_topic       (str, "/perception/image_debug")  empty → off
    publish_rate_hz         (double, 10.0)

    yolo_model              (str, abs path under project models/)
    yolo_conf_threshold     (double, 0.4)
    yolo_imgsz              (int, 640)
    yolo_device             (str, "cuda:0")

    image_width             (int, 800)
    image_height            (int, 600)
    camera_fov_deg          (double, 90.0)
    camera_mount_x_m        (double, 0.3)       camera offset in vehicle frame
    camera_mount_y_m        (double, 0.0)
    camera_mount_z_m        (double, 2.0)
    camera_pitch_deg        (double, -5.0)

    lidar_mount_x_m         (double, 0.0)       LiDAR offset in vehicle frame
    lidar_mount_y_m         (double, 0.0)
    lidar_mount_z_m         (double, 2.5)

    use_lidar_fusion              (bool, True)
    use_ground_plane_fallback     (bool, True)
    lidar_min_z_vehicle_m         (double, 0.3)
    lidar_max_z_vehicle_m         (double, 2.2)
    lidar_bbox_shrink_px          (int, 4)
    lidar_cluster_min_points      (int, 3)
    lidar_cluster_tolerance_m     (double, 0.5)
    max_detection_range_m         (double, 80.0)

    Tracking:
    track_gate_m                  (double, 1.5)   max distance for det→track association
    track_max_age_s               (double, 0.3)   drop tracks not seen this long
    track_velocity_alpha          (double, 0.5)   EMA smoothing for velocity (0=no update, 1=instant)
    track_min_confirmed_frames    (int, 2)        only publish tracks seen at least this many frames
"""

import math
import struct
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Bool
from cv_bridge import CvBridge

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

try:
    from ultralytics import YOLO
    HAS_YOLO = True
except ImportError:
    HAS_YOLO = False


PERSON_CLASS_ID = 0  # COCO class index for 'person'

# Default weights path — stable location outside /tmp so the cache survives
# reboots and the node doesn't depend on its working directory.
_DEFAULT_YOLO_MODEL = "/home/hnh21/iotav/CARLA_0.10/models/yolov8m.pt"


# ============================================================================
# Tracker — greedy nearest-neighbour association + EMA velocity
# ============================================================================
class _Track:
    """A single tracked object."""
    __slots__ = ("id", "x", "y", "z", "vx", "vy", "last_t", "frames", "source")

    def __init__(self, tid, x, y, z, t, source):
        self.id = tid
        self.x = x
        self.y = y
        self.z = z
        self.vx = 0.0
        self.vy = 0.0
        self.last_t = t
        self.frames = 1
        self.source = source  # source of the most recent depth update ("L"/"G")


class Tracker:
    """Greedy nearest-neighbour tracker for VRU detections.

    For each new detection, assign it to the closest existing track within a
    gating distance `gate_m`. Unassigned detections start new tracks.
    Velocity is updated by exponential moving average on successive positions.
    Tracks not seen for `max_age_s` seconds are dropped.

    No motion model / Kalman filter — pedestrians at 10 Hz are simple enough
    that a constant-position prediction with EMA velocity smoothing is
    adequate; the AEB derives TTC from the published velocity.
    """

    def __init__(self, gate_m=1.5, max_age_s=0.3, vel_alpha=0.5):
        self._tracks = {}      # id -> _Track
        self._next_id = 1
        self._gate2 = gate_m * gate_m
        self._max_age_s = max_age_s
        self._alpha = vel_alpha

    def step(self, detections, t_now):
        """Update tracker with this frame's detections.

        Args:
            detections: list of (x_fwd, y_lat, z_grnd, source) tuples
            t_now: timestamp in seconds (monotonic)

        Returns:
            list of _Track objects (the current active set, post-update)
        """
        # 1. Associate each detection to the closest in-range existing track
        used_tracks = set()
        assignments = {}  # det_idx -> track_id
        for di, (dx, dy, _dz, _src) in enumerate(detections):
            best_tid = None
            best_d2 = self._gate2
            for tid, tr in self._tracks.items():
                if tid in used_tracks:
                    continue
                d2 = (tr.x - dx) ** 2 + (tr.y - dy) ** 2
                if d2 < best_d2:
                    best_d2 = d2
                    best_tid = tid
            if best_tid is not None:
                assignments[di] = best_tid
                used_tracks.add(best_tid)

        # 2. Update assigned tracks
        for di, tid in assignments.items():
            dx, dy, dz, src = detections[di]
            tr = self._tracks[tid]
            dt = max(t_now - tr.last_t, 1e-3)
            vx_new = (dx - tr.x) / dt
            vy_new = (dy - tr.y) / dt
            tr.vx = self._alpha * vx_new + (1 - self._alpha) * tr.vx
            tr.vy = self._alpha * vy_new + (1 - self._alpha) * tr.vy
            tr.x = dx
            tr.y = dy
            tr.z = dz
            tr.last_t = t_now
            tr.frames += 1
            tr.source = src

        # 3. Spawn new tracks for unassigned detections
        for di, det in enumerate(detections):
            if di in assignments:
                continue
            dx, dy, dz, src = det
            tid = self._next_id
            self._next_id += 1
            self._tracks[tid] = _Track(tid, dx, dy, dz, t_now, src)

        # 4. Drop expired tracks
        expired = [tid for tid, tr in self._tracks.items()
                   if t_now - tr.last_t > self._max_age_s]
        for tid in expired:
            del self._tracks[tid]

        return list(self._tracks.values())


class CameraLidarPerceptionNode(Node):
    def __init__(self):
        super().__init__("camera_lidar_perception")

        if not HAS_YOLO:
            raise RuntimeError(
                "ultralytics package not installed. "
                "Install with: pip install ultralytics"
            )

        # --- Parameters ---
        self.declare_parameter("agent_actor_id", 0)
        self.declare_parameter("camera_ros_name", "rgb")
        self.declare_parameter("lidar_ros_name", "lidar")
        self.declare_parameter("output_topic", "/perception/detections")
        self.declare_parameter("debug_image_topic", "/perception/image_debug")
        self.declare_parameter("publish_rate_hz", 10.0)

        self.declare_parameter("yolo_model", _DEFAULT_YOLO_MODEL)
        self.declare_parameter("yolo_conf_threshold", 0.4)
        self.declare_parameter("yolo_imgsz", 640)
        self.declare_parameter("yolo_device", "cuda:0")

        self.declare_parameter("image_width", 800)
        self.declare_parameter("image_height", 600)
        self.declare_parameter("camera_fov_deg", 90.0)
        self.declare_parameter("camera_mount_x_m", 0.3)
        self.declare_parameter("camera_mount_y_m", 0.0)
        self.declare_parameter("camera_mount_z_m", 2.0)
        self.declare_parameter("camera_pitch_deg", -5.0)

        self.declare_parameter("lidar_mount_x_m", 0.0)
        self.declare_parameter("lidar_mount_y_m", 0.0)
        self.declare_parameter("lidar_mount_z_m", 2.5)

        self.declare_parameter("use_lidar_fusion", True)
        self.declare_parameter("use_ground_plane_fallback", True)
        self.declare_parameter("lidar_min_z_vehicle_m", 0.3)
        self.declare_parameter("lidar_max_z_vehicle_m", 2.2)
        self.declare_parameter("lidar_bbox_shrink_px", 4)
        self.declare_parameter("lidar_cluster_min_points", 3)
        self.declare_parameter("lidar_cluster_tolerance_m", 0.5)
        self.declare_parameter("max_detection_range_m", 80.0)

        # Tracker
        self.declare_parameter("track_gate_m", 1.5)
        self.declare_parameter("track_max_age_s", 0.3)
        self.declare_parameter("track_velocity_alpha", 0.5)
        self.declare_parameter("track_min_confirmed_frames", 2)

        # --- Resolve parameters ---
        agent_id = int(self.get_parameter("agent_actor_id").value)
        cam_name = str(self.get_parameter("camera_ros_name").value)
        lid_name = str(self.get_parameter("lidar_ros_name").value)
        self._out_topic = str(self.get_parameter("output_topic").value)
        self._dbg_topic = str(self.get_parameter("debug_image_topic").value)
        self._rate_hz = float(self.get_parameter("publish_rate_hz").value)

        model_name = str(self.get_parameter("yolo_model").value)
        self._conf = float(self.get_parameter("yolo_conf_threshold").value)
        self._imgsz = int(self.get_parameter("yolo_imgsz").value)
        device = str(self.get_parameter("yolo_device").value)

        self._W = int(self.get_parameter("image_width").value)
        self._H = int(self.get_parameter("image_height").value)
        fov = float(self.get_parameter("camera_fov_deg").value)

        # Camera extrinsics in vehicle frame (ROS convention: X-fwd, Y-left, Z-up)
        self._cam_mount = np.array([
            float(self.get_parameter("camera_mount_x_m").value),
            float(self.get_parameter("camera_mount_y_m").value),
            float(self.get_parameter("camera_mount_z_m").value),
        ], dtype=np.float32)
        self._cam_pitch = math.radians(
            float(self.get_parameter("camera_pitch_deg").value)
        )

        # LiDAR extrinsics in vehicle frame
        self._lid_mount = np.array([
            float(self.get_parameter("lidar_mount_x_m").value),
            float(self.get_parameter("lidar_mount_y_m").value),
            float(self.get_parameter("lidar_mount_z_m").value),
        ], dtype=np.float32)

        self._use_lidar = bool(self.get_parameter("use_lidar_fusion").value)
        self._use_gp_fallback = bool(self.get_parameter("use_ground_plane_fallback").value)
        self._lidar_z_min = float(self.get_parameter("lidar_min_z_vehicle_m").value)
        self._lidar_z_max = float(self.get_parameter("lidar_max_z_vehicle_m").value)
        self._bbox_shrink = int(self.get_parameter("lidar_bbox_shrink_px").value)
        self._cluster_min_pts = int(self.get_parameter("lidar_cluster_min_points").value)
        self._cluster_tol_m = float(self.get_parameter("lidar_cluster_tolerance_m").value)
        self._max_range = float(self.get_parameter("max_detection_range_m").value)

        track_gate = float(self.get_parameter("track_gate_m").value)
        track_max_age = float(self.get_parameter("track_max_age_s").value)
        track_alpha = float(self.get_parameter("track_velocity_alpha").value)
        self._track_min_frames = int(self.get_parameter("track_min_confirmed_frames").value)
        self._tracker = Tracker(
            gate_m=track_gate,
            max_age_s=track_max_age,
            vel_alpha=track_alpha,
        )

        # --- Intrinsics ---
        self._fx = self._fy = (self._W / 2.0) / math.tan(math.radians(fov) / 2.0)
        self._cx = self._W / 2.0
        self._cy = self._H / 2.0

        # Pre-compute the rotation that takes vehicle-aligned vectors into
        # the camera *body* frame (which is then trivially re-axed to optical).
        # The camera body is rotated by `cam_pitch` around the vehicle Y axis;
        # to go vehicle→body we apply the inverse rotation (+|cam_pitch|).
        theta = -self._cam_pitch
        cp, sp = math.cos(theta), math.sin(theta)
        # Rotation around Y_vehicle (left-axis) by angle theta:
        #   x' =  x*cos - z*sin
        #   y' =  y
        #   z' =  x*sin + z*cos
        self._R_veh_to_camb = np.array([
            [cp,  0.0,  -sp],
            [0.0, 1.0,  0.0],
            [sp,  0.0,   cp],
        ], dtype=np.float32)

        self.get_logger().info(
            f"Camera: {self._W}x{self._H}, FOV={fov:.0f}°, fx={self._fx:.1f}, "
            f"mount={tuple(self._cam_mount.tolist())}, "
            f"pitch={math.degrees(self._cam_pitch):.1f}°"
        )
        self.get_logger().info(
            f"LiDAR: mount={tuple(self._lid_mount.tolist())}, "
            f"fusion={'ON' if self._use_lidar else 'OFF'}, "
            f"z-gate=[{self._lidar_z_min:.1f}, {self._lidar_z_max:.1f}], "
            f"cluster_min_pts={self._cluster_min_pts}, "
            f"cluster_tol={self._cluster_tol_m:.2f} m"
        )

        # --- Topic plumbing ---
        image_topic = f"/carla/actor{agent_id}/{cam_name}/image"
        lidar_topic = f"/carla/actor{agent_id}/{lid_name}/point_cloud"

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._bridge = CvBridge()
        self._image_sub = self.create_subscription(
            Image, image_topic, self._on_image, sensor_qos
        )
        self._lidar_sub = self.create_subscription(
            PointCloud2, lidar_topic, self._on_lidar, sensor_qos
        )
        self._det_pub = self.create_publisher(PoseArray, self._out_topic, 10)
        self._dbg_pub = (
            self.create_publisher(Image, self._dbg_topic, 1) if self._dbg_topic else None
        )
        # Camera-only "any person bbox visible above conf threshold" signal.
        # Published from raw YOLO output BEFORE LiDAR/GP fusion, so it stays True
        # even when LiDAR drops the ped at close range. Used by AEB as a fail-safe
        # clearance signal: brake gating uses fused tracks, release gating
        # also requires this bool to be False (asymmetric: fuse to act, diverge
        # to release).
        self._yolo_person_pub = self.create_publisher(
            Bool, "/perception/yolo_person_present", 10
        )

        # --- Load YOLO ---
        self.get_logger().info(f"Loading YOLO model {model_name!r} on {device}...")
        self._model = YOLO(model_name)
        try:
            self._model.to(device)
        except Exception as e:  # pragma: no cover
            self.get_logger().warn(
                f"Could not place model on {device}: {e}. Falling back to CPU."
            )
            self._model.to("cpu")
        self.get_logger().info("YOLO model loaded.")

        # --- State ---
        self._latest_image = None
        self._latest_stamp = None
        self._latest_lidar_pts = None       # (N, 3) numpy array, vehicle frame
        self._first_image_logged = False
        self._first_lidar_logged = False

        # Per-frame source counters, dumped every ~5 s
        self._cnt_lidar = 0
        self._cnt_groundplane = 0
        self._cnt_dropped_no_lidar_no_fallback = 0
        self._last_count_dump = 0.0

        period = 1.0 / max(self._rate_hz, 1e-3)
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f"camera_lidar_perception ready. image='{image_topic}', "
            f"lidar='{lidar_topic}', out='{self._out_topic}', "
            f"dbg='{self._dbg_topic or '(off)'}', rate={self._rate_hz:.1f} Hz, "
            f"conf={self._conf:.2f}"
        )

    # ------------------------------------------------------------------ I/O

    def _on_image(self, msg: Image):
        if not self._first_image_logged:
            self._first_image_logged = True
            self.get_logger().info(
                f"First image: {msg.width}x{msg.height}, encoding={msg.encoding}"
            )
        self._latest_image = msg
        self._latest_stamp = msg.header.stamp

    def _on_lidar(self, msg: PointCloud2):
        """Cache the latest LiDAR scan as an (N, 3) numpy array in vehicle frame."""
        pts = self._unpack_pointcloud_xyz(msg)
        if pts is None:
            return
        # LiDAR frame → vehicle frame: translate by lidar mount offset
        pts_veh = pts + self._lid_mount  # broadcasts over N
        self._latest_lidar_pts = pts_veh
        if not self._first_lidar_logged:
            self._first_lidar_logged = True
            self.get_logger().info(
                f"First LiDAR: {pts.shape[0]} points, "
                f"z-range=[{pts_veh[:,2].min():.2f}, {pts_veh[:,2].max():.2f}] m"
            )

    @staticmethod
    def _unpack_pointcloud_xyz(msg: PointCloud2):
        """Extract Nx3 float32 array of (x, y, z) from a PointCloud2 message.

        Uses a structured numpy dtype so the field offsets are respected
        regardless of how the encoder lays out the per-point record.
        """
        x_off = y_off = z_off = None
        for f in msg.fields:
            if f.name == "x": x_off = f.offset
            elif f.name == "y": y_off = f.offset
            elif f.name == "z": z_off = f.offset
        if x_off is None or y_off is None or z_off is None:
            return None
        step = msg.point_step
        if step <= 0:
            return None
        n = len(msg.data) // step
        if n == 0:
            return None
        dtype = np.dtype({
            "names": ["x", "y", "z"],
            "formats": ["<f4", "<f4", "<f4"],
            "offsets": [x_off, y_off, z_off],
            "itemsize": step,
        })
        pts = np.frombuffer(bytes(msg.data), dtype=dtype, count=n)
        return np.stack([pts["x"], pts["y"], pts["z"]], axis=1).astype(np.float32)

    # ------------------------------------------------------------------ main loop

    def _tick(self):
        msg = self._latest_image
        if msg is None:
            return

        stamp = msg.header.stamp
        if hasattr(self, "_last_processed_stamp") and stamp == self._last_processed_stamp:
            return
        self._last_processed_stamp = stamp

        try:
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return

        t0 = time.perf_counter()
        results = self._model.predict(
            source=img, imgsz=self._imgsz, conf=self._conf,
            classes=[PERSON_CLASS_ID], verbose=False,
        )
        t_inf = (time.perf_counter() - t0) * 1000.0

        # Camera-only "person bbox present" signal — published BEFORE LiDAR
        # fusion so it survives close-range LiDAR dropout. YOLO is already
        # called with conf=self._conf and classes=[PERSON_CLASS_ID], so any
        # box in the result is a person above threshold.
        n_person_boxes = 0
        if results:
            r0 = results[0]
            if r0.boxes is not None:
                n_person_boxes = int(len(r0.boxes))
        self._yolo_person_pub.publish(Bool(data=n_person_boxes > 0))

        # Project current LiDAR into the image once per frame (used for all bboxes)
        proj = None
        if self._use_lidar and self._latest_lidar_pts is not None:
            proj = self._project_lidar(self._latest_lidar_pts)

        # --- Pass 1: collect raw detections from this frame ---
        raw_detections = []     # list of (x_fwd, y_lat, z_grnd, source, bbox, conf)
        annotated = img.copy() if (HAS_CV2 and self._dbg_pub is not None) else None

        if results:
            r = results[0]
            boxes = r.boxes.xyxy.cpu().numpy() if r.boxes is not None else np.empty((0, 4))
            confs = r.boxes.conf.cpu().numpy() if r.boxes is not None else np.empty((0,))

            for (x1, y1, x2, y2), conf in zip(boxes, confs):
                # LiDAR fusion first
                source = None
                xyz = None
                if proj is not None:
                    xyz = self._lidar_fuse_bbox(proj, x1, y1, x2, y2)
                    if xyz is not None:
                        source = "L"
                if xyz is None and self._use_gp_fallback:
                    u = (x1 + x2) / 2.0
                    v_bot = y2
                    xyz = self._pixel_to_ground(u, v_bot)
                    if xyz is not None:
                        source = "G"
                if xyz is None:
                    self._cnt_dropped_no_lidar_no_fallback += 1
                    continue

                x_fwd, y_lat, z_grnd = xyz
                rng = math.sqrt(x_fwd * x_fwd + y_lat * y_lat)
                if rng > self._max_range or x_fwd <= 0.0:
                    continue

                if source == "L":
                    self._cnt_lidar += 1
                else:
                    self._cnt_groundplane += 1

                raw_detections.append((x_fwd, y_lat, z_grnd, source, (x1, y1, x2, y2), float(conf)))

        # --- Pass 2: feed raw detections to the tracker ---
        t_now_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        det_tuples = [(d[0], d[1], d[2], d[3]) for d in raw_detections]
        active_tracks = self._tracker.step(det_tuples, t_now_sec)

        # Build a lookup so the debug annotator can attach track info to bboxes
        # (greedy nearest match: same association the tracker just did).
        track_for_det = {}
        if raw_detections and active_tracks:
            for di, det in enumerate(raw_detections):
                dx, dy = det[0], det[1]
                best = min(
                    active_tracks,
                    key=lambda tr: (tr.x - dx) ** 2 + (tr.y - dy) ** 2,
                )
                if (best.x - dx) ** 2 + (best.y - dy) ** 2 < 1e-2:
                    track_for_det[di] = best

        # --- Pass 3: build & publish output (only confirmed tracks) ---
        det_msg = PoseArray()
        det_msg.header.stamp = stamp
        det_msg.header.frame_id = "agent"

        for tr in active_tracks:
            if tr.frames < self._track_min_frames:
                continue
            pose = Pose()
            pose.position.x = float(tr.x)
            pose.position.y = float(tr.y)
            pose.position.z = float(tr.z)
            # HIJACKED orientation field — see node docstring at top of file.
            pose.orientation.x = float(tr.vx)            # vx (m/s)
            pose.orientation.y = float(tr.vy)            # vy (m/s)
            pose.orientation.z = float(tr.frames)        # age (frames)
            pose.orientation.w = float(tr.id)            # track id (cast back to int downstream)
            det_msg.poses.append(pose)

        # --- Debug image overlay ---
        if annotated is not None:
            for di, det in enumerate(raw_detections):
                x_fwd, y_lat, z_grnd, source, (x1, y1, x2, y2), conf = det
                rng = math.sqrt(x_fwd * x_fwd + y_lat * y_lat)
                tr = track_for_det.get(di)
                if tr is not None and tr.frames >= self._track_min_frames:
                    colour = (0, 255, 0) if source == "L" else (0, 200, 255)
                    label = (f"T#{tr.id} {rng:.1f}m [{source}] "
                             f"v=({tr.vx:+.1f},{tr.vy:+.1f}) "
                             f"{tr.frames}fr")
                else:
                    colour = (180, 180, 180)  # tentative track
                    label = f"ped(new) {conf:.2f} {rng:.1f}m [{source}]"
                cv2.rectangle(annotated, (int(x1), int(y1)), (int(x2), int(y2)),
                              colour, 2)
                cv2.putText(annotated, label, (int(x1), max(0, int(y1) - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1, cv2.LINE_AA)
                # Velocity arrow inside the bbox (image-space approx)
                if tr is not None and tr.frames >= self._track_min_frames:
                    cx_box = int((x1 + x2) / 2)
                    cy_box = int((y1 + y2) / 2)
                    # Project vehicle-frame velocity to image-space arrow:
                    # +vx (forward) → up in image; +vy (left) → left in image
                    arrow_x = cx_box - int(tr.vy * 25)
                    arrow_y = cy_box - int(tr.vx * 25)
                    cv2.arrowedLine(annotated, (cx_box, cy_box),
                                    (arrow_x, arrow_y), (255, 255, 255), 2,
                                    tipLength=0.3)

        self._det_pub.publish(det_msg)

        # Periodic source-mix log (every ~5 s) so we can see at a glance
        # how often LiDAR fusion succeeded vs how often we fell back.
        now_sec = time.time()
        if now_sec - self._last_count_dump > 5.0:
            self._last_count_dump = now_sec
            total = self._cnt_lidar + self._cnt_groundplane + self._cnt_dropped_no_lidar_no_fallback
            if total > 0:
                self.get_logger().info(
                    f"detection source mix (last 5s window cumulative): "
                    f"LiDAR={self._cnt_lidar}, "
                    f"GroundPlane={self._cnt_groundplane}, "
                    f"Dropped(no-fallback)={self._cnt_dropped_no_lidar_no_fallback}"
                )

        if annotated is not None:
            # Optional: overlay projected LiDAR points (debug / calibration aid)
            if proj is not None and self._dbg_pub is not None:
                u_arr, v_arr, depth_arr, in_img = proj
                if in_img.any():
                    # Z-gate to suppress ground returns in the visualisation
                    z_veh_all = self._latest_lidar_pts[:, 2]
                    show = in_img & (z_veh_all >= self._lidar_z_min) & (z_veh_all <= self._lidar_z_max)
                    if show.any():
                        uu = u_arr[show].astype(np.int32)
                        vv = v_arr[show].astype(np.int32)
                        dd = depth_arr[show]
                        # Colourize by depth (close=red, mid=yellow, far=blue)
                        d_clip = np.clip(dd, 0.0, 30.0) / 30.0
                        b = (d_clip * 255).astype(np.uint8)
                        r_ = ((1.0 - d_clip) * 255).astype(np.uint8)
                        g_ = ((1.0 - 2 * np.abs(d_clip - 0.5)) * 255).clip(0).astype(np.uint8)
                        for i in range(len(uu)):
                            cv2.circle(
                                annotated, (int(uu[i]), int(vv[i])), 3,
                                (int(b[i]), int(g_[i]), int(r_[i])), -1,
                            )
            cv2.putText(
                annotated, f"YOLO {t_inf:.0f}ms  N={len(det_msg.poses)}",
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2, cv2.LINE_AA,
            )
            try:
                dbg = self._bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                dbg.header.stamp = stamp
                self._dbg_pub.publish(dbg)
            except Exception as e:  # pragma: no cover
                self.get_logger().warn(f"debug image publish failed: {e}")

    # ----------------------------------------------------------- LiDAR fusion

    def _project_lidar(self, pts_veh):
        """Project an (N, 3) array of vehicle-frame LiDAR points into the camera
        image. Returns (u, v, depth, in_image_mask) — all length-N numpy arrays.

        depth = z in optical frame (forward distance from camera). in_image_mask
        is True for points in front of the camera with pixel coordinates inside
        the image bounds.
        """
        # Vehicle frame → centred-at-camera vehicle-aligned frame
        pts_cv = pts_veh - self._cam_mount  # (N, 3)

        # Vehicle-aligned → camera body frame (rotation by -pitch around Y)
        pts_cb = pts_cv @ self._R_veh_to_camb.T  # (N, 3)
        xB, yB, zB = pts_cb[:, 0], pts_cb[:, 1], pts_cb[:, 2]

        # Camera body → optical frame:
        #   x_opt = -y_body  (right = -left)
        #   y_opt = -z_body  (down = -up)
        #   z_opt = +x_body  (fwd = fwd)
        x_opt = -yB
        y_opt = -zB
        z_opt = xB

        # Avoid divide-by-zero for points at/behind the camera
        in_front = z_opt > 0.1
        u = np.where(in_front, self._cx + (x_opt / np.where(z_opt > 0, z_opt, 1.0)) * self._fx, -1.0)
        v = np.where(in_front, self._cy + (y_opt / np.where(z_opt > 0, z_opt, 1.0)) * self._fy, -1.0)
        in_img = in_front & (u >= 0) & (u < self._W) & (v >= 0) & (v < self._H)

        return u, v, z_opt, in_img

    def _lidar_fuse_bbox(self, proj, x1, y1, x2, y2):
        """Given the projected-LiDAR cache and a YOLO bbox in pixel coords,
        return (x_fwd, y_lat, z_grnd) for the *closest credible cluster* of
        LiDAR points inside the bbox — or None if no cluster qualifies.

        A "credible cluster" is the closest seed point that has at least
        `lidar_cluster_min_points` neighbours within `lidar_cluster_tolerance_m`
        along the depth axis. This avoids:
          (a) single-point noise spikes pretending to be the ped
          (b) the percentile/median sliding into a dominant background object
              (e.g. a wall, when the bbox cone catches both the ped and a
              wall some metres behind it)

        Filtering:
          * z (vehicle frame) within [lidar_min_z_vehicle, lidar_max_z_vehicle]
          * pixel inside the bbox (shrunken by lidar_bbox_shrink_px)
        """
        u, v, depth, in_img = proj
        if not in_img.any():
            return None

        # Per-point z-gate in vehicle frame
        z_veh = self._latest_lidar_pts[:, 2]
        z_ok = (z_veh >= self._lidar_z_min) & (z_veh <= self._lidar_z_max)

        # Bbox membership (shrunk to avoid edge-bleed into adjacent objects)
        s = self._bbox_shrink
        in_box = (
            in_img
            & z_ok
            & (u >= x1 + s) & (u <= x2 - s)
            & (v >= y1 + s) & (v <= y2 - s)
        )
        if not in_box.any():
            return None

        # Walk in-bbox points by ascending depth. For each candidate seed,
        # count how many other in-bbox points are within ±tol m of it.
        # First seed that hits the threshold defines the cluster.
        in_box_idx = np.where(in_box)[0]
        d_in = depth[in_box_idx]
        order = np.argsort(d_in)
        d_sorted = d_in[order]
        idx_sorted = in_box_idx[order]

        tol = self._cluster_tol_m
        min_pts = self._cluster_min_pts

        chosen = None
        for i in range(len(d_sorted)):
            d_seed = d_sorted[i]
            near = (d_sorted >= d_seed - tol) & (d_sorted <= d_seed + tol)
            if int(near.sum()) >= min_pts:
                chosen = idx_sorted[near]
                break

        if chosen is None or chosen.size == 0:
            return None

        pts_keep = self._latest_lidar_pts[chosen]
        med = np.median(pts_keep, axis=0)

        # LiDAR points in vehicle frame ARE our output convention:
        # x_fwd = vehicle X, y_lat = vehicle Y (left-positive), z_grnd = vehicle Z
        return float(med[0]), float(med[1]), float(med[2])

    # ----------------------------------------------------------- ground-plane fallback

    def _pixel_to_ground(self, u, v):
        """Phase-A monocular fallback: project a pixel onto the ground plane
        using flat-ground + known camera mount height. Used when LiDAR has no
        return inside the bbox.
        """
        x_n = (u - self._cx) / self._fx
        y_n = (v - self._cy) / self._fy

        cp = math.cos(self._cam_pitch)
        sp = math.sin(self._cam_pitch)
        d_y = cp * y_n - sp * 1.0
        d_z = sp * y_n + cp * 1.0
        d_x = x_n

        if d_y <= 1e-6:
            return None  # ray above horizon

        h = self._cam_mount[2]  # camera height above vehicle origin
        t = h / d_y
        if t <= 0:
            return None

        x_cam_level = d_x * t
        z_cam_level = d_z * t

        x_fwd = z_cam_level
        y_lat = -x_cam_level
        z_grnd = 0.0  # ground level in vehicle frame (vehicle origin ≈ road)
        return x_fwd, y_lat, z_grnd


def main(args=None):
    rclpy.init(args=args)
    node = CameraLidarPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
