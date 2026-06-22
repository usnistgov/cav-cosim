#!/usr/bin/env python3
"""
V2X Bridge ROS2 Node — couples CARLA to the ns-3 V2P gateway.

Architecture:
    CARLA server  ◄── carla.Client (Python API) ── this node ◄──TCP:8100──► ns-3 gateway
                                                       │
                                                       ▼
                                            /v2x/cam_received  (nav_msgs/Odometry)

Each tick (10 Hz by default):
  1. Reads ego and ped transforms from CARLA via the Python API
  2. Sends "ego_pose ped_pose" to the ns-3 V2P gateway over TCP
  3. Receives the most-recent successfully-delivered CAM (ped pose+vel at TX)
  4. Publishes that CAM as a ROS2 Odometry message for the AEB to consume

The ns-3 gateway runs as a separate process and connects in as a TCP client.
This node is the TCP server (matches the NIST cosim convention).

Parameters:
    carla_host                (string, "localhost")
    carla_port                (int,    2000)
    carla_version             (string, "0.9.16")  must match the running server
    bridge_port               (int,    8100)     TCP server port for ns-3
    tick_rate_hz              (double, 10.0)     CAM rate
    ego_role_name             (string, "hero")
    ped_role_name             (string, "")       optional; if empty, first walker.* actor
    cam_topic                 (string, "/v2x/cam_received")
    use_synthetic_poses       (bool,   False)    skip CARLA, use synthetic trajectories
                                                 (useful to validate ns-3 cosim alone)
"""

import math
import os
import socket
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header

# NOTE: do NOT `import carla` at module load — the system-installed module is
# version 0.10.0, and the user often runs CARLA 0.9.16. We must call
# carla_setup.setup_carla(version) FIRST (it adjusts sys.path), then import.


class V2XBridgeNode(Node):

    def __init__(self):
        super().__init__("v2x_bridge")

        self.declare_parameter("carla_host", "localhost")
        self.declare_parameter("carla_port", 2000)
        self.declare_parameter("carla_version", "0.9.16")
        self.declare_parameter("bridge_port", 8100)
        self.declare_parameter("tick_rate_hz", 10.0)
        self.declare_parameter("ego_role_name", "hero")
        self.declare_parameter("ped_role_name", "")
        self.declare_parameter("cam_topic", "/v2x/cam_received")
        self.declare_parameter("use_synthetic_poses", False)
        self.declare_parameter("osc2_engine_dir",
                               os.path.expanduser("~/iotav/CARLA_0.10/osc2_engine"))
        # ---- RSU (V2X cooperative-perception) parameters -------------------
        # "ground_truth"  → read ped pose from CARLA Python API (legacy mode,
        #                   assumes perfect ped self-localization).
        # "rsu_perception"→ read ped pose from a second perception node
        #                   running on infrastructure-mounted sensors at the
        #                   stop sign (rsu_spawner_v2x.py). The RSU's body-
        #                   frame detection is transformed into world frame
        #                   using the RSU's static CARLA pose, then sent to
        #                   ns-3 just like ground truth. This models how a
        #                   real V2X deployment actually works: the source
        #                   has its own perception error stack, not a
        #                   perfect GNSS broadcast from the ped's phone.
        self.declare_parameter("ped_pose_source", "ground_truth")
        self.declare_parameter("rsu_role_name", "rsu")
        self.declare_parameter("rsu_perception_topic", "/rsu/perception/detections")
        # How long the bridge keeps publishing predicted CAMs after the RSU
        # stops sending fresh detections (`_rsu_ped_state`). 0.5 disables
        # extrapolation entirely — the bridge only publishes a CAM when
        # there is a fresh RSU detection. Set this higher (e.g. 6.0) to
        # enable constant-velocity extrapolation, useful if the RSU loses
        # the track BEFORE the ped has crossed the lane. In the current
        # scenario the RSU's last detection already places the ped past
        # the lane edge, so the AEB's cached CAM contains a "ped past
        # lane" position and the opposite-side release rule fires from
        # that frozen value — no prediction needed.
        self.declare_parameter("rsu_extrapolate_s", 0.5)
        # ---- RSU spawning (merged from rsu_spawner_v2x.py) -----------------
        # When spawn_rsu=True the bridge ALSO spawns the RSU prop + camera in
        # CARLA on startup and republishes the camera feed to ROS. Folding
        # this into the bridge means only ONE CARLA Python client per V2X
        # run instead of two — fewer sync points = noticeably faster sim.
        self.declare_parameter("spawn_rsu", False)
        # Default placement: NE corner of the crosswalk on the east curb,
        # at the ped path's y-level. Real RSUs at unsignalized crosswalks
        # are routinely placed at intersection corners rather than mid-
        # block stop-sign posts — this is one of the canonical V2P
        # geometries. With (x=-279.64, y=-12.0, yaw=0°):
        #   * Ped path (y=-12, x sweeping -275 → -272) sits at body-frame
        #     (+6.14, 0) to (+4.64, 0) — DEAD CENTER of frame across the
        #     entire crossing.
        #   * Bus (-276, -18) is at body-frame bearing -58° (lower-left
        #     edge of FOV 140°, ±70°) — minimal screen presence, no risk
        #     of YOLO confusion.
        #   * Critical: the bus's front face is at y=-14, the line of
        #     sight from RSU (y=-12) to ped (y=-12) NEVER enters the
        #     bus's y range → ZERO occlusion of the ped throughout the
        #     entire crossing path.
        self.declare_parameter("rsu_world_x", -279.64)
        self.declare_parameter("rsu_world_y", -12.0)
        self.declare_parameter("rsu_world_z", 0.5)
        self.declare_parameter("rsu_yaw_deg", 0.0)
        # Visible pole prop. Empty string → unparented camera (no visual,
        # and the body-frame transform is harder to get right).
        # constructioncone is ~0.7 m tall — well below the 5 m camera mount
        # so it doesn't occlude any of the camera's FOV. warningconstruction
        # was tried previously and DID occlude the lower view, killing all
        # detections.
        self.declare_parameter("rsu_pole_blueprint",
                               "static.prop.constructioncone")
        # Camera at stop-sign mounting height: cone is at z=0.5 (ground),
        # camera offset 2.5 m above → world z=3.0 m. That puts the optical
        # center right at the top edge of a real US stop sign (face center
        # ~2.1 m, top ~2.7 m). Real V2P RSUs at unsignalized crosswalks are
        # routinely mounted on the existing stop-sign pole itself; this
        # height is what the user explicitly asked for ("same height as
        # the stop sign"). Pitch -17° aims optical axis at ped torso
        # (z≈1.0) over the 6.9 m horizontal distance to mid-crossing.
        self.declare_parameter("rsu_camera_mount_z", 2.5)
        self.declare_parameter("rsu_camera_pitch_deg", -18.0)
        self.declare_parameter("rsu_camera_fov_deg", 140.0)
        self.declare_parameter("rsu_camera_width", 1280)
        self.declare_parameter("rsu_camera_height", 720)
        # sensor_tick caps how often CARLA fires the listener — 0.1s = 10 Hz,
        # which is plenty for a 5 Hz perception node. Default (0.0) would
        # fire every world tick (~60 Hz) and waste 80% of the data.
        self.declare_parameter("rsu_camera_sensor_tick", 0.1)
        # RSU LiDAR. Adding this lets the perception node use camera+LiDAR
        # fusion (instead of ground-plane projection only), which gives
        # accurate depth at range — the RSU keeps tracking the ped through
        # the full crossing instead of dropping the track at ~13 m once
        # YOLO confidence falls. Specs mirror the ego LiDAR (32-ch AV-grade)
        # but at shorter range — the RSU only needs to see ~30 m of the
        # crosswalk approach.
        self.declare_parameter("rsu_lidar_mount_z", 2.5)
        self.declare_parameter("rsu_lidar_channels", 32)
        self.declare_parameter("rsu_lidar_points_per_second", 600000)
        self.declare_parameter("rsu_lidar_rotation_frequency", 10.0)
        self.declare_parameter("rsu_lidar_range_m", 50.0)
        self.declare_parameter("rsu_lidar_upper_fov_deg", 10.0)
        self.declare_parameter("rsu_lidar_lower_fov_deg", -30.0)
        self.declare_parameter("rsu_lidar_sensor_tick", 0.1)
        # Where to write the actor_id so run_sweep.py can pick it up
        # and pass it to the second perception node.
        self.declare_parameter("rsu_actor_id_file", "/tmp/carla_rsu_actor_id")

        self._carla_host = self.get_parameter("carla_host").value
        self._carla_port = self.get_parameter("carla_port").value
        self._carla_version = self.get_parameter("carla_version").value
        self._bridge_port = self.get_parameter("bridge_port").value
        self._tick_rate = self.get_parameter("tick_rate_hz").value
        self._ego_role = self.get_parameter("ego_role_name").value
        self._ped_role = self.get_parameter("ped_role_name").value
        self._cam_topic = self.get_parameter("cam_topic").value
        self._synthetic = self.get_parameter("use_synthetic_poses").value
        self._osc2_dir = self.get_parameter("osc2_engine_dir").value
        self._ped_source = str(self.get_parameter("ped_pose_source").value)
        self._rsu_extrapolate_s = float(self.get_parameter("rsu_extrapolate_s").value)
        self._rsu_role = str(self.get_parameter("rsu_role_name").value)
        self._rsu_perc_topic = str(self.get_parameter("rsu_perception_topic").value)
        self._spawn_rsu = bool(self.get_parameter("spawn_rsu").value)
        self._rsu_world_x = float(self.get_parameter("rsu_world_x").value)
        self._rsu_world_y = float(self.get_parameter("rsu_world_y").value)
        self._rsu_world_z = float(self.get_parameter("rsu_world_z").value)
        self._rsu_yaw = float(self.get_parameter("rsu_yaw_deg").value)
        self._rsu_pole_bp = str(self.get_parameter("rsu_pole_blueprint").value)
        self._rsu_cam_mount_z = float(self.get_parameter("rsu_camera_mount_z").value)
        self._rsu_cam_pitch = float(self.get_parameter("rsu_camera_pitch_deg").value)
        self._rsu_cam_fov = float(self.get_parameter("rsu_camera_fov_deg").value)
        self._rsu_cam_w = int(self.get_parameter("rsu_camera_width").value)
        self._rsu_cam_h = int(self.get_parameter("rsu_camera_height").value)
        self._rsu_cam_tick = float(self.get_parameter("rsu_camera_sensor_tick").value)
        self._rsu_lid_mount_z = float(self.get_parameter("rsu_lidar_mount_z").value)
        self._rsu_lid_channels = int(self.get_parameter("rsu_lidar_channels").value)
        self._rsu_lid_pps = int(self.get_parameter("rsu_lidar_points_per_second").value)
        self._rsu_lid_rot = float(self.get_parameter("rsu_lidar_rotation_frequency").value)
        self._rsu_lid_range = float(self.get_parameter("rsu_lidar_range_m").value)
        self._rsu_lid_upper = float(self.get_parameter("rsu_lidar_upper_fov_deg").value)
        self._rsu_lid_lower = float(self.get_parameter("rsu_lidar_lower_fov_deg").value)
        self._rsu_lid_tick = float(self.get_parameter("rsu_lidar_sensor_tick").value)
        self._rsu_lid_pub = None  # set in _spawn_rsu_assets
        self._rsu_actor_id_file = str(self.get_parameter("rsu_actor_id_file").value)
        self._rsu_spawned_actors = []  # cleaned up in destroy_node

        if self._ped_source not in ("ground_truth", "rsu_perception"):
            raise ValueError(
                f"ped_pose_source must be 'ground_truth' or 'rsu_perception', "
                f"got {self._ped_source!r}")

        self._pub = self.create_publisher(Odometry, self._cam_topic, 10)
        # Also publish ego pose so the V2X-aware AEB doesn't depend on CARLA's
        # native ROS2 odometry topic (which varies by CARLA version).
        self._ego_pose_pub = self.create_publisher(Odometry, "/v2x/ego_pose", 10)

        # ---- RSU subscription (only when ped_pose_source == "rsu_perception")
        # Holds the most recent RSU body-frame detection plus its arrival
        # time so we can finite-difference for velocity in _tick.
        self._rsu_last_world = None      # (x, y, z) world frame
        self._rsu_prev_world = None      # for velocity diff
        self._rsu_last_t = 0.0
        self._rsu_prev_t = 0.0
        # EMA-smoothed ped world velocity from RSU detection sequence.
        # Single-step finite-differences are noisy enough at typical
        # RSU ranges (~50 cm lateral error at 10 m) that the velocity
        # used for extrapolation can be 3-4× off the true value, which
        # makes a 6 s extrapolation cover only ~2 m instead of ~8 m.
        # alpha=0.3 with a 3 m/s plausibility gate keeps the velocity
        # stable across noisy frames.
        self._rsu_smoothed_vel = (0.0, 0.0, 0.0)
        self._rsu_actor = None           # CARLA actor handle (for static pose)
        if self._ped_source == "rsu_perception":
            self.create_subscription(
                PoseArray, self._rsu_perc_topic, self._on_rsu_detections, 10)
            self.get_logger().info(
                f"RSU mode: subscribing to {self._rsu_perc_topic!r}, "
                f"will use closest detection's world-frame pose as ped state")

        # ---- TCP server: open the listening socket but don't accept yet.
        # ns-3 can already buffer a SYN against this socket while we
        # finish CARLA setup and RSU spawning below.
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind(("0.0.0.0", self._bridge_port))
        self._srv.listen(1)
        self.get_logger().info(
            f"Listening on :{self._bridge_port} for ns-3 to connect...")
        self._buf = ""

        # ---- CARLA client (lazy import — must call setup_carla() first) ----
        self._ego = None
        self._ped = None
        self._carla = None  # the imported module, kept as an attribute for later use
        self._rsu_cam_pub = None  # set in _spawn_rsu_assets if spawn_rsu=True
        if not self._synthetic:
            self._setup_and_import_carla()
            self._client = self._carla.Client(self._carla_host, self._carla_port)
            self._client.set_timeout(10.0)
            self._world = self._client.get_world()
            self.get_logger().info(
                f"Connected to CARLA {self._carla_version} at "
                f"{self._carla_host}:{self._carla_port}")
            # Spawn the RSU pole + camera now, BEFORE ns-3 connects — that
            # way the RSU actor_id file exists by the time run_sweep.py
            # starts the second perception node.
            if self._spawn_rsu:
                self._spawn_rsu_assets()

        # ---- Now wait for ns-3 ----
        self._conn, _ = self._srv.accept()
        self._conn.settimeout(5.0)
        self.get_logger().info("ns-3 connected.")

        # ---- Tick timer ----
        self._t0 = time.time()
        self.create_timer(1.0 / self._tick_rate, self._tick)

    # -------------------------------------------------------------------- helpers

    def _setup_and_import_carla(self):
        """Configure sys.path via the OSC2 engine's carla_setup, then import carla.

        This is required because the system has both CARLA 0.10.0 (pip install,
        ~/.local) and 0.9.16 (egg under CARLA_0.9.16/). Without setup_carla,
        `import carla` picks up 0.10.0, which then crashes against a 0.9.16 server.
        """
        if self._osc2_dir not in sys.path:
            sys.path.insert(0, self._osc2_dir)
        try:
            from carla_setup import setup_carla
        except ImportError as e:
            raise RuntimeError(
                f"Could not import carla_setup from {self._osc2_dir}. "
                f"Set osc2_engine_dir parameter to the correct path. ({e})")
        setup_carla(self._carla_version)
        import carla as carla_mod
        self._carla = carla_mod
        self.get_logger().info(
            f"CARLA Python API ready (version {self._carla_version})")

    def _spawn_rsu_assets(self):
        """Spawn the RSU pole (static prop) + camera and start publishing
        the camera feed to ROS.

        Replaces the standalone rsu_spawner_v2x.py script — folded in here
        so we don't have two separate processes each connecting to CARLA
        as Python clients (each CARLA client adds tick-sync overhead).

        Visual: a static prop (default: warningconstruction sign) at ground
        level to make the RSU location visible in CARLA / Foxglove. The
        camera itself is mounted with attach_to=<pole> so its world pose
        moves with the prop if anyone ever drags it in the editor.

        Performance: the camera sets sensor_tick=0.1 so CARLA fires the
        listener at 10 Hz instead of every render frame (~60 Hz). The
        downstream perception node only runs at 5 Hz, so anything faster
        than 10 Hz is wasted bytes.
        """
        bp_lib = self._world.get_blueprint_library()

        # 1) Optional pole (visual only). Default config skips this so the
        # ONLY visual is the in-map stop sign that Town05_Opt already has
        # at the RSU location — avoids placing a collidable footprint near
        # the ego's lane edge.
        pole = None
        if self._rsu_pole_bp:
            for bp_name in [self._rsu_pole_bp, "static.prop.constructioncone"]:
                try:
                    pole_bp = bp_lib.find(bp_name)
                except (IndexError, RuntimeError):
                    continue
                if pole_bp.has_attribute("role_name"):
                    pole_bp.set_attribute("role_name", self._rsu_role)
                pole_tf = self._carla.Transform(
                    self._carla.Location(x=self._rsu_world_x,
                                         y=self._rsu_world_y,
                                         z=self._rsu_world_z),
                    self._carla.Rotation(yaw=self._rsu_yaw))
                for _ in range(5):
                    pole = self._world.try_spawn_actor(pole_bp, pole_tf)
                    if pole is not None:
                        break
                    time.sleep(0.2)
                if pole is not None:
                    self.get_logger().info(
                        f"RSU pole spawned: {bp_name} actor_id={pole.id}")
                    break

        # 2) RGB camera. If we have a pole, attach to it (sensor inherits
        # pole's transform). If not, spawn the camera unparented at the
        # RSU world location with the mount-z added in — same final pose,
        # no collidable footprint.
        cam_bp = bp_lib.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", str(self._rsu_cam_w))
        cam_bp.set_attribute("image_size_y", str(self._rsu_cam_h))
        cam_bp.set_attribute("fov", str(self._rsu_cam_fov))
        if cam_bp.has_attribute("sensor_tick"):
            cam_bp.set_attribute("sensor_tick", str(self._rsu_cam_tick))
        if cam_bp.has_attribute("ros_name"):
            cam_bp.set_attribute("ros_name", "rgb")
        if cam_bp.has_attribute("role_name"):
            cam_bp.set_attribute("role_name", self._rsu_role)
        if pole is not None:
            # Sensor pose is relative to the pole.
            cam_tf = self._carla.Transform(
                self._carla.Location(x=0.0, y=0.0, z=self._rsu_cam_mount_z),
                self._carla.Rotation(pitch=self._rsu_cam_pitch, yaw=0.0))
            cam = self._world.spawn_actor(cam_bp, cam_tf, attach_to=pole)
            self._rsu_spawned_actors.append(pole)
        else:
            # Unparented: absolute world pose.
            cam_tf = self._carla.Transform(
                self._carla.Location(
                    x=self._rsu_world_x,
                    y=self._rsu_world_y,
                    z=self._rsu_world_z + self._rsu_cam_mount_z),
                self._carla.Rotation(
                    pitch=self._rsu_cam_pitch,
                    yaw=self._rsu_yaw))
            cam = self._world.spawn_actor(cam_bp, cam_tf, attach_to=None)
        self._rsu_spawned_actors.append(cam)

        # The "RSU actor" used for body→world detection transforms is
        # whichever actor's world pose matches the camera mount yaw — pole
        # if we have one, otherwise the camera itself.
        self._rsu_actor = pole if pole is not None else cam

        # Reference actor_id: the agent body frame the perception will use
        # to project detections. With a pole, the pole has horizontal yaw +
        # zero pitch, so it's the correct reference. Without a pole, we
        # fall back to the camera (which is tilted — caller beware).
        # The perception node uses this id to subscribe to
        # /carla/actor{id}/rgb/image, so we MUST publish under the same id.
        ref_id = pole.id if pole is not None else cam.id
        try:
            with open(self._rsu_actor_id_file, "w") as f:
                f.write(str(ref_id))
        except OSError as e:
            self.get_logger().warn(
                f"Could not write {self._rsu_actor_id_file}: {e}")

        self.get_logger().info(
            f"RSU camera spawned: actor_id={cam.id} ref_id={ref_id} "
            f"{self._rsu_cam_w}x{self._rsu_cam_h} FOV={self._rsu_cam_fov:.0f}° "
            f"pitch={self._rsu_cam_pitch:.0f}° tick={self._rsu_cam_tick:.2f}s "
            f"pole={'yes' if pole else 'no (in-map stop sign is the visual)'}")

        # 3) ROS publisher for the camera frames.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._rsu_cam_pub = self.create_publisher(
            Image, f"/carla/actor{ref_id}/rgb/image", qos)
        self.get_logger().info(
            f"Publishing RSU camera to /carla/actor{ref_id}/rgb/image")

        # 4) Wire the CARLA sensor → ROS publisher
        cam.listen(self._on_rsu_camera_image)

        # 5) RSU LiDAR. Same attach-to-pole or unparented logic as the
        # camera, mounted at the same height (z=2.5 m on the pole).
        # Camera-only RSU perception was forced to use ground-plane
        # projection for depth (no LiDAR data to fuse), which has ~50 cm
        # lateral noise at 10 m growing with distance — and YOLO confidence
        # drops at the same range. Adding LiDAR fixes both: accurate depth
        # at range, and gives the perception node a second detection
        # source so the tracker holds the ped through the full crossing.
        lid_bp = bp_lib.find("sensor.lidar.ray_cast")
        lid_bp.set_attribute("channels", str(self._rsu_lid_channels))
        lid_bp.set_attribute("points_per_second", str(self._rsu_lid_pps))
        lid_bp.set_attribute("rotation_frequency", str(self._rsu_lid_rot))
        lid_bp.set_attribute("range", str(self._rsu_lid_range))
        lid_bp.set_attribute("upper_fov", str(self._rsu_lid_upper))
        lid_bp.set_attribute("lower_fov", str(self._rsu_lid_lower))
        if lid_bp.has_attribute("sensor_tick"):
            lid_bp.set_attribute("sensor_tick", str(self._rsu_lid_tick))
        if lid_bp.has_attribute("ros_name"):
            lid_bp.set_attribute("ros_name", "lidar")
        if lid_bp.has_attribute("role_name"):
            lid_bp.set_attribute("role_name", self._rsu_role)
        if pole is not None:
            lid_tf = self._carla.Transform(
                self._carla.Location(x=0.0, y=0.0, z=self._rsu_lid_mount_z),
                self._carla.Rotation(pitch=0.0, yaw=0.0))
            lid = self._world.spawn_actor(lid_bp, lid_tf, attach_to=pole)
        else:
            lid_tf = self._carla.Transform(
                self._carla.Location(
                    x=self._rsu_world_x,
                    y=self._rsu_world_y,
                    z=self._rsu_world_z + self._rsu_lid_mount_z),
                self._carla.Rotation(pitch=0.0, yaw=self._rsu_yaw))
            lid = self._world.spawn_actor(lid_bp, lid_tf, attach_to=None)
        self._rsu_spawned_actors.append(lid)
        self._rsu_lid_pub = self.create_publisher(
            PointCloud2, f"/carla/actor{ref_id}/lidar/point_cloud", qos)
        self.get_logger().info(
            f"RSU LiDAR spawned: actor_id={lid.id} "
            f"channels={self._rsu_lid_channels} range={self._rsu_lid_range:.0f}m "
            f"→ /carla/actor{ref_id}/lidar/point_cloud")
        lid.listen(self._on_rsu_lidar_data)

    def _on_rsu_lidar_data(self, carla_lidar):
        """CARLA LidarMeasurement → sensor_msgs/PointCloud2.

        Each CARLA point is 4 float32 values (x, y, z, intensity) packed
        contiguously. PointCloud2 with the same field layout is a direct
        bytes copy — no per-point Python loop. The perception node treats
        this as the sensor-frame point cloud and applies its own body→world
        transform via the RSU actor's pose."""
        if self._rsu_lid_pub is None:
            return
        try:
            msg = PointCloud2()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "rsu_lidar"
            n_points = len(carla_lidar) if hasattr(carla_lidar, "__len__") else 0
            msg.height = 1
            msg.width = n_points
            msg.fields = [
                PointField(name="x", offset=0,  datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4,  datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8,  datatype=PointField.FLOAT32, count=1),
                PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            msg.is_bigendian = False
            msg.point_step = 16
            msg.row_step = msg.point_step * n_points
            msg.data = bytes(carla_lidar.raw_data)
            msg.is_dense = True
            self._rsu_lid_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"_on_rsu_lidar_data failed: {e}")

    def _on_rsu_camera_image(self, carla_image):
        """CARLA RGB → sensor_msgs/Image (bgra8). Perception's cv_bridge
        converts to bgr8 transparently, so no manual channel reorder."""
        if self._rsu_cam_pub is None:
            return
        try:
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "rsu_camera"
            msg.height = carla_image.height
            msg.width = carla_image.width
            msg.encoding = "bgra8"
            msg.is_bigendian = 0
            msg.step = carla_image.width * 4
            msg.data = bytes(carla_image.raw_data)
            self._rsu_cam_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"_on_rsu_camera_image failed: {e}")

    def _resolve_actors(self):
        """Lazily look up ego, ped, and (in RSU mode) the RSU pole.

        In rsu_perception mode the ped's CARLA actor is NOT required — the
        bridge gets ped state from the RSU's perception output. But we
        still try to resolve it (best-effort, used only for diagnostics).
        """
        need_rsu = self._ped_source == "rsu_perception"
        if self._ego and self._ped and (not need_rsu or self._rsu_actor):
            return True
        actors = self._world.get_actors()
        if not self._ego:
            for a in actors:
                if a.attributes.get("role_name") == self._ego_role:
                    self._ego = a
                    break
        if not self._ped:
            for a in actors.filter("walker.*"):
                if not self._ped_role or a.attributes.get("role_name") == self._ped_role:
                    self._ped = a
                    break
        if need_rsu and not self._rsu_actor:
            for a in actors:
                if a.attributes.get("role_name") == self._rsu_role:
                    self._rsu_actor = a
                    self.get_logger().info(
                        f"Resolved RSU actor (id={a.id}) at "
                        f"{a.get_transform().location}")
                    break
        ready = bool(self._ego) and (not need_rsu or bool(self._rsu_actor))
        # ped is only strictly needed in ground_truth mode
        if self._ped_source == "ground_truth":
            ready = ready and bool(self._ped)
        if ready:
            ped_id = self._ped.id if self._ped else "?"
            rsu_id = self._rsu_actor.id if self._rsu_actor else "-"
            self.get_logger().info(
                f"Bridge resolved: ego={self._ego.id}, ped={ped_id}, "
                f"rsu={rsu_id}, source={self._ped_source}")
            return True
        return False

    def _on_rsu_detections(self, msg: PoseArray):
        """Receive RSU perception output, transform body→world, cache it.

        The RSU runs the same camera+LiDAR perception node as the ego, so
        its detections come in the RSU's body frame:
            pose.position.x = forward in RSU body frame
            pose.position.y = left+   in RSU body frame
            pose.position.z = up      in RSU body frame
        The RSU is static; its world pose comes from the CARLA actor lookup.
        """
        if not msg.poses or self._rsu_actor is None:
            return
        # Pick the closest detection in body frame — for this scenario there
        # is only one ped, so this is unambiguous.
        best = min(msg.poses,
                   key=lambda p: p.position.x * p.position.x +
                                 p.position.y * p.position.y)
        try:
            rsu_tf = self._rsu_actor.get_transform()
        except Exception:
            return
        yaw_rad = math.radians(rsu_tf.rotation.yaw)
        cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
        # Empirically validated: the perception node's docstring says y_lat
        # is "left-positive", but its projection actually publishes y_lat in
        # CARLA's body-frame Y direction (RIGHT-positive in CARLA's left-
        # handed body frame). The baseline ego AEB doesn't notice because
        # it only uses |y_lat| for corridor checks. For body→world we need
        # the correct sign, so we flip by here before applying the rotation.
        bx = best.position.x
        by = -best.position.y      # see comment above
        bz = best.position.z
        wx = rsu_tf.location.x + cy * bx - sy * by
        wy = rsu_tf.location.y + sy * bx + cy * by
        wz = rsu_tf.location.z + bz
        now = time.time()
        # EMA-smooth the velocity from finite-differences. Reject obvious
        # outliers (>3 m/s — peds don't sprint here) to keep the estimate
        # robust to ground-plane projection jitter.
        if self._rsu_last_world is not None:
            dt = now - self._rsu_last_t
            if 0.05 < dt < 1.0:
                rvx = (wx - self._rsu_last_world[0]) / dt
                rvy = (wy - self._rsu_last_world[1]) / dt
                rvz = (wz - self._rsu_last_world[2]) / dt
                if math.hypot(rvx, rvy) < 3.0:
                    a = 0.3
                    sv = self._rsu_smoothed_vel
                    self._rsu_smoothed_vel = (
                        a * rvx + (1 - a) * sv[0],
                        a * rvy + (1 - a) * sv[1],
                        a * rvz + (1 - a) * sv[2],
                    )
        self._rsu_prev_world = self._rsu_last_world
        self._rsu_prev_t = self._rsu_last_t
        self._rsu_last_world = (wx, wy, wz)
        self._rsu_last_t = now

    @staticmethod
    def _carla_state(actor):
        loc = actor.get_location()
        vel = actor.get_velocity()
        return (loc.x, loc.y, loc.z), (vel.x, vel.y, vel.z)

    def _publish_ego_pose(self, ego_pos, ego_vel):
        """Publish ego pose+velocity as a ROS2 Odometry message.

        Yaw is read directly from the CARLA actor (not from velocity, which is
        zero when the car isn't moving). Encoded as a quaternion in the message.
        """
        if self._ego is None:
            return
        try:
            yaw_deg = self._ego.get_transform().rotation.yaw
        except Exception:
            return
        yaw = math.radians(yaw_deg)
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "map"
        odom.child_frame_id = "ego"
        odom.pose.pose.position.x = ego_pos[0]
        odom.pose.pose.position.y = ego_pos[1]
        odom.pose.pose.position.z = ego_pos[2]
        # quaternion (x=0, y=0, z=sin(yaw/2), w=cos(yaw/2)) for yaw-only rotation
        odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
        odom.twist.twist.linear.x = ego_vel[0]
        odom.twist.twist.linear.y = ego_vel[1]
        odom.twist.twist.linear.z = ego_vel[2]
        self._ego_pose_pub.publish(odom)

    def _rsu_ped_state(self):
        """World-frame ped pose+vel from the cached RSU detection,
        with constant-velocity extrapolation when the RSU drops the track.

        Returns ((x, y, z), (vx, vy, vz)) or None.

        Three regimes, controlled by `rsu_extrapolate_s`:
          * Fresh   (age <= 0.5 s)              — actual last-seen position.
          * Predicted (0.5 s < age <= 3 s)      — extrapolate position from
              the last known velocity (vel = last finite-diff). The ego
              receives CAMs covering the ped's predicted trajectory across
              the road, so corridor-clear release fires naturally even when
              RSU YOLO loses the track mid-crossing. Mirrors what ETSI CPM
              implementations do: the RSU's tracker memory carries the ped
              past brief detection dropouts.
          * Lost    (age > 3 s)                 — stop publishing; the AEB
              falls back to `max_hold_s` as the safety net.
        """
        if self._rsu_last_world is None:
            return None
        age = time.time() - self._rsu_last_t
        if age > self._rsu_extrapolate_s:
            return None

        # EMA-smoothed velocity from `_on_rsu_detections`. Much more
        # stable than a single-step finite-difference, so extrapolation
        # actually carries the ped past the corridor even at the edge of
        # the RSU's detection range.
        vel = self._rsu_smoothed_vel

        if age <= 0.5:
            # Fresh detection — publish the measured position.
            pos = self._rsu_last_world
        else:
            # Predicted: extrapolate from the last seen pose.
            pos = (
                self._rsu_last_world[0] + vel[0] * age,
                self._rsu_last_world[1] + vel[1] * age,
                self._rsu_last_world[2] + vel[2] * age,
            )
        return pos, vel

    def _synthetic_state(self):
        t = time.time() - self._t0
        ego_pos = (-30.0 + 11.0 * t, 0.0, 0.5)
        ego_vel = (11.0, 0.0, 0.0)
        ped_pos = (0.0, -5.0 + 1.4 * t, 1.0)
        ped_vel = (0.0, 1.4, 0.0)
        return ego_pos, ego_vel, ped_pos, ped_vel

    # -------------------------------------------------------------------- main tick

    def _tick(self):
        if self._synthetic:
            ego_pos, ego_vel, ped_pos, ped_vel = self._synthetic_state()
        else:
            if not self._resolve_actors():
                return
            ego_pos, ego_vel = self._carla_state(self._ego)
            if self._ped_source == "rsu_perception":
                ped_state = self._rsu_ped_state()
                if ped_state is None:
                    # RSU hasn't seen the ped yet — fall through without
                    # publishing a CAM. The ego will only react once the
                    # RSU detects the ped, which is the realistic case.
                    self._publish_ego_pose(ego_pos, ego_vel)
                    return
                ped_pos, ped_vel = ped_state
            else:
                ped_pos, ped_vel = self._carla_state(self._ped)
            # Publish ego pose so the AEB doesn't depend on CARLA's ROS2 setup
            self._publish_ego_pose(ego_pos, ego_vel)

        try:
            cam = self._exchange_with_ns3(ego_pos, ego_vel, ped_pos, ped_vel)
        except (ConnectionError, socket.timeout) as e:
            self.get_logger().error(f"ns-3 exchange failed: {e}")
            return

        if cam is None or cam["last_rx_time_s"] < 0:
            return  # no CAM delivered yet

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "map"
        odom.child_frame_id = "ped"
        odom.pose.pose.position.x = cam["ped_x"]
        odom.pose.pose.position.y = cam["ped_y"]
        odom.pose.pose.position.z = cam["ped_z"]
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = cam["ped_vx"]
        odom.twist.twist.linear.y = cam["ped_vy"]
        odom.twist.twist.linear.z = cam["ped_vz"]
        # encode CAM age (s) in twist.angular.z — so AEB can extrapolate forward
        odom.twist.twist.angular.z = cam["msg_age_s"]
        self._pub.publish(odom)

    def _exchange_with_ns3(self, ego_pos, ego_vel, ped_pos, ped_vel):
        ex, ey, ez = ego_pos
        evx, evy, evz = ego_vel
        px, py, pz = ped_pos
        pvx, pvy, pvz = ped_vel
        now = time.time()
        ts = int(now)
        tns = int((now - ts) * 1e9)

        msg = (f"{ts} {tns} "
               f"{ex} {ey} {ez} {evx} {evy} {evz} "
               f"{px} {py} {pz} {pvx} {pvy} {pvz}\r\n")
        self._conn.sendall(msg.encode())

        # ns-3 Gateway sends: "<ego_csv> <ped_csv>\r\n" (one message, fields space-separated)
        while "\r\n" not in self._buf:
            chunk = self._conn.recv(4096).decode()
            if not chunk:
                raise ConnectionError("ns-3 closed the connection")
            self._buf += chunk
        line, self._buf = self._buf.split("\r\n", 1)

        # Two node values joined by a space; we only care about ego (first one).
        node_values = line.split(" ")
        if len(node_values) < 1:
            self.get_logger().warning(f"Empty ns-3 response: {line!r}")
            return None
        parts = node_values[0].split(",")
        if len(parts) < 8:
            self.get_logger().warning(f"Malformed ns-3 response: {lines[0]!r}")
            return None
        return {
            "last_rx_time_s": float(parts[0]),
            "ped_x":  float(parts[1]),
            "ped_y":  float(parts[2]),
            "ped_z":  float(parts[3]),
            "ped_vx": float(parts[4]),
            "ped_vy": float(parts[5]),
            "ped_vz": float(parts[6]),
            "msg_age_s": float(parts[7]),
        }

    def destroy_node(self):
        # Tear down any RSU actors we spawned (camera first, pole last so
        # CARLA doesn't complain about destroying a parent before its child).
        for actor in reversed(self._rsu_spawned_actors):
            try:
                if hasattr(actor, "stop"):
                    actor.stop()
            except Exception:
                pass
            try:
                actor.destroy()
            except Exception:
                pass
        try:
            os.unlink(self._rsu_actor_id_file)
        except OSError:
            pass
        try:
            self._conn.close()
        except Exception:
            pass
        try:
            self._srv.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = V2XBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
