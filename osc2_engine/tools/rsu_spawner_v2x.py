#!/usr/bin/env python3
"""RSU spawner + sensor bridge — V2X-only sidecar.

Spawns a stationary "RSU pole" at the stop sign on the ego's side of the
crosswalk in vru_ped_occluded_parked_bus_realistic.osc, then attaches an
RGB camera and a 32-channel LiDAR mounted 5 m up.

Why this script also publishes ROS messages directly (rather than relying
on CARLA's --ros2 native bridge to do it):

  CARLA's --ros2 bridge auto-publishes sensors that exist when the bridge
  is initialised, but does not reliably pick up sensors spawned later via
  the Python API. The OSC2 engine spawns the EGO sensors at scenario
  start (so those work), but the RSU is spawned by this side process
  after the scenario is already running. So we use sensor.listen()
  callbacks here and republish to the topic format the perception node
  expects (/carla/actor{rsu_id}/rgb/image and /carla/actor{rsu_id}/
  lidar/point_cloud), so the second perception-node instance can
  subscribe without modification.

The script prints
    RSU_ACTOR_ID=<id>
on stdout so run_sweep.py can capture the actor ID and pass it to the
second perception node, then spins until SIGTERM, cleaning up the
spawned actors on exit.

Placement: stop sign at world (-272.19, -15.06). The bus (south at
y=-18) does NOT occlude the RSU's view of the ped (north at y=-12) —
the whole point of cooperative perception is putting a sensor above
the occlusion.
"""

import argparse
import os
import signal
import struct
import sys
import time

# Add osc2_engine to sys.path so we can import carla_setup
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_OSC2_DIR = os.path.dirname(_THIS_DIR)
if _OSC2_DIR not in sys.path:
    sys.path.insert(0, _OSC2_DIR)

from carla_setup import setup_carla  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import Image, PointCloud2, PointField  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  # noqa: E402


# RSU placement constants — east sidewalk just south of the crosswalk.
# The stop sign in vru_ped_occluded_parked_bus_realistic.osc sits at
# (-272.19, -15.06), but that's inside the ego's driving corridor (ego
# spawns at x≈-271.9 with ~0.92 m half-width, so x=-272.19 clips the
# left edge of the ego). We push 2.7 m east to (-269.5) so the RSU pole
# is clearly off the road, on the east sidewalk, while still having a
# clean angle on the crosswalk and the ped's full west→east trajectory.
RSU_WORLD_X = -269.5
RSU_WORLD_Y = -15.06
RSU_WORLD_Z = 0.5
RSU_YAW_DEG = 90.0      # body +x faces north (toward crosswalk + ped path)

SENSOR_MOUNT_Z = 5.0    # 5 m above the bike — typical RSU camera height
CAMERA_PITCH_DEG = -25.0
# 140° FOV is needed because the ped's bearing from this mount sweeps
# ~117° as it crosses (from ~151° at start to ~34° at end of the path).
CAMERA_FOV_DEG = 140.0
CAMERA_W = 1280
CAMERA_H = 720
LIDAR_CHANNELS = 32
LIDAR_POINTS_PER_SECOND = 600000
LIDAR_ROTATION_HZ = 10
LIDAR_RANGE_M = 100.0

CAMERA_ROS_NAME = "rgb"
LIDAR_ROS_NAME = "lidar"

# Parent actor blueprint: low-profile bike, physics off → static mount.
RSU_PARENT_BLUEPRINT = "vehicle.diamondback.century"
RSU_ROLE_NAME = "rsu"


def _parse_args():
    p = argparse.ArgumentParser(description="V2X RSU spawner + sensor bridge")
    p.add_argument("--carla-host", default="localhost")
    p.add_argument("--carla-port", type=int, default=2000)
    p.add_argument("--carla-version", default="0.9.16",
                   choices=["0.9.16", "0.10.0"])
    return p.parse_args()


class RSUNode(Node):

    def __init__(self, rsu_id):
        super().__init__("rsu_spawner")
        self._rsu_id = rsu_id

        # Best-effort QoS matches what CARLA's --ros2 bridge typically uses
        # for sensor data, so the perception node's sensor_qos config picks
        # up our messages without complaint.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._cam_pub = self.create_publisher(
            Image, f"/carla/actor{rsu_id}/{CAMERA_ROS_NAME}/image", qos)
        self._lidar_pub = self.create_publisher(
            PointCloud2, f"/carla/actor{rsu_id}/{LIDAR_ROS_NAME}/point_cloud",
            qos)
        self.get_logger().info(
            f"Publishing /carla/actor{rsu_id}/{CAMERA_ROS_NAME}/image and "
            f"/carla/actor{rsu_id}/{LIDAR_ROS_NAME}/point_cloud")

    # ------------------------------------------------------------------ CARLA callbacks

    def on_image(self, carla_image):
        """CARLA RGB image (BGRA) → sensor_msgs/Image (bgra8 encoding).

        The perception node converts to bgr8 via cv_bridge, which accepts
        bgra8 input — no manual channel reordering needed.
        """
        try:
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f"rsu_camera_{self._rsu_id}"
            msg.height = carla_image.height
            msg.width = carla_image.width
            msg.encoding = "bgra8"
            msg.is_bigendian = 0
            msg.step = carla_image.width * 4
            msg.data = bytes(carla_image.raw_data)
            self._cam_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"on_image failed: {e}")

    def on_lidar(self, carla_lidar):
        """CARLA LidarMeasurement → sensor_msgs/PointCloud2 with x,y,z,intensity.

        CARLA's raw_data is a packed float32 stream of [x,y,z,intensity] per
        point; we wrap it as a PointCloud2 with the matching field layout
        so the perception node's existing parser picks up x,y,z directly.
        """
        try:
            n = len(carla_lidar)  # number of points
            if n == 0:
                return
            msg = PointCloud2()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f"rsu_lidar_{self._rsu_id}"
            msg.height = 1
            msg.width = n
            msg.fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="intensity", offset=12,
                           datatype=PointField.FLOAT32, count=1),
            ]
            msg.is_bigendian = False
            msg.point_step = 16
            msg.row_step = msg.point_step * n
            msg.is_dense = True
            msg.data = bytes(carla_lidar.raw_data)
            self._lidar_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"on_lidar failed: {e}")


def main():
    args = _parse_args()
    setup_carla(args.carla_version)
    import carla  # noqa: E402

    client = carla.Client(args.carla_host, args.carla_port)
    client.set_timeout(10.0)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()

    # ---- 1. Parent actor (bike, static) ----------------------------------
    parent_bp = bp_lib.find(RSU_PARENT_BLUEPRINT)
    if parent_bp.has_attribute("role_name"):
        parent_bp.set_attribute("role_name", RSU_ROLE_NAME)

    spawn_tf = carla.Transform(
        carla.Location(x=RSU_WORLD_X, y=RSU_WORLD_Y, z=RSU_WORLD_Z),
        carla.Rotation(yaw=RSU_YAW_DEG),
    )
    rsu = None
    for attempt in range(20):
        rsu = world.try_spawn_actor(parent_bp, spawn_tf)
        if rsu is not None:
            break
        time.sleep(0.5)
    if rsu is None:
        print(f"ERROR: failed to spawn RSU parent at {spawn_tf.location}",
              file=sys.stderr)
        sys.exit(1)

    rsu.set_simulate_physics(False)

    print(f"RSU_ACTOR_ID={rsu.id}", flush=True)
    print(f"RSU spawned: actor_id={rsu.id}, world={spawn_tf.location}, "
          f"yaw={RSU_YAW_DEG}deg",
          file=sys.stderr, flush=True)

    # ---- 2. ROS node so we can republish sensor data ---------------------
    rclpy.init()
    node = RSUNode(rsu.id)

    spawned = [rsu]

    # ---- 3. Camera (RGB) -------------------------------------------------
    cam_bp = bp_lib.find("sensor.camera.rgb")
    cam_bp.set_attribute("image_size_x", str(CAMERA_W))
    cam_bp.set_attribute("image_size_y", str(CAMERA_H))
    cam_bp.set_attribute("fov", str(CAMERA_FOV_DEG))
    cam_tf = carla.Transform(
        carla.Location(x=0.0, y=0.0, z=SENSOR_MOUNT_Z),
        carla.Rotation(pitch=CAMERA_PITCH_DEG, yaw=0.0),
    )
    cam = world.spawn_actor(cam_bp, cam_tf, attach_to=rsu)
    spawned.append(cam)
    cam.listen(node.on_image)
    print(f"RSU camera spawned: actor_id={cam.id}, listening to bridge",
          file=sys.stderr, flush=True)

    # ---- 4. LiDAR — disabled for performance --------------------------------
    # CARLA's LiDAR raycaster + a second YOLOv8 instance saturates the GPU
    # enough to push the sim below real-time. The RSU runs camera-only
    # perception (use_lidar_fusion:=false in run_sweep.py), so we skip
    # spawning the LiDAR rather than spawning it just to throw the data
    # away. If you ever re-enable LiDAR fusion on the RSU, restore this
    # block AND drop the second perception node's publish_rate_hz further.
    print("RSU LiDAR: skipped (RSU is camera-only — saves CARLA raycast cost)",
          file=sys.stderr, flush=True)

    # ---- 5. Spin until terminated ---------------------------------------
    _stop = {"flag": False}

    def _handler(signum, _frame):
        _stop["flag"] = True

    signal.signal(signal.SIGTERM, _handler)
    signal.signal(signal.SIGINT, _handler)

    print("RSU spawner: ready, spinning until signal...",
          file=sys.stderr, flush=True)
    try:
        # Spin in small chunks so we can react to the stop flag without
        # blocking forever on a single rclpy.spin() call.
        while not _stop["flag"]:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        print("RSU spawner: cleaning up...", file=sys.stderr, flush=True)
        for actor in reversed(spawned):
            try:
                if hasattr(actor, "stop"):
                    actor.stop()
            except Exception:
                pass
            try:
                actor.destroy()
            except Exception as e:
                print(f"  failed to destroy {actor}: {e}",
                      file=sys.stderr, flush=True)
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
