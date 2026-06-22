#!/usr/bin/env python3
"""Tiny relay so the Foxglove layout can point at a stable topic name.

CARLA publishes camera/LiDAR data keyed by actor ID (e.g. /carla/actor309/...),
and the ID changes every run. This node subscribes to the ego's chase camera
on the per-run topic and republishes the frames verbatim on a fixed name
(/dashboard/chase_image) that the layout JSON can hard-code.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ChaseRelay(Node):
    def __init__(self):
        super().__init__("chase_relay")
        self.declare_parameter("ego_actor_id", 0)
        self.declare_parameter("camera_ros_name", "chase")
        self.declare_parameter("output_topic", "/dashboard/chase_image")

        ego_id = int(self.get_parameter("ego_actor_id").value)
        cam_name = str(self.get_parameter("camera_ros_name").value)
        out_topic = str(self.get_parameter("output_topic").value)
        in_topic = f"/carla/actor{ego_id}/{cam_name}/image"

        self._pub = self.create_publisher(Image, out_topic, 5)
        self.create_subscription(Image, in_topic, self._pub.publish, 5)
        self.get_logger().info(f"chase_relay: {in_topic}  →  {out_topic}")


def main(args=None):
    rclpy.init(args=args)
    node = ChaseRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
