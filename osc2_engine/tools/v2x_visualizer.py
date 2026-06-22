#!/usr/bin/env python3
"""V2X visualizer — draws a small wifi-icon above the ego in the CARLA world.

When a CAM has arrived in the last 200 ms the three wifi-style arcs are drawn
in a soft cyan-green; otherwise nothing is drawn. From the chase camera the
arcs appear as nested half-circles above the car's roof, blinking in and out
as the link cycles.

UE4 rendering pitfalls learned the hard way:
  * Bright debug-draw lines bloom into giant glowing balls — keep colour
    channels ≲ 160 and thickness ≲ 0.03.
  * Animated radial expansions create overlapping bright bands. We just
    show / hide the icon based on link state; no expanding rings.

Only spawned when run_sweep.py is invoked with --v2x. Parameters:
    ego_role_name   (string, "hero")
    carla_host      (string, "localhost")
    carla_port      (int,    2000)
    carla_version   (string, "0.9.16")
    icon_height_m   (double, 2.2)   metres above ego's origin
    fresh_window_s  (double, 0.2)   how recent a CAM must be to show the icon
"""

import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class V2XVisualizer(Node):
    def __init__(self):
        super().__init__("v2x_visualizer")

        self.declare_parameter("ego_role_name", "hero")
        self.declare_parameter("carla_host", "localhost")
        self.declare_parameter("carla_port", 2000)
        self.declare_parameter("carla_version", "0.9.16")
        self.declare_parameter("icon_height_m", 2.2)
        self.declare_parameter("fresh_window_s", 0.2)
        self.declare_parameter("osc2_engine_dir",
                               os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

        self._ego_role = self.get_parameter("ego_role_name").value
        self._osc2_dir = str(self.get_parameter("osc2_engine_dir").value)
        self._icon_h = float(self.get_parameter("icon_height_m").value)
        self._fresh_win = float(self.get_parameter("fresh_window_s").value)

        self._setup_carla()
        self._client = self._carla.Client(
            self.get_parameter("carla_host").value,
            int(self.get_parameter("carla_port").value),
        )
        self._client.set_timeout(10.0)
        self._world = self._client.get_world()
        self._debug = self._world.debug

        self._ego_actor = self._find_ego()
        if self._ego_actor is None:
            self.get_logger().warn(
                f"No ego (role={self._ego_role!r}) at startup; will keep trying."
            )

        self._last_cam_time = 0.0
        self.create_subscription(
            Odometry, "/v2x/cam_received", self._on_cam, 10
        )

        self._tick_period = 0.1
        self.create_timer(self._tick_period, self._tick)
        self.get_logger().info("V2X visualizer started (ego wifi-icon mode)")

    # ------------------------------------------------------------------ setup

    def _setup_carla(self):
        if self._osc2_dir not in sys.path:
            sys.path.insert(0, self._osc2_dir)
        from carla_setup import setup_carla
        setup_carla(self.get_parameter("carla_version").value)
        import carla as carla_mod
        self._carla = carla_mod

    def _find_ego(self):
        for a in self._world.get_actors():
            if a.attributes.get("role_name") == self._ego_role:
                return a
        return None

    def _on_cam(self, _msg):
        self._last_cam_time = time.time()

    # ------------------------------------------------------------------ render

    def _tick(self):
        try:
            if self._ego_actor is None:
                self._ego_actor = self._find_ego()
                if self._ego_actor is None:
                    return

            fresh = (time.time() - self._last_cam_time) < self._fresh_win
            if not fresh:
                return  # no signal => no icon

            tf = self._ego_actor.get_transform()
            ego_loc = tf.location
            right = tf.get_right_vector()
            up = tf.get_up_vector()
            center = self._carla.Location(
                x=ego_loc.x,
                y=ego_loc.y,
                z=ego_loc.z + self._icon_h,
            )

            life = self._tick_period + 0.05
            col_inner = self._carla.Color(70, 160, 130)
            col_mid   = self._carla.Color(60, 130, 120)
            col_outer = self._carla.Color(50, 100, 110)

            self._draw_arc(center, right, up, radius=0.22, color=col_inner, life=life)
            self._draw_arc(center, right, up, radius=0.40, color=col_mid,   life=life)
            self._draw_arc(center, right, up, radius=0.60, color=col_outer, life=life)

            # Tiny solid dot at the icon base so something is always centred there.
            self._debug.draw_point(
                center,
                size=0.06,
                color=col_inner,
                life_time=life,
            )
        except Exception as e:  # pragma: no cover
            self.get_logger().warn(f"tick failed: {e}")

    def _draw_arc(self, center, right, up, radius, color, life):
        """Half-circle from right (θ=0) through up (θ=π/2) to left (θ=π).
        Drawn in the plane spanned by `right` and `up` so the chase camera
        (looking from behind the ego) sees nested arcs above the roof."""
        steps = 16
        for i in range(steps):
            t1 = math.pi * i / steps
            t2 = math.pi * (i + 1) / steps
            c1, s1 = math.cos(t1), math.sin(t1)
            c2, s2 = math.cos(t2), math.sin(t2)
            p1 = self._carla.Location(
                x=center.x + radius * (c1 * right.x + s1 * up.x),
                y=center.y + radius * (c1 * right.y + s1 * up.y),
                z=center.z + radius * (c1 * right.z + s1 * up.z),
            )
            p2 = self._carla.Location(
                x=center.x + radius * (c2 * right.x + s2 * up.x),
                y=center.y + radius * (c2 * right.y + s2 * up.y),
                z=center.z + radius * (c2 * right.z + s2 * up.z),
            )
            self._debug.draw_line(p1, p2, thickness=0.02, color=color, life_time=life)


def main(args=None):
    rclpy.init(args=args)
    node = V2XVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
