#!/usr/bin/env python3
"""
V2X NETWORK MIDDLEBOX — Activity 3 perturbations.

Standalone ROS 2 node that sits between the V2X bridge and the AEB-V2X agent
to emulate imperfect wireless channels (CAM/CPM as nav_msgs/Odometry).

Topology when active (no change to bridge or AEB code):

        carla_v2x_bridge ──► /v2x/cam_received_raw
                                     │
                                     ▼
                               (this node)
                              drop / delay / noise
                                     │
                                     ▼
        /v2x/cam_received ──► aeb_v2x_node

Activated by run_sweep_MC_net.py only. Without it, the bridge still publishes
straight to /v2x/cam_received and this node is never started, so default
behaviour is bit-for-bit identical to the clean V2X sweep.

Parameters (CLI flags):
    --in-topic       (default /v2x/cam_received_raw)
    --out-topic      (default /v2x/cam_received)
    --drop-prob      (0.0 .. 1.0)  Bernoulli per-message drop probability
    --delay-ms       (>= 0)        extra wall-clock delay before re-publishing
    --pos-noise-m    (>= 0)        Gaussian sigma added to ped x/y in metres
    --burst-start-s  (>= 0)        wall-clock start time of a blackout window
                                   (relative to node startup). 0 = disabled.
    --burst-dur-s    (>= 0)        blackout duration. During the window EVERY
                                   incoming CAM is dropped (correlated loss).
    --cam-rate-hz    (>= 0)        downsample output to this rate (Hz). 0 or
                                   >=10 = pass-through (input is already 10 Hz).
    --seed           (int)         RNG seed for reproducibility
    --report-every   (int, default 100)  log a stats line every N input msgs

Usage:
    ros2 launch nothing — invoke directly:
    python3 v2x_net_middlebox.py --drop-prob 0.3 --delay-ms 80
    python3 v2x_net_middlebox.py --burst-start-s 4.0 --burst-dur-s 3.0
    python3 v2x_net_middlebox.py --cam-rate-hz 1.0
"""

import argparse
import random
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class V2XNetMiddlebox(Node):
    def __init__(self, opts):
        super().__init__("v2x_net_middlebox")
        self._opts = opts
        self._rng = random.Random(opts.seed)
        self._n_in = 0
        self._n_dropped = 0          # Bernoulli + burst combined
        self._n_dropped_burst = 0
        self._n_dropped_rate = 0
        self._n_out = 0
        self._lock = threading.Lock()
        self._t0 = time.monotonic()  # node-startup reference for burst window
        self._last_pub_t = 0.0       # for rate-cap downsampling

        # Pre-compute min interval between published CAMs (rate cap).
        # 0 or >=10 Hz means: no downsampling (input is already 10 Hz).
        self._min_interval = 0.0
        if 0.0 < opts.cam_rate_hz < 10.0:
            self._min_interval = 1.0 / opts.cam_rate_hz

        self._pub = self.create_publisher(Odometry, opts.out_topic, 10)
        self._sub = self.create_subscription(
            Odometry, opts.in_topic, self._on_cam, 10)

        self.get_logger().info(
            f"V2X middlebox up — in={opts.in_topic} out={opts.out_topic} "
            f"drop_prob={opts.drop_prob} delay_ms={opts.delay_ms} "
            f"pos_noise_m={opts.pos_noise_m} "
            f"burst=[{opts.burst_start_s}s, +{opts.burst_dur_s}s] "
            f"cam_rate_hz={opts.cam_rate_hz} seed={opts.seed}")

    def _on_cam(self, msg: Odometry):
        with self._lock:
            self._n_in += 1

        now = time.monotonic()

        # 1a) Burst loss — deterministic blackout window. During the window
        # EVERY incoming CAM is dropped (correlated channel outage, e.g.
        # NLOS occlusion by a passing truck).
        if self._opts.burst_dur_s > 0.0:
            elapsed = now - self._t0
            if (self._opts.burst_start_s <= elapsed
                    <= self._opts.burst_start_s + self._opts.burst_dur_s):
                with self._lock:
                    self._n_dropped += 1
                    self._n_dropped_burst += 1
                self._maybe_report()
                return

        # 1b) Bernoulli drop
        if self._opts.drop_prob > 0.0 and self._rng.random() < self._opts.drop_prob:
            with self._lock:
                self._n_dropped += 1
            self._maybe_report()
            return

        # 1c) Rate cap — downsample by enforcing a minimum interval between
        # published CAMs. Catches the "channel can only carry N Hz" case.
        if self._min_interval > 0.0:
            if (now - self._last_pub_t) < self._min_interval:
                with self._lock:
                    self._n_dropped += 1
                    self._n_dropped_rate += 1
                self._maybe_report()
                return
            self._last_pub_t = now

        # 2) Additive Gaussian noise on ped position (CAM payload abuse-of-
        # nav_msgs/Odometry: ped pose lives in pose.pose.position.x / .y)
        if self._opts.pos_noise_m > 0.0:
            msg.pose.pose.position.x += self._rng.gauss(0.0, self._opts.pos_noise_m)
            msg.pose.pose.position.y += self._rng.gauss(0.0, self._opts.pos_noise_m)

        # 3) Extra delay → schedule the publish on a worker thread so we
        # don't block this rclpy callback.
        if self._opts.delay_ms > 0:
            t = threading.Timer(self._opts.delay_ms / 1000.0,
                                self._publish, args=(msg,))
            t.daemon = True
            t.start()
        else:
            self._publish(msg)

        self._maybe_report()

    def _publish(self, msg):
        self._pub.publish(msg)
        with self._lock:
            self._n_out += 1

    def _maybe_report(self):
        if self._opts.report_every <= 0:
            return
        with self._lock:
            n_in = self._n_in
            n_drop = self._n_dropped
            n_out = self._n_out
        if n_in % self._opts.report_every == 0:
            with self._lock:
                n_burst = self._n_dropped_burst
                n_rate  = self._n_dropped_rate
            self.get_logger().info(
                f"[stats] in={n_in} dropped={n_drop} "
                f"(burst={n_burst} rate={n_rate}) "
                f"({(n_drop/n_in*100 if n_in else 0):.1f}%) out={n_out}")


def parse_args():
    p = argparse.ArgumentParser(description="V2X CAM perturbation middlebox")
    p.add_argument("--in-topic",  default="/v2x/cam_received_raw")
    p.add_argument("--out-topic", default="/v2x/cam_received")
    p.add_argument("--drop-prob",     type=float, default=0.0)
    p.add_argument("--delay-ms",      type=float, default=0.0)
    p.add_argument("--pos-noise-m",   type=float, default=0.0)
    p.add_argument("--burst-start-s", type=float, default=0.0,
                   help="Wall-clock start of correlated-loss window "
                        "(relative to node startup). 0 = disabled.")
    p.add_argument("--burst-dur-s",   type=float, default=0.0,
                   help="Duration of correlated-loss window (s). "
                        "0 = disabled.")
    p.add_argument("--cam-rate-hz",   type=float, default=0.0,
                   help="Downsample output to this rate (Hz). "
                        "0 or >=10 = pass-through.")
    p.add_argument("--seed",          type=int,   default=42)
    p.add_argument("--report-every",  type=int,   default=100)
    # rclpy sometimes injects --ros-args + extras; absorb them.
    args, _unknown = p.parse_known_args()
    return args


def main():
    opts = parse_args()
    rclpy.init()
    node = V2XNetMiddlebox(opts)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f"V2X middlebox shutting down — total in={node._n_in} "
            f"dropped={node._n_dropped} "
            f"(burst={node._n_dropped_burst} rate={node._n_dropped_rate}) "
            f"out={node._n_out}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
