#!/usr/bin/env python3
"""
Parametric Speed Sweep for VRU Occlusion Scenario.

Runs the parked bus occlusion scenario at multiple ego cruise speeds to
determine the threshold at which AEB can no longer prevent collision.

For each speed:
  1. Starts the AEB ROS2 node with cruise_speed_kmh parameter
  2. Runs the scenario via the OSC2 engine
  3. Parses per-tick CSV to extract detection distance and speed at detection
  4. Collects summary metrics (TTC, min_distance, collision)
  5. Kills the AEB node

Output:
  sweep_results_YYYYMMDD_HHMMSS.csv — one row per speed with all metrics

Usage:
    python3 run_sweep.py
    python3 run_sweep.py --speeds 30 40 50 60 70 80
    python3 run_sweep.py --speeds 40 60 --carla-version 0.9.16
"""

import argparse
import csv
import os
import re
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

# Add engine dir to path
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)


def prompt_interactive(v2x=False, fusion_mode="both"):
    """Interactive prompts for all sweep parameters.

    AEB mode is selected via CLI flags (--v2x, --v2x-fusion), not interactively;
    those flags are passed in here so the rest of the interactive flow can use them.
    """
    print("=" * 60)
    print("  VRU OCCLUSION SPEED SWEEP — Configuration")
    print("=" * 60)

    # CARLA version
    print("\nCARLA version:")
    print("  [1] 0.9.16 (UE4)")
    print("  [2] 0.10.0 (UE5)")
    while True:
        c = input("Select [1-2] (default: 1): ").strip()
        if c in ("", "1"):
            carla_version = "0.9.16"; break
        elif c == "2":
            carla_version = "0.10.0"; break

    # Speeds
    default_speeds = "20 25 30 35 40 45 50 55 60"
    print(f"\nSpeeds to test (km/h):")
    print(f"  Enter values separated by spaces, e.g.: 30 40 50")
    print(f"  Or press Enter for default: {default_speeds}")
    s = input("Speeds: ").strip()
    if s:
        speeds = [float(x) for x in s.split()]
    else:
        speeds = [float(x) for x in default_speeds.split()]

    # Trigger distances — default is d_critical +/- 30% around each speed.
    # d_trigger = sqrt((L/v_p)^2 * v_e^2 + d_lat^2),  L=2.5, v_p=1.4, d_lat=2.5.
    # See docs/trigger_distance_derivation.docx for the derivation.
    L, v_p, d_lat = 2.5, 1.4, 2.5
    multipliers = [0.70, 0.85, 1.00, 1.15, 1.30]
    default_dists = []
    for v_kmh in speeds:
        v_e = v_kmh / 3.6
        d_long = (L / v_p) * v_e
        dcrit = (d_long ** 2 + d_lat ** 2) ** 0.5
        default_dists.extend(round(m * dcrit, 2) for m in multipliers)
    default_dists = sorted(set(default_dists))
    default_distances = " ".join(f"{d:g}" for d in default_dists)

    print(f"\nTrigger distances in meters (ped starts walking when ego is this far):")
    print(f"  Enter values separated by spaces, e.g.: 15 20 30")
    print(f"  Default below is d_critical × [0.70, 0.85, 1.00, 1.15, 1.30]")
    print(f"  computed for the selected speed(s):")
    for v_kmh in speeds:
        v_e = v_kmh / 3.6
        d_long = (L / v_p) * v_e
        dcrit = (d_long ** 2 + d_lat ** 2) ** 0.5
        perts = [round(m * dcrit, 2) for m in multipliers]
        print(f"    {v_kmh:>5g} km/h  d_crit = {dcrit:.2f} m  →  {perts}")
    print(f"  Or press Enter for default: {default_distances}")
    t = input("Distances: ").strip()
    if t:
        trigger_distances = [float(x) for x in t.split()]
    else:
        trigger_distances = list(default_dists)

    # Camera view: Foxglove shows both chase and driver panels, so we no
    # longer prompt. Use chase as the run_scenario.py default.
    view = "chase"

    # Scenario
    scenarios = sorted(Path(os.path.join(script_dir, "scenarios")).glob("*.osc"))
    print("\nAvailable scenarios:")
    for i, sc in enumerate(scenarios, 1):
        marker = " (default)" if "parked_bus_baseline" in sc.name else ""
        print(f"  [{i}] {sc.name}{marker}")
    default_idx = next((i for i, s in enumerate(scenarios, 1) if "parked_bus_baseline" in s.name), 1)
    while True:
        c = input(f"Select [1-{len(scenarios)}] (default: {default_idx}): ").strip()
        if c == "":
            scenario = str(scenarios[default_idx - 1]); break
        try:
            idx = int(c)
            if 1 <= idx <= len(scenarios):
                scenario = str(scenarios[idx - 1]); break
        except ValueError:
            pass

    # Save results to disk?
    out_subdir = "sweep_v2x" if v2x else "sweep"
    print(f"\nSave results to output/{out_subdir}/?")
    print("  [Y] Yes — keep CSVs and per-run logs (default)")
    print("  [N] No  — run in a temp dir, discard everything when done")
    while True:
        c = input("Select [Y/N] (default: Y): ").strip().lower()
        if c in ("", "y", "yes"):
            save_results = True; break
        if c in ("n", "no"):
            save_results = False; break

    return argparse.Namespace(
        speeds=speeds,
        trigger_distances=trigger_distances,
        scenario=scenario,
        carla_version=carla_version,
        view=view,
        output_dir=os.path.join(script_dir, "..", "output", out_subdir),
        save_results=save_results,
        v2x=v2x,
        fusion_mode=fusion_mode,
        host="localhost",
        port=2000,
    )


def parse_args():
    """Use CLI args if provided, otherwise interactive prompts."""
    parser = argparse.ArgumentParser(description="VRU Occlusion Speed Sweep")
    parser.add_argument("--speeds", nargs="+", type=float, default=None)
    parser.add_argument("--trigger-distances", nargs="+", type=float, default=None,
                        help="Trigger distances in meters (e.g.: 10 15 20 25 30)")
    parser.add_argument("--scenario", default=None)
    parser.add_argument("--carla-version", default=None, choices=["0.9.16", "0.10.0"])
    parser.add_argument("--view", default=None, choices=["chase", "driver"])
    parser.add_argument("--output-dir", default=None,
                        help="Output dir (default: output/sweep or output/sweep_v2x with --v2x)")
    parser.add_argument("--no-save", action="store_true",
                        help="Run in a temp dir and discard results (for demos)")
    v2x_group = parser.add_mutually_exclusive_group()
    v2x_group.add_argument("--v2x", action="store_true",
                           help="V2X-only AEB: V2X stack ON, only CAM messages trigger braking "
                                "(LiDAR ignored). ns-3 is launched automatically by this script. "
                                "Output goes to output/sweep_v2x/.")
    v2x_group.add_argument("--v2x-fusion", action="store_true", dest="v2x_fusion",
                           help="V2X + LiDAR fusion AEB: V2X stack ON, either signal triggers "
                                "braking. Same auto-launched ns-3 / output dir as --v2x.")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=2000)

    args = parser.parse_args()
    args.save_results = not args.no_save

    # Derive (v2x, fusion_mode) from the mutually-exclusive flags.
    if args.v2x_fusion:
        args.v2x = True
        args.fusion_mode = "both"
    elif args.v2x:
        args.fusion_mode = "v2x_only"
    else:
        args.fusion_mode = "both"  # unused when v2x is False

    # If no key args provided, go interactive (preserving V2X flags from CLI).
    if args.speeds is None and args.trigger_distances is None and args.carla_version is None:
        return prompt_interactive(v2x=args.v2x, fusion_mode=args.fusion_mode)

    # Fill defaults for CLI mode
    if args.speeds is None:
        args.speeds = [20, 25, 30, 35, 40, 45, 50, 55, 60]
    if args.trigger_distances is None:
        args.trigger_distances = [15.0]
    if args.scenario is None:
        args.scenario = "scenarios/vru_ped_occluded_parked_bus_baseline.osc"
    if args.carla_version is None:
        args.carla_version = "0.9.16"
    if args.view is None:
        args.view = "chase"
    if args.output_dir is None:
        sub = "sweep_v2x" if args.v2x else "sweep"
        args.output_dir = os.path.join(script_dir, "..", "output", sub)

    return args


def prepare_scenario(base_scenario, trigger_distance, output_dir):
    """Create a modified .osc file with the given trigger distance.

    Rewrites the `d_trigger: length = Xm` parameter default. The wait line
    references `d_trigger` symbolically and is left untouched.
    """
    text = Path(base_scenario).read_text()
    import re
    new_text, n = re.subn(
        r'(d_trigger\s*:\s*length\s*=\s*)\d+(\.\d+)?m',
        lambda m: f'{m.group(1)}{trigger_distance:g}m',
        text,
    )
    if n == 0:
        raise ValueError(
            f"Scenario {base_scenario} has no `d_trigger: length = Xm` parameter — "
            "cannot sweep trigger distance."
        )
    out_path = os.path.join(output_dir, "scenario.osc")
    os.makedirs(output_dir, exist_ok=True)
    Path(out_path).write_text(new_text)
    return out_path


def start_aeb_node(speed_kmh, ego_actor_id, v2x=False, fusion_mode="both",
                   ns3_duration=120, carla_version="0.9.16", run_log_dir=None,
                   scenario_label="", trigger_distance_m=0.0):
    """Start AEB node (baseline) OR the V2X stack.

    Returns a list of subprocess.Popen handles so kill_aeb_node can shut everything down.
    Baseline returns the perception + AEB + chase_relay handles. V2X returns the full
    stack: bridge, ns-3, perception, baseline AEB (metrics-only), V2X AEB, chase_relay,
    and the v2x_visualizer — all launched automatically by this script.
    """
    def _log(name):
        """Open a logfile in run_log_dir for the given subprocess name (or PIPE)."""
        if not run_log_dir:
            return subprocess.PIPE
        path = os.path.join(run_log_dir, f"{name}.log")
        return open(path, "w")

    if not v2x:
        # 1) Camera+LiDAR perception (publishes /perception/detections)
        perc_cmd = [
            "ros2", "run", "carla_camera_lidar_perception",
            "camera_lidar_perception",
            "--ros-args",
            "-p", f"agent_actor_id:={ego_actor_id}",
            "-p", "yolo_conf_threshold:=0.25",
        ]
        perc_proc = subprocess.Popen(
            perc_cmd,
            stdout=_log("perception"),
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
        time.sleep(3)  # give YOLO a moment to load weights

        # 2) AEB node (consumes /perception/detections + /ego/speed)
        cmd = [
            "ros2", "run", "carla_aeb_agent", "carla_aeb_agent",
            "--ros-args",
            "-p", f"cruise_speed_kmh:={speed_kmh}",
            "-p", f"ego_actor_id:={ego_actor_id}",
            "-p", f"scenario_label:={scenario_label}",
            "-p", f"trigger_distance_m:={trigger_distance_m}",
        ]
        proc = subprocess.Popen(
            cmd,
            stdout=_log("aeb"),
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )

        # 3) Chase camera relay → /dashboard/chase_image (stable name for layout)
        relay_cmd = [
            "python3",
            os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "tools", "chase_relay.py"),
            "--ros-args",
            "-p", f"ego_actor_id:={ego_actor_id}",
        ]
        relay_proc = subprocess.Popen(
            relay_cmd,
            stdout=_log("chase_relay"),
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )

        time.sleep(2)
        return [perc_proc, proc, relay_proc]

    # ---- V2X stack: bridge (also spawns the RSU) → ns-3 → perception → AEB.
    # RSU cooperative-perception model: a roadside camera is mounted at the
    # stop sign on the ego's side. Its perception output (NOT CARLA ground
    # truth) is what the bridge forwards to ns-3, so the ego's CAM carries
    # realistic perception error + wireless channel delay — the actual
    # deployment story for V2P infrastructure.
    #
    # The bridge now owns RSU spawning (was a separate rsu_spawner_v2x.py
    # process before — folded in so the V2X path uses only ONE CARLA Python
    # client instead of two, ~2× faster).
    procs = []

    # 1) v2x_bridge_node:
    #    - opens TCP server on :8100 for ns-3
    #    - connects to CARLA, spawns RSU pole + camera, writes actor_id to
    #      /tmp/carla_rsu_actor_id, then accepts ns-3
    #    - ped_pose_source=rsu_perception → reads ped state from the RSU
    #      perception node (launched below) instead of CARLA ground truth.
    rsu_actor_id_file = "/tmp/carla_rsu_actor_id"
    try:
        os.unlink(rsu_actor_id_file)
    except OSError:
        pass
    bridge_cmd = [
        "ros2", "run", "carla_v2x_bridge", "carla_v2x_bridge",
        "--ros-args",
        "-p", "ego_role_name:=hero",
        "-p", f"carla_version:={carla_version}",
        "-p", "ped_pose_source:=rsu_perception",
        "-p", "rsu_role_name:=rsu",
        "-p", "rsu_perception_topic:=/rsu/perception/detections",
        "-p", "spawn_rsu:=true",
        "-p", f"rsu_actor_id_file:={rsu_actor_id_file}",
    ]
    # Activity 3 hook: opt-in injection of additional bridge args (e.g. a
    # cam_topic override so the V2X net middlebox can intercept CAMs).
    # Behaviour is bit-for-bit unchanged when the env var is unset.
    _extra = os.environ.get("V2X_BRIDGE_EXTRA_PARAMS", "").strip()
    if _extra:
        bridge_cmd += _extra.split()
    bridge_proc = subprocess.Popen(
        bridge_cmd,
        stdout=_log("bridge"),
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )
    procs.append(bridge_proc)

    # Wait for the bridge to spawn the RSU and write its actor_id.
    rsu_actor_id = None
    deadline = time.time() + 30
    while time.time() < deadline:
        if os.path.exists(rsu_actor_id_file):
            try:
                with open(rsu_actor_id_file) as f:
                    content = f.read().strip()
                if content:
                    rsu_actor_id = int(content)
                    print(f"  RSU ID: {rsu_actor_id}")
                    break
            except (ValueError, OSError):
                pass
        if bridge_proc.poll() is not None:
            print("  ERROR: bridge died before spawning RSU.")
            kill_aeb_node(procs)
            return []
        time.sleep(0.5)
    if rsu_actor_id is None:
        print("  ERROR: bridge did not write RSU actor_id within 30 s")
        kill_aeb_node(procs)
        return []

    # 2) ns-3 V2P gateway — launched in-process so a single command runs the
    # whole stack. Output goes to ns3.log in the run dir. If it fails, check
    # ns3.log first; the standalone tools/run_ns3_v2x.sh script can be used
    # to reproduce the launch manually for debugging.
    ns3_cmd = [
        "bash", "-c",
        f'cd ~/iotav/ns-3-dev && ./ns3 run "gateway-v2p-wifi --verbose --duration={ns3_duration}"',
    ]
    ns3_proc = subprocess.Popen(
        ns3_cmd,
        stdout=_log("ns3"),
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )
    procs.append(ns3_proc)

    bridge_log = os.path.join(run_log_dir, "bridge.log") if run_log_dir else None
    deadline = time.time() + 60
    connected = False
    print("  Waiting for ns-3 to connect to the V2X bridge ...")
    while time.time() < deadline:
        if bridge_log and os.path.exists(bridge_log):
            try:
                with open(bridge_log) as f:
                    if "ns-3 connected" in f.read():
                        connected = True
                        break
            except OSError:
                pass
        if bridge_proc.poll() is not None:
            print("  ERROR: bridge died before ns-3 connected.")
            break
        if ns3_proc.poll() is not None:
            print("  ERROR: ns-3 exited before connecting — see ns3.log")
            break
        time.sleep(0.5)
    if connected:
        print("  ns-3 connected — starting AEB-V2X.")
    else:
        print("  WARNING: ns-3 did not connect within 60s, continuing anyway.")

    # 3a) Ego perception is SKIPPED in V2X mode — the V2X AEB consumes only
    # CAMs from the bridge, not /perception/detections, so running YOLO on
    # the ego's camera is pure overhead that halves CARLA's frame rate.
    # The dashboard's "ego raw camera" panel still works because CARLA's
    # --ros2 native bridge publishes /carla/actor{ego_id}/rgb/image
    # directly; only the YOLO-annotated /perception/image_debug is lost,
    # and that's irrelevant in V2X-only mode where the ego doesn't see.
    # 3b) RSU camera+LiDAR perception — SAME node, second instance pointed
    # at the RSU's sensors. Output topics are remapped to /rsu/perception/*
    # so the bridge can subscribe to /rsu/perception/detections.
    # Camera intrinsics MUST match what v2x_bridge_node.py attaches
    # (1280x720 RGB, FOV=140°, pitch=-18°, mount z=2.5m).
    # Mount height = real stop-sign top height; the RSU pole IS the
    # in-map stop sign, not a 5 m signal-arm mast.
    # YOLO model: same YOLOv8m + conf=0.25 as the ego perception baseline.
    # The RSU runs the SAME detector as the ego — only the mounting point
    # differs. This catches small/distant pedestrians (the ped walks
    # straight away from this camera, so the bbox shrinks to ~30 px once
    # they've crossed; YOLOv8n was missing those, YOLOv8m holds the track).
    # Simulation will be slower than with yolov8n, but ego perception is
    # disabled in V2X mode so the GPU has the headroom.
    rsu_perc_cmd = [
        "ros2", "run", "carla_camera_lidar_perception",
        "camera_lidar_perception",
        "--ros-args",
        "-r", "__node:=camera_lidar_perception_rsu",
        "-p", f"agent_actor_id:={rsu_actor_id}",
        "-p", "yolo_model:=/home/hnh21/iotav/CARLA_0.10/models/yolov8m.pt",
        "-p", "yolo_conf_threshold:=0.25",
        "-p", "publish_rate_hz:=5.0",
        "-p", "use_lidar_fusion:=true",
        "-p", "output_topic:=/rsu/perception/detections",
        "-p", "debug_image_topic:=/rsu/perception/image_debug",
        "-p", "image_width:=1280",
        "-p", "image_height:=720",
        "-p", "camera_fov_deg:=140.0",
        "-p", "camera_mount_x_m:=0.0",
        "-p", "camera_mount_y_m:=0.0",
        "-p", "camera_mount_z_m:=2.5",
        "-p", "camera_pitch_deg:=-18.0",
        "-p", "lidar_mount_z_m:=2.5",
        "-p", "max_detection_range_m:=30.0",
    ]
    rsu_perc_proc = subprocess.Popen(
        rsu_perc_cmd,
        stdout=_log("rsu_perception"),
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )
    procs.append(rsu_perc_proc)
    time.sleep(3)  # YOLO load (both instances share weights cache)

    # 4a) baseline AEB in metrics-only mode — feeds dashboard /metrics/* topics
    # (state, ped_distance, throttle, brake, scenario_info, etc.) while the
    # V2X AEB owns vehicle control. Without this the dashboard panels are blank.
    metrics_cmd = [
        "ros2", "run", "carla_aeb_agent", "carla_aeb_agent",
        "--ros-args",
        "-p", f"cruise_speed_kmh:={speed_kmh}",
        "-p", f"ego_actor_id:={ego_actor_id}",
        "-p", f"scenario_label:={scenario_label}",
        "-p", f"trigger_distance_m:={trigger_distance_m}",
        "-p", "metrics_only:=true",
    ]
    metrics_proc = subprocess.Popen(
        metrics_cmd,
        stdout=_log("aeb_metrics"),
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )
    procs.append(metrics_proc)

    # 4b) aeb_node_v2x — V2X-aware AEB (publishes vehicle control)
    aeb_cmd = [
        "ros2", "run", "carla_aeb_v2x_agent", "carla_aeb_v2x_agent",
        "--ros-args",
        "-p", f"cruise_speed_kmh:={speed_kmh}",
        "-p", f"ego_actor_id:={ego_actor_id}",
        "-p", f"fusion_mode:={fusion_mode}",
    ]
    aeb_proc = subprocess.Popen(
        aeb_cmd,
        stdout=_log("aeb_v2x"),
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )
    procs.append(aeb_proc)

    # 5) Chase camera relay → /dashboard/chase_image
    relay_cmd = [
        "python3",
        os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "tools", "chase_relay.py"),
        "--ros-args",
        "-p", f"ego_actor_id:={ego_actor_id}",
    ]
    relay_proc = subprocess.Popen(
        relay_cmd,
        stdout=_log("chase_relay"),
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )
    procs.append(relay_proc)

    # 6) V2X visualizer — draws RSU pulse + CAM-delivery line in the CARLA
    # world. Only spawned in V2X mode; the baseline run has no broadcast.
    viz_cmd = [
        "python3",
        os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "tools", "v2x_visualizer.py"),
        "--ros-args",
        "-p", f"carla_version:={carla_version}",
    ]
    viz_proc = subprocess.Popen(
        viz_cmd,
        stdout=_log("v2x_visualizer"),
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )
    procs.append(viz_proc)
    time.sleep(2)

    return procs


def pkill_v2x_stragglers():
    """Belt-and-braces cleanup: kill any V2X bridge / AEB / ns-3 processes
    still alive in this user's process tree, regardless of how the previous
    sweep cell left them. Prevents 'port 8100 already in use' between cells."""
    patterns = [
        "carla_v2x_bridge",
        "aeb_v2x_node",
        "carla_aeb_v2x_agent",
        "carla_aeb_agent",
        "camera_lidar_perception",
        "chase_relay",
        "v2x_visualizer",
        "gateway-v2p",
    ]
    for pat in patterns:
        try:
            subprocess.run(["pkill", "-f", pat], check=False, timeout=5)
        except Exception:
            pass


def kill_aeb_node(procs):
    """Kill AEB process group(s). Accepts a single Popen or a list.

    First tries graceful SIGTERM on each process group (so children get a
    chance to clean up sockets), then escalates to SIGKILL, and finally
    does a `pkill -f` sweep to catch any orphans that survived because they
    detached from the process group (this happens with some ROS2 launchers).
    """
    if not isinstance(procs, list):
        procs = [procs]
    # Kill in reverse order: AEB / visualizer first, then perception, ns-3, bridge.
    for proc in reversed(procs):
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=5)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass
    # Sweep any straggler V2X processes that survived the killpg above.
    # Without this, the next cell hits "Address already in use" on port 8100.
    pkill_v2x_stragglers()
    time.sleep(0.5)  # let the OS release the bound socket


def run_scenario_with_aeb(args, speed_kmh, run_output_dir, trigger_dist=0.0,
                          scenario_label=""):
    """Run scenario + AEB. Returns (stdout_text, return_code).

    1. Remove old actor ID file
    2. Start scenario (background) — spawns ego, writes ID to /tmp/carla_ego_actor_id
    3. Wait for actor ID file
    4. Start AEB with that ID
    5. Wait for scenario to finish
    6. Kill AEB
    """
    id_file = "/tmp/carla_ego_actor_id"
    try:
        os.remove(id_file)
    except FileNotFoundError:
        pass

    # Defensive: kill any V2X stragglers from a prior cell that didn't tear
    # down cleanly. Cheap if there are none, prevents port 8100 conflicts.
    pkill_v2x_stragglers()

    # Reset CARLA to async mode before each run (in case previous run crashed)
    try:
        sys.path.insert(0, script_dir)
        from carla_setup import setup_carla
        config = setup_carla(args.carla_version)
        import carla
        client = carla.Client(args.host, args.port)
        client.set_timeout(10)
        world = client.get_world()
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)
    except Exception:
        pass

    # Start scenario in background (use absolute paths)
    run_scenario_path = os.path.join(script_dir, "run_scenario.py")
    cmd = [
        sys.executable, run_scenario_path,
        args.scenario,
        "--carla-version", args.carla_version,
        "--view", args.view,
        "--host", args.host,
        "--port", str(args.port),
        "--output-dir", run_output_dir,
    ]
    scen_proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        cwd=script_dir,
    )

    # Wait for actor ID file (max 60s — map loading can take 10-15s)
    ego_id = None
    for i in range(120):
        time.sleep(0.5)
        # Check if scenario subprocess died
        if scen_proc.poll() is not None:
            break
        if os.path.exists(id_file):
            try:
                with open(id_file) as f:
                    content = f.read().strip()
                    if content:
                        ego_id = int(content)
                        break
            except (ValueError, IOError):
                pass

    aeb_procs = None
    if ego_id is not None:
        v2x = getattr(args, "v2x", False)
        fusion = getattr(args, "fusion_mode", "both")
        if v2x:
            print(f"  Ego actor ID: {ego_id}")
            print(f"  Starting V2X stack ...")
        else:
            print(f"  Ego actor ID: {ego_id}")
            print(f"  Starting AEB ...")
        aeb_procs = start_aeb_node(speed_kmh, ego_id, v2x=v2x, fusion_mode=fusion,
                                   carla_version=args.carla_version,
                                   run_log_dir=run_output_dir,
                                   scenario_label=scenario_label,
                                   trigger_distance_m=trigger_dist)
    else:
        # Check if scenario already finished (crashed or completed too fast)
        rc = scen_proc.poll()
        print(f"    WARNING: ego actor ID not found (scenario rc={rc}), AEB not started")

    # Wait for scenario to finish. The V2X-RSU path runs at ~3.5% real-time
    # because the second perception node + RSU sensor bridge load CARLA more
    # than the perception-only baseline. A 17.54 m / 35 km/h scenario takes
    # ~10 sim seconds → ~5 min wallclock at the slower rate, so allow 360 s.
    scen_timeout_s = 360 if getattr(args, "v2x", False) else 120
    try:
        stdout, _ = scen_proc.communicate(timeout=scen_timeout_s)
    except subprocess.TimeoutExpired:
        scen_proc.kill()
        stdout, _ = scen_proc.communicate()

    # Kill AEB / V2X stack
    if aeb_procs:
        kill_aeb_node(aeb_procs)

    return stdout, scen_proc.returncode


def parse_scenario_output(stdout):
    """Extract key metrics from scenario stdout."""
    result = {
        "detection_distance_m": None,
        "speed_at_detection_kmh": None,
    }

    # Parse detection log: "Pedestrian DETECTED at (...), distance=17.8m"
    det_match = re.search(r"Pedestrian DETECTED at .+distance=(\d+\.?\d*)m", stdout)
    if det_match:
        result["detection_distance_m"] = float(det_match.group(1))

    # Parse AEB brake log for speed: "speed=29.2 km/h" or from detection
    # Get speed from per-second log at detection time
    speed_match = re.search(r"AEB BRAKE.*speed=(\d+\.?\d*)\s*km/h", stdout)
    if speed_match:
        result["speed_at_detection_kmh"] = float(speed_match.group(1))
    else:
        # Try from the second-by-second log near detection
        det_speed = re.search(r"PED DETECTED.*speed=(\d+\.?\d*)\s*km/h", stdout)
        if det_speed:
            result["speed_at_detection_kmh"] = float(det_speed.group(1))

    return result


def parse_summary_csv(run_output_dir):
    """Read the latest summary CSV in the output directory."""
    summary_files = sorted(Path(run_output_dir).glob("*_summary_*.csv"))
    if not summary_files:
        return None

    latest = summary_files[-1]
    with open(latest) as f:
        reader = csv.DictReader(f)
        row = next(reader)

    def safe_float(val):
        if val is None or val == "N/A" or val == "inf":
            return None
        try:
            return float(val)
        except (ValueError, TypeError):
            return None

    return {
        "ttc_min_s": safe_float(row.get("ttc_min_s", row.get("ttc_min"))),
        "pet_s": safe_float(row.get("pet_s", row.get("pet"))),
        "min_distance_m": safe_float(row.get("min_distance_m", row.get("min_distance"))),
        "collision": row.get("collision", row.get("collision_detected", "")) == "True",
        "near_miss": row.get("near_miss", "") == "True",
        "near_miss_distance_m": safe_float(row.get("near_miss_distance_m")),
        "path_blocking_m": safe_float(row.get("path_blocking_m")),
        "lateral_clearance_min_m": safe_float(row.get("lateral_clearance_min_m")),
        "collision_speed_kmh": safe_float(row.get("collision_speed_kmh")),
        "max_decel_ms2": safe_float(row.get("max_decel_ms2")),
        "max_jerk_ms3": safe_float(row.get("max_jerk_ms3")),
        "speed_at_detection_kmh": safe_float(row.get("speed_at_detection_kmh")),
        "distance_at_detection_m": safe_float(row.get("distance_at_detection_m")),
        "stopping_distance_m": safe_float(row.get("stopping_distance_m")),
        "brake_reaction_time_s": safe_float(row.get("brake_reaction_time_s")),
        "total_time_s": safe_float(row.get("total_time_s", row.get("total_time"))),
    }


def classify_outcome(row):
    """Three-tier outcome classification.

    SAFE       — AEB stopped the ego SHORT of the pedestrian's path. The
                 ped had room to walk across in front of the ego (the STAY
                 / mutual-yield case).
    NEAR_MISS  — AEB triggered but the ego came to rest AT the ped's level,
                 blocking the path. The ego then has to creep forward to
                 clear the crosswalk before the ped can pass (the CREEP /
                 near-miss-resume case).
    COLLISION  — physical contact registered.

    Discriminator is `path_blocking_m`, the signed distance from the ego
    front bumper to the ped clearance line at the ego's slowest moment.
    PET and lateral clearance are kept as informational columns but do
    NOT drive the classification — the spatial blocking question is the
    one the user cares about.
    """
    if row.get("collision"):
        return "COLLISION"
    blocking = row.get("path_blocking_m")
    # blocking is None when the AEB never engaged a hard brake (no
    # detection_tick). If there was also no collision, the ego either
    # didn't see the ped or didn't need to brake — either way the ped
    # was not hit, so the outcome is SAFE.
    if blocking is None:
        return "SAFE"
    return "NEAR_MISS" if blocking > 0.0 else "SAFE"


def main():
    args = parse_args()

    # Output directory — either the normal sweep/ folder or a throwaway temp dir
    save_results = getattr(args, "save_results", True)
    if save_results:
        os.makedirs(args.output_dir, exist_ok=True)
    else:
        import tempfile
        args.output_dir = tempfile.mkdtemp(prefix="sweep_demo_")

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    speeds_tag = "_".join(f"{s:g}kmh" for s in args.speeds)
    sweep_csv = os.path.join(args.output_dir, f"sweep_results_{speeds_tag}_{timestamp}.csv")

    total_runs = len(args.trigger_distances) * len(args.speeds)
    v2x_on = getattr(args, "v2x", False)
    print("=" * 70)
    if v2x_on:
        print("  VRU OCCLUSION SPEED x DISTANCE SWEEP — V2X ENABLED")
        print(f"  Fusion mode: {getattr(args, 'fusion_mode', 'both')}")
        print(f"  ns-3:        launched automatically per run (gateway-v2p-wifi)")
    else:
        print("  VRU OCCLUSION SPEED x DISTANCE SWEEP")
    print(f"  Speeds: {args.speeds} km/h")
    print(f"  Trigger distances: {args.trigger_distances} m")
    print(f"  Total runs: {total_runs}")
    print(f"  Scenario: {args.scenario}")
    if save_results:
        print(f"  Output: {sweep_csv}")
    else:
        print(f"  Output: [demo mode — discarded when done, temp dir {args.output_dir}]")
    print("=" * 70)

    results = []
    run_num = 0

    for dist in args.trigger_distances:
        print(f"\n{'=' * 70}")
        print(f"  TRIGGER DISTANCE: {dist:g}m")
        print(f"{'=' * 70}")

        for speed in args.speeds:
            run_num += 1
            print(f"\n{'─' * 70}")
            print(f"  Run {run_num}/{total_runs}: {speed:g} km/h @ {dist:g}m trigger")
            print(f"{'─' * 70}")

            run_dir = os.path.join(args.output_dir, f"run_{dist:g}m_{speed:g}kmh")
            os.makedirs(run_dir, exist_ok=True)

            scenario_path = prepare_scenario(args.scenario, dist, run_dir)

            try:
                print(f"  Running scenario ...")
                args_copy = argparse.Namespace(**vars(args))
                args_copy.scenario = scenario_path
                scen_label = os.path.splitext(os.path.basename(args.scenario))[0]
                stdout, rc = run_scenario_with_aeb(
                    args_copy, speed, run_dir,
                    trigger_dist=dist, scenario_label=scen_label,
                )

                log_path = os.path.join(run_dir, "scenario_log.txt")
                with open(log_path, "w") as f:
                    f.write(stdout)

                if rc != 0:
                    print(f"  WARNING: scenario exited with code {rc}")

                output_metrics = parse_scenario_output(stdout)
                summary = parse_summary_csv(run_dir)

                row = {
                    "target_speed_kmh": speed,
                    "trigger_distance_m": dist,
                }
                if summary:
                    row.update(summary)
                else:
                    row.update({k: None for k in [
                        "ttc_min_s", "pet_s", "min_distance_m", "collision", "near_miss", "near_miss_distance_m",
                        "path_blocking_m", "lateral_clearance_min_m",
                        "collision_speed_kmh", "max_decel_ms2", "max_jerk_ms3",
                        "speed_at_detection_kmh", "distance_at_detection_m",
                        "stopping_distance_m", "brake_reaction_time_s", "total_time_s",
                    ]})
                row["outcome"] = classify_outcome(row) if row.get("collision") is not None else None
                results.append(row)

                if row["collision"] is not None:
                    outcome_str = row["outcome"] or "UNKNOWN"
                    md = f"{row['min_distance_m']:.1f}" if row['min_distance_m'] is not None else "N/A"
                    ttc = f"{row['ttc_min_s']:.2f}" if row['ttc_min_s'] is not None else "N/A"
                    lat = row.get("lateral_clearance_min_m")
                    lat_s = f"{lat:+.1f}m" if lat is not None else "N/A"
                    print(f"  Result: {outcome_str}, min_dist={md}m, TTC={ttc}s, lat_clr={lat_s}")
                else:
                    print(f"  Result: FAILED (no metrics)")

            except subprocess.TimeoutExpired:
                print(f"  ERROR: scenario timed out")
                results.append({
                    "target_speed_kmh": speed,
                    "trigger_distance_m": dist,
                    "min_distance_m": None,
                    "ttc_min_s": None,
                    "collision": None,
                    "total_time_s": None,
                })
            finally:
                time.sleep(2)

    # Write combined CSV
    fieldnames = [
        "target_speed_kmh", "trigger_distance_m", "outcome",
        "ttc_min_s", "pet_s", "min_distance_m", "collision",
        "near_miss", "near_miss_distance_m",
        "path_blocking_m", "lateral_clearance_min_m",
        "collision_speed_kmh", "max_decel_ms2", "max_jerk_ms3",
        "speed_at_detection_kmh", "distance_at_detection_m",
        "stopping_distance_m", "brake_reaction_time_s", "total_time_s",
    ]
    with open(sweep_csv, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(results)

    # Print summary table grouped by trigger distance
    def fmt(val, precision=1):
        if val is None:
            return "N/A"
        if isinstance(val, float) and val == float('inf'):
            return "inf"
        return f"{val:.{precision}f}"

    header = (f"  {'Speed':>6s}  {'Trig':>5s}  {'MinDist':>8s}  {'TTC':>6s}  {'PET':>6s}  "
              f"{'LatClr':>7s}  {'Decel':>7s}  {'StopD':>6s}  {'Outcome':>10s}")
    units = (f"  {'km/h':>6s}  {'m':>5s}  {'m':>8s}  {'s':>6s}  {'s':>6s}  "
             f"{'m':>7s}  {'m/s²':>7s}  {'m':>6s}  {'':>10s}")
    separator = (f"  {'─'*6}  {'─'*5}  {'─'*8}  {'─'*6}  {'─'*6}  "
                 f"{'─'*7}  {'─'*7}  {'─'*6}  {'─'*10}")

    print(f"\n{'=' * 100}")
    print(f"  SWEEP RESULTS — Speed x Trigger Distance")
    print(f"  Outcome: COLLISION | NEAR_MISS | SAFE")
    print(f"{'=' * 100}")
    print(header)
    print(units)
    print(separator)

    for dist in args.trigger_distances:
        dist_results = [r for r in results if r.get('trigger_distance_m') == dist]
        for r in dist_results:
            result_str = r.get('outcome') or "FAILED"
            print(f"  {r['target_speed_kmh']:>6g}  "
                  f"{r['trigger_distance_m']:>5g}  "
                  f"{fmt(r.get('min_distance_m')):>8s}  "
                  f"{fmt(r.get('ttc_min_s'), 2):>6s}  "
                  f"{fmt(r.get('pet_s'), 2):>6s}  "
                  f"{fmt(r.get('lateral_clearance_min_m'), 2):>7s}  "
                  f"{fmt(r.get('max_decel_ms2')):>7s}  "
                  f"{fmt(r.get('stopping_distance_m')):>6s}  "
                  f"{result_str:>10s}")
        if dist != args.trigger_distances[-1]:
            print(separator)

    # Print collision boundary summary
    print(f"\n{'─' * 100}")
    print(f"  SAFETY BOUNDARY (max safe speed per trigger distance):")
    for dist in args.trigger_distances:
        dist_results = [r for r in results if r.get('trigger_distance_m') == dist]
        safe_speeds = [r['target_speed_kmh'] for r in dist_results
                       if r.get('outcome') == "SAFE"]
        near_miss_speeds = [r['target_speed_kmh'] for r in dist_results
                           if r.get('outcome') == "NEAR_MISS"]
        collision_speeds = [r['target_speed_kmh'] for r in dist_results
                           if r.get('outcome') == "COLLISION"]
        max_safe = max(safe_speeds) if safe_speeds else 0
        nm_str = f", near miss at {near_miss_speeds} km/h" if near_miss_speeds else ""
        col_str = f", collision at {collision_speeds} km/h" if collision_speeds else ""
        print(f"    {dist:g}m trigger -> safe up to {max_safe:g} km/h{nm_str}{col_str}")

    if save_results:
        print(f"\n  Results saved: {sweep_csv}")
        print(f"  Per-run data: {args.output_dir}/")
    else:
        import shutil
        shutil.rmtree(args.output_dir, ignore_errors=True)
        print(f"\n  Demo mode — output directory discarded (nothing saved)")
    print(f"{'=' * 100}")


if __name__ == "__main__":
    main()
