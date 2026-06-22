#!/usr/bin/env python3
"""
OSC2 Scenario Runner for CARLA

Parses an OpenSCENARIO 2.0 (.osc) file and executes it against a live CARLA server.
Supports both scripted (BehaviorAgent) and ROS2 (external AEB) ego control.
The mode is determined by the ego_control parameter in the .osc scenario.

Usage:
    python3 run_scenario.py scenarios/vru_crossing.osc
    python3 run_scenario.py scenarios/vru_ped_occluded_parked_bus_baseline.osc --carla-version 0.9.16
    python3 run_scenario.py scenarios/vru_crossing.osc --output-dir ./output -v
"""

import argparse
import logging
import os
import sys
import time

# Parse args FIRST (before any carla imports)
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

parser = argparse.ArgumentParser(
    description="OSC2 Scenario Runner for CARLA",
    formatter_class=argparse.RawDescriptionHelpFormatter,
    epilog="""
Examples:
  python3 run_scenario.py scenarios/vru_crossing.osc
  python3 run_scenario.py scenarios/vru_ped_occluded_parked_bus_baseline.osc --carla-version 0.9.16
  python3 run_scenario.py scenarios/vru_crossing.osc --carla-version 0.10.0 -v
    """,
)
parser.add_argument("scenario", help="Path to .osc scenario file")
parser.add_argument("--carla-version", choices=["0.10.0", "0.9.16"],
                    default=None, help="CARLA version (prompts if not specified)")
parser.add_argument("--host", default="localhost", help="CARLA host (default: localhost)")
parser.add_argument("--port", type=int, default=2000, help="CARLA port (default: 2000)")
parser.add_argument("--output-dir", default=os.path.join(script_dir, "..", "output"),
                    help="Output directory for CSV metrics")
parser.add_argument("--view", choices=["chase", "driver"], default=None,
                    help="Camera view: chase (3rd person) or driver (1st person)")
parser.add_argument("--v2x", action="store_true",
                    help="V2X mode: print instructions for starting ns-3 gateway, "
                         "v2x_bridge_node, and aeb_node_v2x instead of the baseline aeb_node. "
                         "(Scenario file is unchanged; the V2X stack runs alongside.)")
parser.add_argument("-v", "--verbose", action="store_true", help="Enable verbose logging")
args = parser.parse_args()

# Setup CARLA paths BEFORE importing engine modules
from carla_setup import setup_carla, prompt_version

if args.carla_version is None:
    args.carla_version = prompt_version()

config = setup_carla(args.carla_version)
print(f"Using CARLA {args.carla_version}")
print(f"  Launch with: {config['launch_script']}")

# NOW import engine modules
from grammar.parser import OSC2Parser
from engine.executor import ScenarioExecutor
from carla_backend.connection import CarlaConnection
from metrics.csv_writer import MetricsCSVWriter


def prompt_view():
    print("\nAvailable camera views:")
    print("  [1] Chase camera (3rd person, behind and above)")
    print("  [2] Driver view  (1st person, from driver's seat)")
    while True:
        try:
            choice = input("Select view [1-2]: ").strip()
            if choice == "1":
                return "chase"
            elif choice == "2":
                return "driver"
        except (ValueError, EOFError):
            pass
        print("  Please enter 1 or 2")


def main():
    log_level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(
        format="%(asctime)s [%(levelname)s] %(message)s",
        level=log_level,
        datefmt="%H:%M:%S",
    )

    # Stage 1: Parse scenario
    logging.info(f"Parsing scenario: {args.scenario}")
    osc2_parser = OSC2Parser()
    try:
        scenario_ir = osc2_parser.parse_file(args.scenario)
    except Exception as e:
        logging.error(f"Failed to parse scenario: {e}")
        sys.exit(1)

    ego_control = scenario_ir.get_param("ego_control", "python_api")
    logging.info(
        f"Parsed '{scenario_ir.name}' — "
        f"{len(scenario_ir.actor_instances)} instances, "
        f"ego_control={ego_control}"
    )

    if ego_control == "ros2":
        print("\n" + "=" * 60)
        if args.v2x:
            print("  ROS2 EGO CONTROL MODE — V2X ENABLED")
            print("  Make sure CARLA is started with --ros2")
            print("  After the ego actor ID prints below, start the V2X stack")
            print("  in ONE other terminal (bundles aeb_node_v2x + bridge + ns-3):")
            print("    ros2 launch carla_vru_demo aeb_v2x.launch.py \\")
            print("      ego_actor_id:=<ID>")
        else:
            print("  ROS2 EGO CONTROL MODE")
            print("  Make sure CARLA is started with --ros2")
            print("  Start AEB node after this script prints the ego actor ID")
        print("=" * 60 + "\n")

    # Stage 2: Connect to CARLA
    conn = CarlaConnection(host=args.host, port=args.port)
    try:
        conn.connect()
    except Exception as e:
        logging.error(f"Failed to connect to CARLA: {e}")
        sys.exit(1)

    # Select camera view
    view_mode = args.view if args.view else prompt_view()

    # Stage 3: Execute scenario (same executor handles both modes)
    executor = ScenarioExecutor(scenario_ir, conn, view_mode=view_mode)
    result = None
    try:
        result = executor.run()
    except Exception as e:
        logging.error(f"Scenario execution failed: {e}")
        import traceback
        traceback.print_exc()

    # Stage 4: Output results
    if result is not None:
        scenario_name = scenario_ir.name or "scenario"
        sim_delta = 1.0 / float(scenario_ir.get_param("sim_frequency", 60.0))

        MetricsCSVWriter.write_tick_log(
            executor.metrics.records, args.output_dir, scenario_name, delta=sim_delta
        )
        _, summary = MetricsCSVWriter.write_summary(
            executor.metrics.records, executor.metrics.collision_detected,
            args.output_dir, scenario_name, delta=sim_delta,
            near_miss=executor.metrics.near_miss,
            near_miss_distance=executor.metrics.near_miss_distance,
        )

        logging.info("=" * 60)
        logging.info("SCENARIO RESULTS")
        logging.info("=" * 60)
        logging.info(f"  TTC_min: {summary.get('ttc_min_s', 'N/A')}s | "
                     f"PET: {summary.get('pet_s', 'N/A')}s | "
                     f"Min dist: {summary.get('min_distance_m', 'N/A')}m | "
                     f"Collision: {summary.get('collision', 'N/A')}")
        logging.info(f"  Max decel: {summary.get('max_decel_ms2', 'N/A')} m/s² | "
                     f"Max jerk: {summary.get('max_jerk_ms3', 'N/A')} m/s³")
        logging.info(f"  Detection: speed={summary.get('speed_at_detection_kmh', 'N/A')} km/h, "
                     f"dist={summary.get('distance_at_detection_m', 'N/A')}m")
        logging.info(f"  Stopping dist: {summary.get('stopping_distance_m', 'N/A')}m | "
                     f"Brake reaction: {summary.get('brake_reaction_time_s', 'N/A')}s")
        logging.info(f"  Output: {args.output_dir}/")
        logging.info("=" * 60)
    else:
        logging.warning("No metrics collected (scenario may have failed)")


if __name__ == "__main__":
    main()
