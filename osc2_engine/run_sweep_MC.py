#!/usr/bin/env python3
"""
Monte Carlo variant of run_sweep.py.

Repeats each (speed, trigger-distance) cell N times with random perturbations
on ped walking speed and ped spawn lateral offset. Produces a probabilistic
outcome distribution per cell, which is necessary for boundary cells where a
single deterministic run is not statistically meaningful.

Output goes to a separate folder (default: output/sweep_MC/) so it never mixes
with the deterministic sweep.

Perturbations (default):
  - ped walking speed: Gaussian N(1.4, 0.25) m/s, clipped to [0.8, 2.0]
  - ped spawn x offset: Uniform U(-0.5, +0.5) m around nominal -275.0
"""

import argparse
import csv
import math
import os
import random
import re
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

from run_sweep import (
    run_scenario_with_aeb,
    parse_summary_csv,
    classify_outcome,
)

# --- Perturbation distributions (tweak here if you want different noise) ---
PED_SPEED_MEAN = 1.4
PED_SPEED_STD = 0.25
PED_SPEED_MIN = 0.8
PED_SPEED_MAX = 2.0
PED_X_OFFSET_RANGE = 0.5  # uniform +/- this value around nominal

# Nominal values in the master scenario file (for patching)
NOMINAL_PED_X = -275.0

# d_crit geometry for default distance suggestion
L_CROSS = 2.5
V_P_NOMINAL = 1.4
D_LAT = 2.5


def compute_d_crit(v_kmh):
    v_e = v_kmh / 3.6
    d_long = (L_CROSS / V_P_NOMINAL) * v_e
    return math.sqrt(d_long ** 2 + D_LAT ** 2)


def sample_perturbation():
    """Draw one MC sample: (ped_speed_mps, ped_x_offset_m)."""
    s = max(PED_SPEED_MIN, min(PED_SPEED_MAX,
                               random.gauss(PED_SPEED_MEAN, PED_SPEED_STD)))
    x = random.uniform(-PED_X_OFFSET_RANGE, PED_X_OFFSET_RANGE)
    return s, x


def prepare_scenario_mc(base_scenario, trigger_distance, output_dir,
                        ped_speed, ped_x_offset):
    """Write a perturbed copy of the scenario into output_dir/scenario.osc."""
    text = Path(base_scenario).read_text()

    # Trigger distance — rewrite the `d_trigger: length = Xm` parameter default.
    text, n = re.subn(
        r'(d_trigger\s*:\s*length\s*=\s*)\d+(\.\d+)?m',
        lambda m: f'{m.group(1)}{trigger_distance:g}m',
        text,
    )
    if n == 0:
        raise ValueError(
            f"Scenario {base_scenario} has no `d_trigger: length = Xm` parameter — "
            "cannot sweep trigger distance."
        )

    # Ped walking speed — the scenario has a single `speed(Xmps)` (ped.walk)
    text = re.sub(
        r'speed\(\d+(\.\d+)?mps\)',
        f'speed({ped_speed:.3f}mps)',
        text,
    )

    # Ped spawn x — anchor on the block header so bus/parked positions aren't
    # touched. Matches "ped.assign_position() with:\n ... position(x: <num>"
    new_x = NOMINAL_PED_X + ped_x_offset
    text = re.sub(
        r'(ped\.assign_position\(\)\s*with:\s*\n\s*position\(x:\s*)-?\d+(\.\d+)?',
        lambda m: f'{m.group(1)}{new_x:.3f}',
        text,
    )

    os.makedirs(output_dir, exist_ok=True)
    out_path = os.path.join(output_dir, "scenario.osc")
    Path(out_path).write_text(text)
    return out_path


def prompt_interactive(v2x=False, fusion_mode="both"):
    """Interactive prompts. AEB mode comes from CLI flags (--v2x, --v2x-fusion)."""
    print("=" * 60)
    print("  VRU OCCLUSION — MONTE CARLO SWEEP — Configuration")
    print("=" * 60)

    print("\nCARLA version:")
    print("  [1] 0.9.16 (UE4)")
    print("  [2] 0.10.0 (UE5)")
    while True:
        c = input("Select [1-2] (default: 1): ").strip()
        if c in ("", "1"):
            carla_version = "0.9.16"; break
        if c == "2":
            carla_version = "0.10.0"; break

    default_speeds = "35 40 45"
    print(f"\nBoundary-band speeds to test (km/h):")
    print(f"  Enter values separated by spaces, e.g.: 30 35 40")
    print(f"  Or press Enter for default: {default_speeds}")
    s = input("Speeds: ").strip()
    speeds = [float(x) for x in (s if s else default_speeds).split()]

    # Default distances: d_crit for each selected speed (1.00 × multiplier)
    default_dists = sorted({round(compute_d_crit(v), 2) for v in speeds})
    default_distances = " ".join(f"{d:g}" for d in default_dists)

    print(f"\nTrigger distances in meters:")
    print(f"  Enter values separated by spaces, e.g.: 15 20")
    print(f"  Default = d_crit for each selected speed (1.00 × multiplier):")
    for v_kmh in speeds:
        dcrit = compute_d_crit(v_kmh)
        print(f"    {v_kmh:>5g} km/h  d_crit = {dcrit:.2f} m")
    print(f"  Or press Enter for default: {default_distances}")
    t = input("Distances: ").strip()
    trigger_distances = [float(x) for x in t.split()] if t else list(default_dists)

    print(f"\nMonte Carlo repeats per cell:")
    print(f"  Enter an integer (default: 10)")
    n = input("Repeats: ").strip()
    mc_n = int(n) if n else 10

    print(f"\nRandom seed (optional, for reproducibility):")
    print(f"  Enter an integer or press Enter for system-random")
    seed_input = input("Seed: ").strip()
    seed = int(seed_input) if seed_input else None

    # Camera view: Foxglove shows both chase and driver panels, so we no
    # longer prompt. Match run_sweep.py's hardcoded chase default.
    view = "chase"

    scenarios = sorted(Path(os.path.join(script_dir, "scenarios")).glob("*.osc"))
    print("\nAvailable scenarios:")
    for i, sc in enumerate(scenarios, 1):
        marker = " (default)" if "parked_bus_baseline" in sc.name else ""
        print(f"  [{i}] {sc.name}{marker}")
    default_idx = next((i for i, s in enumerate(scenarios, 1)
                        if "parked_bus_baseline" in s.name), 1)
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
    out_subdir = "sweep_MC_v2x" if v2x else "sweep_MC"
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
        monte_carlo=mc_n,
        seed=seed,
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
    parser = argparse.ArgumentParser(description="VRU Occlusion Monte Carlo Sweep")
    parser.add_argument("--speeds", nargs="+", type=float, default=None)
    parser.add_argument("--trigger-distances", nargs="+", type=float, default=None)
    parser.add_argument("--monte-carlo", type=int, default=None,
                        help="MC repeats per cell")
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--scenario", default=None)
    parser.add_argument("--carla-version", default=None,
                        choices=["0.9.16", "0.10.0"])
    parser.add_argument("--view", default=None, choices=["chase", "driver"])
    parser.add_argument("--output-dir", default=None,
                        help="Output dir (default: output/sweep_MC, or sweep_MC_v2x with --v2x)")
    parser.add_argument("--no-save", action="store_true",
                        help="Run in a temp dir and discard results (for demos)")
    v2x_group = parser.add_mutually_exclusive_group()
    v2x_group.add_argument("--v2x", action="store_true",
                           help="V2X-only AEB: V2X stack ON, only CAM messages trigger braking "
                                "(LiDAR ignored). ns-3 is launched automatically per cell. "
                                "Output goes to output/sweep_MC_v2x/.")
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

    no_explicit = (args.speeds is None and args.trigger_distances is None
                   and args.monte_carlo is None and args.carla_version is None)
    if no_explicit:
        return prompt_interactive(v2x=args.v2x, fusion_mode=args.fusion_mode)

    if args.speeds is None:
        args.speeds = [35, 40, 45]
    if args.trigger_distances is None:
        args.trigger_distances = [round(compute_d_crit(v), 2) for v in args.speeds]
    if args.monte_carlo is None:
        args.monte_carlo = 10
    if args.scenario is None:
        args.scenario = os.path.join(
            script_dir, "scenarios", "vru_ped_occluded_parked_bus_baseline.osc")
    if args.carla_version is None:
        args.carla_version = "0.9.16"
    if args.view is None:
        args.view = "chase"
    if args.output_dir is None:
        sub = "sweep_MC_v2x" if args.v2x else "sweep_MC"
        args.output_dir = os.path.join(script_dir, "..", "output", sub)
    return args


def main():
    args = parse_args()

    if args.seed is not None:
        random.seed(args.seed)
        print(f"  Random seed set to {args.seed}")

    save_results = getattr(args, "save_results", True)
    if save_results:
        os.makedirs(args.output_dir, exist_ok=True)
    else:
        import tempfile
        args.output_dir = tempfile.mkdtemp(prefix="sweep_MC_demo_")

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    speeds_tag = "_".join(f"{s:g}kmh" for s in args.speeds)
    sweep_csv = os.path.join(
        args.output_dir,
        f"sweep_MC_results_{speeds_tag}_{timestamp}.csv",
    )

    total_runs = len(args.trigger_distances) * len(args.speeds) * args.monte_carlo
    v2x_on = getattr(args, "v2x", False)
    print("=" * 70)
    if v2x_on:
        print("  VRU OCCLUSION MONTE CARLO SWEEP — V2X ENABLED")
        print(f"  Fusion mode: {getattr(args, 'fusion_mode', 'both')}")
    else:
        print("  VRU OCCLUSION MONTE CARLO SWEEP")
    print(f"  Speeds: {args.speeds} km/h")
    print(f"  Trigger distances: {args.trigger_distances} m")
    print(f"  MC repeats per cell: {args.monte_carlo}")
    print(f"  Total runs: {total_runs}")
    print(f"  Perturbations: ped_speed ~ N({PED_SPEED_MEAN}, {PED_SPEED_STD}) "
          f"in [{PED_SPEED_MIN}, {PED_SPEED_MAX}] m/s, "
          f"ped_x_offset ~ U(±{PED_X_OFFSET_RANGE}) m")
    print(f"  Scenario: {args.scenario}")
    if save_results:
        print(f"  Output: {sweep_csv}")
    else:
        print(f"  Output: [demo mode — discarded when done, temp dir {args.output_dir}]")
    print("=" * 70)

    results = []
    run_num = 0

    for dist in args.trigger_distances:
        for speed in args.speeds:
            print(f"\n{'=' * 70}")
            print(f"  CELL: {speed:g} km/h @ {dist:g}m trigger  "
                  f"({args.monte_carlo} MC runs)")
            print(f"{'=' * 70}")

            for mc_i in range(args.monte_carlo):
                run_num += 1
                ped_speed, ped_x_off = sample_perturbation()

                print(f"\n{'─' * 70}")
                print(f"  Run {run_num}/{total_runs}  "
                      f"(MC {mc_i+1}/{args.monte_carlo}): "
                      f"{speed:g} km/h @ {dist:g}m  "
                      f"ped_v={ped_speed:.3f}mps  ped_x_off={ped_x_off:+.3f}m")
                print(f"{'─' * 70}")

                run_dir = os.path.join(
                    args.output_dir,
                    f"run_{dist:g}m_{speed:g}kmh_mc{mc_i:02d}",
                )
                os.makedirs(run_dir, exist_ok=True)

                scenario_path = prepare_scenario_mc(
                    args.scenario, dist, run_dir, ped_speed, ped_x_off,
                )

                args_copy = argparse.Namespace(**vars(args))
                args_copy.scenario = scenario_path

                row = {
                    "target_speed_kmh": speed,
                    "trigger_distance_m": dist,
                    "mc_iter": mc_i,
                    "ped_speed_mps": round(ped_speed, 4),
                    "ped_x_offset_m": round(ped_x_off, 4),
                }

                try:
                    scen_label = os.path.splitext(
                        os.path.basename(args.scenario))[0]
                    stdout, rc = run_scenario_with_aeb(
                        args_copy, speed, run_dir,
                        trigger_dist=dist, scenario_label=scen_label,
                    )
                    with open(os.path.join(run_dir, "scenario_log.txt"), "w") as f:
                        f.write(stdout)
                    if rc != 0:
                        print(f"  WARNING: scenario exited with code {rc}")

                    summary = parse_summary_csv(run_dir)
                    if summary:
                        row.update(summary)
                    else:
                        row.update({k: None for k in [
                            "ttc_min_s", "pet_s", "min_distance_m",
                            "collision", "near_miss", "near_miss_distance_m",
                            "path_blocking_m", "lateral_clearance_min_m",
                            "collision_speed_kmh", "max_decel_ms2", "max_jerk_ms3",
                            "speed_at_detection_kmh", "distance_at_detection_m",
                            "stopping_distance_m", "brake_reaction_time_s",
                            "total_time_s",
                        ]})
                    row["outcome"] = (classify_outcome(row)
                                      if row.get("collision") is not None else None)

                    # "Result:" line — identical format to run_sweep.py so
                    # the on-screen output of MC iterations reads the same
                    # as deterministic cells.
                    if row.get("collision") is not None:
                        outcome_str = row.get("outcome") or "UNKNOWN"
                        md = (f"{row['min_distance_m']:.1f}"
                              if row.get('min_distance_m') is not None else "N/A")
                        ttc = (f"{row['ttc_min_s']:.2f}"
                               if row.get('ttc_min_s') is not None else "N/A")
                        lat = row.get("lateral_clearance_min_m")
                        lat_s = f"{lat:+.1f}m" if lat is not None else "N/A"
                        print(f"  Result: {outcome_str}, min_dist={md}m, "
                              f"TTC={ttc}s, lat_clr={lat_s}")
                    else:
                        print(f"  Result: FAILED (no metrics)")

                except subprocess.TimeoutExpired:
                    print(f"  ERROR: scenario timed out")
                except Exception as e:
                    print(f"  ERROR: {e}")
                finally:
                    # Same 2-sec inter-cell drain as run_sweep.py — lets the
                    # V2X stack tear down (port 8100 free, RSU actors gone)
                    # before the next cell spawns. Without this, consecutive
                    # MC iterations race and the next ego can start late or
                    # see a stale RSU position.
                    time.sleep(2)

                results.append(row)

    # Write raw results CSV (one row per MC run).
    # Schema MUST match run_sweep.py's column order EXACTLY so the same
    # plot scripts work on both CSVs. The MC-specific perturbation columns
    # are appended at the END — plotters that don't know about them just
    # ignore the extras.
    fieldnames = [
        # — identical schema to run_sweep.py —
        "target_speed_kmh", "trigger_distance_m", "outcome",
        "ttc_min_s", "pet_s", "min_distance_m", "collision",
        "near_miss", "near_miss_distance_m",
        "path_blocking_m", "lateral_clearance_min_m",
        "collision_speed_kmh", "max_decel_ms2", "max_jerk_ms3",
        "speed_at_detection_kmh", "distance_at_detection_m",
        "stopping_distance_m", "brake_reaction_time_s", "total_time_s",
        # — MC-specific extras (appended so the prefix stays a non-MC schema) —
        "mc_iter", "ped_speed_mps", "ped_x_offset_m",
    ]
    with open(sweep_csv, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction='ignore')
        writer.writeheader()
        for r in results:
            out = dict(r)
            for k in ("collision", "near_miss"):
                if isinstance(out.get(k), bool):
                    out[k] = str(out[k])
            writer.writerow(out)

    # --- Per-cell probability summary ----------------------------------
    print(f"\n{'=' * 70}")
    print(f"  MONTE CARLO — per-cell summary")
    print(f"{'=' * 70}")
    hdr = (f"  {'Speed':>6}  {'Trig':>6}  {'N':>3}  "
           f"{'P(coll)':>7}  {'P(nm)':>7}  {'P(safe)':>7}  "
           f"{'Min d̄':>7}  {'Coll v̄':>8}")
    print(hdr)
    print(f"  {'─' * (len(hdr) - 2)}")

    def mean(xs):
        xs = [x for x in xs if x is not None]
        return (sum(xs) / len(xs)) if xs else float('nan')

    for dist in args.trigger_distances:
        for speed in args.speeds:
            cell = [r for r in results
                    if r["target_speed_kmh"] == speed
                    and r["trigger_distance_m"] == dist
                    and r.get("collision") is not None]
            n = len(cell)
            if n == 0:
                print(f"  {speed:>6g}  {dist:>6g}  {0:>3}  "
                      f"{'—':>7}  {'—':>7}  {'—':>7}  {'—':>7}  {'—':>8}")
                continue
            n_coll = sum(1 for r in cell if r.get("collision") is True)
            n_nm = sum(1 for r in cell
                       if r.get("collision") is False and r.get("near_miss") is True)
            n_safe = sum(1 for r in cell
                         if r.get("collision") is False and r.get("near_miss") is False)
            md = mean([r.get("min_distance_m") for r in cell])
            cs = mean([r.get("collision_speed_kmh") for r in cell])
            cs_str = f"{cs:>8.2f}" if not math.isnan(cs) else f"{'—':>8}"
            print(f"  {speed:>6g}  {dist:>6g}  {n:>3}  "
                  f"{n_coll/n:>7.2f}  {n_nm/n:>7.2f}  {n_safe/n:>7.2f}  "
                  f"{md:>7.2f}  {cs_str}")

    if save_results:
        print(f"\n  Results saved: {sweep_csv}")
        print(f"  Per-run data: {args.output_dir}/")
    else:
        import shutil
        shutil.rmtree(args.output_dir, ignore_errors=True)
        print(f"\n  Demo mode — output directory discarded (nothing saved)")


if __name__ == "__main__":
    main()
