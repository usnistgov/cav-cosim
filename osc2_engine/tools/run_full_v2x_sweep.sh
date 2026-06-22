#!/usr/bin/env bash
# Full V2X sweep: 9 speeds x 5 per-speed d_critical multipliers = 45 cells.
# Each speed gets its own distance grid: d_crit * [0.70, 0.85, 1.00, 1.15, 1.30].
# Scenario: vru_ped_occluded_parked_bus_realistic.osc (LiDAR-enabled RSU + t_fallback).
# Launches run_sweep.py once per speed; CSVs land in output/sweep_v2x/.

cd /home/hnh21/iotav/CARLA_0.10/osc2_engine
source /opt/ros/humble/setup.bash
source /home/hnh21/iotav/CARLA_0.10/ros_workspace/install/setup.bash

SCEN=/home/hnh21/iotav/CARLA_0.10/osc2_engine/scenarios/vru_ped_occluded_parked_bus_realistic.osc

# speed_kmh : "d1 d2 d3 d4 d5" (computed from formula in run_sweep.prompt_interactive)
declare -A GRID=(
  [20]="7.16 8.7  10.23 11.77 13.3"
  [25]="8.86 10.75 12.65 14.55 16.45"
  [30]="10.56 12.83 15.09 17.35 19.62"
  [35]="12.28 14.91 17.54 20.17 22.8"
  [40]="14 17 20 23 26"
  [45]="15.72 19.09 22.46 25.83 29.2"
  [50]="17.45 21.19 24.93 28.67 32.41"
  [55]="19.18 23.29 27.4 31.51 35.61"
  [60]="20.91 25.39 29.87 34.36 38.84"
)

for SPEED in 20 25 30 35 40 45 50 55 60; do
  DISTS=${GRID[$SPEED]}
  echo "================================================================"
  echo "  FULL V2X SWEEP — Speed $SPEED km/h, distances: $DISTS"
  echo "================================================================"
  python3 run_sweep.py --v2x-fusion --carla-version 0.9.16 \
    --speeds "$SPEED" --trigger-distances $DISTS --scenario "$SCEN" || \
    echo "  WARN: sweep at $SPEED km/h returned non-zero"
done

echo
echo "================================================================"
echo "  FULL V2X SWEEP COMPLETE"
echo "================================================================"
