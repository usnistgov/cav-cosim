"""
CSV Writer.

Outputs per-tick data and summary metrics to CSV files.
Includes derived metrics: acceleration, jerk, TTC, PET, detection info.
"""

import csv
import logging
import math
import os
from datetime import datetime


class MetricsCSVWriter:
    """Write metrics data to CSV files with derived fields."""

    @staticmethod
    def _compute_derived(records, delta):
        """Compute acceleration, jerk, TTC per tick from raw records."""
        rows = []
        W = 6  # smoothing half-window (6 ticks = 0.1s at 60Hz)

        for i, r in enumerate(records):
            lo = max(0, i - W)
            hi = min(len(records) - 1, i + W)
            dt = (hi - lo) * delta
            accel = (records[hi].ego_speed_ms - records[lo].ego_speed_ms) / dt if dt > 0 and hi != lo else 0.0

            if i >= W and i + W < len(records):
                lo_a_lo = max(0, (i - W) - W)
                lo_a_hi = min(len(records) - 1, (i - W) + W)
                lo_dt = (lo_a_hi - lo_a_lo) * delta
                prev_accel = (records[lo_a_hi].ego_speed_ms - records[lo_a_lo].ego_speed_ms) / lo_dt if lo_dt > 0 else 0.0
                jerk = (accel - prev_accel) / (W * delta) if W * delta > 0 else 0.0
            else:
                jerk = 0.0

            lo_c = max(0, i - W)
            closing_speed = -(r.distance - records[lo_c].distance) / ((i - lo_c) * delta) if i > lo_c else 0.0
            ttc = r.distance / closing_speed if closing_speed > 0.1 else float('inf')

            rows.append({
                "tick": r.tick, "timestamp": r.timestamp,
                "ego_x": r.ego_x, "ego_y": r.ego_y, "ego_z": r.ego_z,
                "ego_speed_ms": r.ego_speed_ms, "ego_speed_kmh": r.ego_speed_kmh,
                "ego_yaw": r.ego_yaw, "ego_accel_ms2": accel, "ego_jerk_ms3": jerk,
                "ped_x": r.ped_x, "ped_y": r.ped_y, "ped_z": r.ped_z,
                "ped_speed_ms": r.ped_speed_ms, "ped_yaw": r.ped_yaw,
                "distance": r.distance, "closing_speed_ms": closing_speed, "ttc": ttc,
            })
        return rows

    @staticmethod
    def _compute_summary(records, derived_rows, collision_detected, delta,
                         ego_lane_x=-272.0, ped_crossing_y=-12.0, conflict_radius=3.0,
                         near_miss=False, near_miss_distance=None):
        """Compute all summary metrics."""
        if not records or not derived_rows:
            return {}

        min_distance = min(r.distance for r in records)
        min_ttc = min((d["ttc"] for d in derived_rows if d["ttc"] != float('inf')), default=float('inf'))
        accels = [d["ego_accel_ms2"] for d in derived_rows]
        jerks = [d["ego_jerk_ms3"] for d in derived_rows]
        max_decel = min(accels) if accels else 0.0
        max_jerk = max(abs(j) for j in jerks) if jerks else 0.0

        # Detection: first moment ego brakes hard while driving
        detection_tick = speed_at_detection = distance_at_detection = None
        for d in derived_rows:
            if d["ego_accel_ms2"] < -2.0 and d["ego_speed_ms"] > 3.0:
                detection_tick = d["tick"]
                speed_at_detection = d["ego_speed_kmh"]
                distance_at_detection = d["distance"]
                break

        # Collision speed
        collision_speed = None
        if collision_detected:
            min_dist_idx = min(range(len(records)), key=lambda i: records[i].distance)
            collision_speed = records[min_dist_idx].ego_speed_kmh

        # Stopping distance
        stopping_distance = None
        if detection_tick is not None:
            det_idx = next((i for i, d in enumerate(derived_rows) if d["tick"] == detection_tick), None)
            if det_idx is not None:
                det_y = records[det_idx].ego_y
                for j in range(det_idx, len(records)):
                    if records[j].ego_speed_ms < 0.5:
                        stopping_distance = abs(records[j].ego_y - det_y)
                        break

        # Brake reaction time
        ped_in_lane_tick = None
        for d in derived_rows:
            if abs(d["ped_x"] - ego_lane_x) < 1.5 and d["ped_speed_ms"] > 0.01:
                ped_in_lane_tick = d["tick"]
                break
        brake_reaction_time = None
        if ped_in_lane_tick is not None and detection_tick is not None:
            brake_reaction_time = max(0.0, (detection_tick - ped_in_lane_tick) * delta)

        # PET
        t_ego_conflict = t_ped_conflict = None
        for d in derived_rows:
            if t_ego_conflict is None and abs(d["ego_y"] - ped_crossing_y) < conflict_radius:
                t_ego_conflict = d["timestamp"]
            if t_ped_conflict is None and abs(d["ped_x"] - ego_lane_x) < 1.5:
                t_ped_conflict = d["timestamp"]
        pet = abs(t_ego_conflict - t_ped_conflict) if (t_ego_conflict and t_ped_conflict) else float('inf')

        # Path-blocking encroachment: signed distance from ego front bumper
        # to the "ped clearance line" (ped's walking path minus a small
        # margin) at the ego's slowest moment after AEB engaged.
        #   > 0 = ego came to rest AT the ped's level — blocking the path,
        #         must creep forward to clear → NEAR_MISS by user definition.
        #   ≤ 0 = ego came to rest SHORT of the ped's path with room for the
        #         ped to walk through → SAFE.
        # Using min-ego-speed (not first-full-stop) so the CREEP case where
        # the ego only briefly slows before creeping is still captured.
        # Ego drives northbound (+y) so front bumper is ego_y + half_length.
        EGO_HALF_LENGTH = 2.25  # MKZ
        EGO_HALF_WIDTH = 1.0    # MKZ
        PED_PATH_Y = -12.0      # ped walks along y = -12 in this scenario
        PED_MARGIN_M = 0.30     # clearance needed for the ped to pass safely
        PED_BLOCKING_LINE_Y = PED_PATH_Y - PED_MARGIN_M  # = -12.30
        path_blocking = None
        if detection_tick is not None:
            det_idx = next((i for i, d in enumerate(derived_rows) if d["tick"] == detection_tick), None)
            if det_idx is not None:
                slowest_idx = min(range(det_idx, len(records)),
                                  key=lambda i: records[i].ego_speed_ms)
                ego_front_y = records[slowest_idx].ego_y + EGO_HALF_LENGTH
                path_blocking = ego_front_y - PED_BLOCKING_LINE_Y

        # Lateral clearance at min-distance tick: perpendicular gap between
        # the pedestrian and the ego corridor edge, in the ego body frame.
        # Positive = ped outside the corridor; negative = ped inside corridor
        # (the AEB barely missed). Captures "did the ped have room to pass?".
        lateral_clearance = None
        if records:
            min_idx = min(range(len(records)), key=lambda i: records[i].distance)
            r = records[min_idx]
            yaw_rad = math.radians(r.ego_yaw)
            dx = r.ped_x - r.ego_x
            dy = r.ped_y - r.ego_y
            lateral = -math.sin(yaw_rad) * dx + math.cos(yaw_rad) * dy
            lateral_clearance = abs(lateral) - EGO_HALF_WIDTH

        return {
            "ttc_min_s": min_ttc, "pet_s": pet, "min_distance_m": min_distance,
            "collision": collision_detected, "near_miss": near_miss,
            "near_miss_distance_m": near_miss_distance,
            "path_blocking_m": path_blocking,
            "lateral_clearance_min_m": lateral_clearance,
            "collision_speed_kmh": collision_speed,
            "max_decel_ms2": max_decel, "max_jerk_ms3": max_jerk,
            "speed_at_detection_kmh": speed_at_detection,
            "distance_at_detection_m": distance_at_detection,
            "stopping_distance_m": stopping_distance,
            "brake_reaction_time_s": brake_reaction_time,
            "total_ticks": len(records), "total_time_s": records[-1].timestamp,
        }

    @staticmethod
    def write_tick_log(records, output_dir, scenario_name="scenario", delta=1/60.0):
        """Write per-tick CSV with all derived metrics."""
        os.makedirs(output_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = os.path.join(output_dir, f"{scenario_name}_ticks_{timestamp}.csv")

        derived = MetricsCSVWriter._compute_derived(records, delta)
        fieldnames = [
            "tick", "timestamp", "ego_x", "ego_y", "ego_z",
            "ego_speed_ms", "ego_speed_kmh", "ego_yaw",
            "ego_accel_ms2", "ego_jerk_ms3",
            "ped_x", "ped_y", "ped_z", "ped_speed_ms", "ped_yaw",
            "distance", "closing_speed_ms", "ttc",
        ]

        with open(filepath, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for d in derived:
                row = {}
                for k in fieldnames:
                    v = d[k]
                    if isinstance(v, float):
                        row[k] = "inf" if v == float('inf') else f"{v:.4f}"
                    else:
                        row[k] = v
                writer.writerow(row)

        logging.info(f"Tick log written: {filepath} ({len(records)} records)")
        return filepath

    @staticmethod
    def write_summary(records, collision_detected, output_dir,
                      scenario_name="scenario", delta=1/60.0,
                      near_miss=False, near_miss_distance=None):
        """Write summary CSV with all metrics. Returns (filepath, summary_dict)."""
        os.makedirs(output_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = os.path.join(output_dir, f"{scenario_name}_summary_{timestamp}.csv")

        derived = MetricsCSVWriter._compute_derived(records, delta)
        summary = MetricsCSVWriter._compute_summary(
            records, derived, collision_detected, delta,
            near_miss=near_miss, near_miss_distance=near_miss_distance,
        )

        fieldnames = list(summary.keys())
        with open(filepath, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            row = {}
            for k, v in summary.items():
                if v is None:
                    row[k] = "N/A"
                elif isinstance(v, float):
                    row[k] = "inf" if v == float('inf') else f"{v:.4f}"
                elif isinstance(v, bool):
                    row[k] = str(v)
                else:
                    row[k] = v
            writer.writerow(row)

        logging.info(f"Summary written: {filepath}")
        return filepath, summary
