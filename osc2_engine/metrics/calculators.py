"""
Metrics Calculators.

Computes TTC, PET, minimum distance, and collision metrics from tick records.
"""

from dataclasses import dataclass
from typing import List


@dataclass
class MetricsResult:
    """Summary of computed safety metrics."""
    ttc_min: float          # Minimum Time-to-Collision (seconds)
    pet: float              # Post-Encroachment Time (seconds)
    min_distance: float     # Minimum ego-pedestrian distance (meters)
    collision_detected: bool
    total_ticks: int
    total_time: float       # Total scenario time (seconds)

    def __str__(self):
        return (
            f"TTC_min: {self.ttc_min:.2f}s | "
            f"PET: {self.pet:.2f}s | "
            f"Min dist: {self.min_distance:.2f}m | "
            f"Collision: {self.collision_detected} | "
            f"Duration: {self.total_time:.1f}s"
        )


class MetricsCalculators:
    """Static methods for computing safety metrics from tick records."""

    @classmethod
    def _get_delta(cls, records):
        """Derive tick delta from records."""
        if len(records) >= 2:
            return records[1].timestamp - records[0].timestamp
        return 0.05

    @classmethod
    def compute(cls, records, collision_detected) -> MetricsResult:
        """Compute all metrics from tick records."""
        if not records:
            return MetricsResult(
                ttc_min=float('inf'),
                pet=float('inf'),
                min_distance=float('inf'),
                collision_detected=collision_detected,
                total_ticks=0,
                total_time=0.0,
            )

        return MetricsResult(
            ttc_min=cls.compute_min_ttc(records),
            pet=cls.compute_pet(records),
            min_distance=cls.compute_min_distance(records),
            collision_detected=collision_detected,
            total_ticks=len(records),
            total_time=records[-1].timestamp if records else 0.0,
        )

    @classmethod
    def compute_min_ttc(cls, records) -> float:
        """
        Time-to-Collision: distance / closing_speed.

        Computed each tick where the closing speed is positive (actors approaching).
        Returns the minimum TTC observed.
        """
        min_ttc = float('inf')

        for i in range(1, len(records)):
            # Closing speed = rate of decrease of distance
            delta = cls._get_delta(records)
            closing_speed = -(records[i].distance - records[i - 1].distance) / delta

            if closing_speed > 0.1:  # Only when actually approaching
                ttc = records[i].distance / closing_speed
                if ttc < min_ttc:
                    min_ttc = ttc

        return min_ttc

    @classmethod
    def compute_pet(cls, records, conflict_x=-43.7, conflict_radius=3.0) -> float:
        """
        Post-Encroachment Time: time difference between ego and pedestrian
        passing through the conflict zone (crosswalk center area).

        PET = |t_ego_at_conflict - t_ped_at_conflict|

        A lower PET indicates a more dangerous encounter.
        """
        t_ego = None
        t_ped = None

        for r in records:
            if t_ego is None and abs(r.ego_x - conflict_x) < conflict_radius:
                t_ego = r.timestamp
            if t_ped is None and abs(r.ped_x - conflict_x) < conflict_radius:
                t_ped = r.timestamp

        if t_ego is not None and t_ped is not None:
            return abs(t_ego - t_ped)

        return float('inf')

    @classmethod
    def compute_min_distance(cls, records) -> float:
        """Minimum Euclidean distance between ego and pedestrian."""
        if not records:
            return float('inf')
        return min(r.distance for r in records)
