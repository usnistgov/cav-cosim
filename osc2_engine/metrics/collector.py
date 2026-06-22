"""
Metrics Collector.

Records per-tick measurements from actors and detects collisions.
"""

import logging
from dataclasses import dataclass
from typing import List, Optional

import carla

from metrics.calculators import MetricsCalculators, MetricsResult


@dataclass
class TickRecord:
    """Per-tick state snapshot."""
    tick: int
    timestamp: float
    ego_x: float
    ego_y: float
    ego_z: float
    ego_speed_ms: float
    ego_speed_kmh: float
    ego_yaw: float
    ped_x: float
    ped_y: float
    ped_z: float
    ped_speed_ms: float
    ped_yaw: float
    distance: float


class MetricsCollector:
    """Collect per-tick measurements from actors."""

    def __init__(self, delta=0.05):
        self.records: List[TickRecord] = []
        self.collision_detected: bool = False
        self.collision_speed: float = 0.0
        self.collision_tick: int = -1
        self.near_miss: bool = False
        self.near_miss_distance: float = float('inf')
        self._collision_sensor = None
        self._tick_count: int = 0
        self._delta = delta

    def attach_collision_sensor(self, vehicle, conn):
        """Attach a collision sensor to the ego vehicle."""
        try:
            bp = conn.get_blueprint_library().find('sensor.other.collision')
            self._collision_sensor = conn.spawn_actor(
                bp, carla.Transform(), attach_to=vehicle
            )
            self._collision_sensor.listen(self._on_collision)
            logging.info("Collision sensor attached to ego vehicle")
        except Exception as e:
            logging.warning(f"Failed to attach collision sensor: {e}")

    def _on_collision(self, event):
        """Callback when collision occurs."""
        if not self.collision_detected:
            self.collision_detected = True
            self.collision_tick = self._tick_count
            impulse = event.normal_impulse
            self.collision_speed = (impulse.x**2 + impulse.y**2 + impulse.z**2) ** 0.5
            other = event.other_actor
            logging.warning(
                f"COLLISION detected at tick {self._tick_count} "
                f"with {other.type_id} (impulse: {self.collision_speed:.1f})"
            )

    def tick(self, actors):
        """Record state for all actors this tick."""
        self._tick_count += 1
        delta = self._delta

        try:
            ego_transform = actors.get_transform("ego")
            ego_speed = actors.get_speed("ego")

            ped_transform = actors.get_transform("ped")
            ped_speed = actors.get_speed("ped")

            distance = actors.distance_between("ego", "ped")

            record = TickRecord(
                tick=self._tick_count,
                timestamp=self._tick_count * delta,
                ego_x=ego_transform.location.x,
                ego_y=ego_transform.location.y,
                ego_z=ego_transform.location.z,
                ego_speed_ms=ego_speed,
                ego_speed_kmh=ego_speed * 3.6,
                ego_yaw=ego_transform.rotation.yaw,
                ped_x=ped_transform.location.x,
                ped_y=ped_transform.location.y,
                ped_z=ped_transform.location.z,
                ped_speed_ms=ped_speed,
                ped_yaw=ped_transform.rotation.yaw,
                distance=distance,
            )
            self.records.append(record)

        except (KeyError, AttributeError) as e:
            # Actor might not be spawned yet
            pass

    def finalize(self) -> MetricsResult:
        """Compute final metrics from collected records."""
        return MetricsCalculators.compute(self.records, self.collision_detected)

    def get_sensor_ids(self):
        """Return collision sensor actor IDs for cleanup."""
        if self._collision_sensor is not None:
            return [self._collision_sensor.id]
        return []
