"""
Actor Manager.

Manages CARLA actor lifecycle: registration, spawning, state queries, destruction.
"""

import logging
import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import carla


@dataclass
class ManagedActor:
    """Tracks an actor's OSC2 declaration and CARLA instance."""
    name: str
    parent_type: str
    fields: Dict[str, Any]
    carla_actor: Optional[Any] = None
    controller: Optional[Any] = None  # BehaviorAgent for vehicles


class ActorManager:
    """Manages CARLA actor lifecycle, keyed by OSC2 instance names."""

    def __init__(self, conn):
        self.conn = conn
        self._actors: Dict[str, ManagedActor] = {}
        self._paused_walkers: set = set()

    def pause_walker(self, instance_name):
        """Mark a walker as paused — WalkHandler will apply speed=0 and stop advancing."""
        self._paused_walkers.add(instance_name)

    def unpause_walker(self, instance_name):
        """Resume a paused walker — WalkHandler will continue toward its target."""
        self._paused_walkers.discard(instance_name)

    def is_walker_paused(self, instance_name):
        return instance_name in self._paused_walkers

    def register(self, instance_name, parent_type, fields):
        """Register an actor type without spawning."""
        self._actors[instance_name] = ManagedActor(
            name=instance_name,
            parent_type=parent_type,
            fields=fields,
        )
        logging.debug(f"Registered actor '{instance_name}' (type: {parent_type})")

    def spawn(self, instance_name, x, y, z=0.5, yaw=0.0, project_to_road=False):
        """Spawn a registered actor in the CARLA world."""
        managed = self._actors[instance_name]
        model = managed.fields.get("model", "")
        bp_library = self.conn.get_blueprint_library()

        if managed.parent_type in ("vehicle",):
            return self._spawn_vehicle(managed, bp_library, x, y, z, yaw, project_to_road)
        elif managed.parent_type in ("person", "pedestrian"):
            return self._spawn_pedestrian(managed, bp_library, x, y, z, yaw)
        else:
            raise ValueError(f"Unknown parent type: {managed.parent_type}")

    def _spawn_vehicle(self, managed, bp_library, x, y, z, yaw, project_to_road):
        """Spawn a vehicle actor."""
        model = managed.fields.get("model", "vehicle.lincoln.mkz")

        # Try primary model, then fallbacks
        models_to_try = [model]
        fallback = managed.fields.get("fallback_models", "")
        if fallback:
            models_to_try.extend(fallback.split(","))

        vehicle_bp = None
        for m in models_to_try:
            m = m.strip()
            # Try exact match first, then wildcard (0.9.16 needs explicit wildcard)
            results = bp_library.filter(m)
            if len(results) == 0:
                results = bp_library.filter(m + "*")
            if len(results) > 0:
                vehicle_bp = results[0]
                if vehicle_bp.id != m:
                    logging.info(f"Blueprint '{m}' matched as '{vehicle_bp.id}'")
                break

        if vehicle_bp is None:
            raise RuntimeError(f"No blueprint found for {managed.name}. Tried: {models_to_try}")

        # Set ROS2 attributes if provided
        role_name = managed.fields.get("role_name", managed.name)
        ros_name = managed.fields.get("ros_name", managed.name)
        vehicle_bp.set_attribute("role_name", str(role_name))
        if vehicle_bp.has_attribute("ros_name"):
            vehicle_bp.set_attribute("ros_name", str(ros_name))

        # Determine spawn transform
        location = carla.Location(x=x, y=y, z=z)
        if project_to_road:
            waypoint = self.conn.get_waypoint(location, project_to_road=True)
            if waypoint:
                spawn_point = waypoint.transform
                spawn_point.location.z += 0.5
            else:
                spawn_point = carla.Transform(location, carla.Rotation(yaw=yaw))
        else:
            # Always project to road for a valid spawn, but preserve the requested yaw
            waypoint = self.conn.get_waypoint(location, project_to_road=True)
            if waypoint:
                spawn_point = waypoint.transform
                spawn_point.location.z += 0.5
                if yaw != 0.0:
                    spawn_point.rotation.yaw = yaw
            else:
                spawn_point = carla.Transform(
                    carla.Location(x=x, y=y, z=max(z, 0.5) + 0.5),
                    carla.Rotation(yaw=yaw),
                )

        actor = self.conn.try_spawn_actor(vehicle_bp, spawn_point)
        if actor is None:
            raise RuntimeError(f"Failed to spawn vehicle '{managed.name}' at ({x}, {y}, {z})")

        managed.carla_actor = actor
        logging.info(
            f"Spawned vehicle '{managed.name}' ({model}) at "
            f"({spawn_point.location.x:.1f}, {spawn_point.location.y:.1f}, {spawn_point.location.z:.1f})"
        )
        return actor

    def _spawn_pedestrian(self, managed, bp_library, x, y, z, yaw):
        """Spawn a pedestrian actor."""
        model = managed.fields.get("model", "walker.pedestrian.0020")
        results = bp_library.filter(model)
        if len(results) == 0:
            results = bp_library.filter(model + "*")
        if len(results) == 0:
            raise RuntimeError(f"No pedestrian blueprint found for '{model}'")
        ped_bp = results[0]

        if ped_bp.has_attribute("is_invincible"):
            invincible = managed.fields.get("is_invincible", False)
            ped_bp.set_attribute("is_invincible", str(invincible).lower())

        transform = carla.Transform(
            carla.Location(x=x, y=y, z=z),
            carla.Rotation(yaw=yaw),
        )

        actor = self.conn.try_spawn_actor(ped_bp, transform)
        if actor is None:
            raise RuntimeError(f"Failed to spawn pedestrian '{managed.name}' at ({x}, {y}, {z})")

        managed.carla_actor = actor
        logging.info(f"Spawned pedestrian '{managed.name}' at ({x:.1f}, {y:.1f}, {z:.1f})")
        return actor

    def get(self, instance_name):
        """Get the ManagedActor by name."""
        return self._actors[instance_name]

    def get_carla_actor(self, instance_name):
        """Get the underlying CARLA actor."""
        return self._actors[instance_name].carla_actor

    def get_location(self, instance_name):
        """Get actor's current location."""
        actor = self._actors[instance_name].carla_actor
        return actor.get_location()

    def get_transform(self, instance_name):
        """Get actor's current transform."""
        actor = self._actors[instance_name].carla_actor
        return actor.get_transform()

    def get_velocity(self, instance_name):
        """Get actor's current velocity vector."""
        actor = self._actors[instance_name].carla_actor
        return actor.get_velocity()

    def get_speed(self, instance_name):
        """Get actor's current speed in m/s."""
        v = self.get_velocity(instance_name)
        return math.sqrt(v.x**2 + v.y**2 + v.z**2)

    def get_speed_kmh(self, instance_name):
        """Get actor's current speed in km/h."""
        return self.get_speed(instance_name) * 3.6

    def distance_between(self, name_a, name_b):
        """Euclidean distance between two actors."""
        loc_a = self.get_location(name_a)
        loc_b = self.get_location(name_b)
        return loc_a.distance(loc_b)

    def distance_to_point(self, instance_name, x, y):
        """Distance from an actor to a point (2D)."""
        loc = self.get_location(instance_name)
        dx = loc.x - x
        dy = loc.y - y
        return math.sqrt(dx**2 + dy**2)

    def set_controller(self, instance_name, controller):
        """Store a controller (e.g., BehaviorAgent) for an actor."""
        self._actors[instance_name].controller = controller

    def get_controller(self, instance_name):
        """Get the stored controller for an actor."""
        return self._actors[instance_name].controller

    def apply_vehicle_control(self, instance_name, throttle=0.0, steer=0.0,
                               brake=0.0, hand_brake=False):
        """Apply VehicleControl to a vehicle."""
        actor = self._actors[instance_name].carla_actor
        control = carla.VehicleControl(
            throttle=throttle, steer=steer,
            brake=brake, hand_brake=hand_brake,
        )
        actor.apply_control(control)

    def apply_walker_control(self, instance_name, direction, speed):
        """Apply WalkerControl to a pedestrian."""
        actor = self._actors[instance_name].carla_actor
        control = carla.WalkerControl(
            direction=carla.Vector3D(x=direction[0], y=direction[1], z=0.0),
            speed=speed,
        )
        actor.apply_control(control)

    def get_all_actor_ids(self):
        """Get IDs of all spawned CARLA actors for cleanup."""
        ids = []
        for managed in self._actors.values():
            if managed.carla_actor is not None:
                ids.append(managed.carla_actor.id)
        return ids

    def destroy_all(self):
        """Batch destroy all actors and restore connection settings."""
        ids = self.get_all_actor_ids()
        self.conn.batch_destroy(ids)
        for managed in self._actors.values():
            managed.carla_actor = None
            managed.controller = None
        logging.info(f"Destroyed {len(ids)} actors")
