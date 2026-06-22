"""
Action Handlers.

Maps OSC2 standard and CARLA-extension actions to CARLA Python API calls.
Standard actions: drive, walk, assign_position, change_speed, remain_stationary
Legacy actions: stop (maps to change_speed with target 0)
"""

import logging
import math
from abc import ABC, abstractmethod

import carla
from agents.navigation.behavior_agent import BehaviorAgent

from grammar.ir import PhysicalValue


def resolve_modifier_value(modifier, key, default=None):
    """Extract a value from a modifier's named or positional args."""
    if key in modifier.args:
        val = modifier.args[key]
        return _to_float(val) if isinstance(val, (int, float, PhysicalValue)) else val
    if modifier.positional_args:
        val = modifier.positional_args[0]
        return _to_float(val) if isinstance(val, (int, float, PhysicalValue)) else val
    return default


def _to_float(val):
    if isinstance(val, PhysicalValue):
        return val.value
    return float(val)


class BaseActionHandler(ABC):
    """Base class for action handlers."""

    @abstractmethod
    def start(self, phase, actors, conn):
        pass

    @abstractmethod
    def step(self, actors, conn):
        pass


class AssignPositionHandler(BaseActionHandler):
    """Implements assign_position() — spawn/teleport an actor.
    Reads position from position() modifier (standard) or direct args (legacy)."""

    def __init__(self):
        self.actor_name = None

    def start(self, phase, actors, conn):
        self.actor_name = phase.actor_ref

        # Standard form: assign_position() with: position(x: ..., y: ...)
        position_mod = phase.get_modifier("position")
        if position_mod:
            x = _to_float(position_mod.args.get("x", 0))
            y = _to_float(position_mod.args.get("y", 0))
            z = _to_float(position_mod.args.get("z", 0.5))
            yaw = _to_float(position_mod.args.get("yaw", 0.0))
            project_to_road = position_mod.args.get("project_to_road", False)
        else:
            # Legacy form: assign_position(x: ..., y: ...)
            x = _to_float(phase.args.get("x", 0))
            y = _to_float(phase.args.get("y", 0))
            z = _to_float(phase.args.get("z", 0.5))
            yaw = _to_float(phase.args.get("yaw", 0.0))
            project_to_road = phase.args.get("project_to_road", False)

        if isinstance(project_to_road, PhysicalValue):
            project_to_road = False

        actors.spawn(self.actor_name, x=x, y=y, z=z, yaw=yaw,
                     project_to_road=project_to_road)

    def step(self, actors, conn):
        return True


class DriveHandler(BaseActionHandler):
    """Implements vehicle.drive() via BehaviorAgent or raw throttle control.

    Standard modifiers: speed(), position()
    CARLA extension modifiers: carla_behavior(), carla_throttle()
    Legacy modifiers: behavior(), throttle(), keep_straight()
    """

    def __init__(self):
        self.actor_name = None
        self.use_agent = False
        self.throttle = 0.0
        self.keep_straight = False
        self._done = False

    def start(self, phase, actors, conn):
        self.actor_name = phase.actor_ref

        # Accept both standard and legacy modifier names
        behavior_mod = phase.get_modifier("carla_behavior") or phase.get_modifier("behavior")
        throttle_mod = phase.get_modifier("carla_throttle") or phase.get_modifier("throttle")
        position_mod = phase.get_modifier("position")

        if behavior_mod is not None:
            self.use_agent = True

            # Extract behavior mode from named arg or positional
            if behavior_mod.name == "carla_behavior":
                behavior = behavior_mod.args.get("mode", "cautious")
            elif behavior_mod.positional_args:
                val = behavior_mod.positional_args[0]
                behavior = val if isinstance(val, str) else str(val.value if hasattr(val, 'value') else val)
            else:
                behavior = "cautious"

            vehicle = actors.get_carla_actor(self.actor_name)
            agent = BehaviorAgent(vehicle, behavior=behavior)

            if position_mod:
                dest_x = _to_float(position_mod.args.get("x", 0))
                dest_y = _to_float(position_mod.args.get("y", 0))
                dest_z = _to_float(position_mod.args.get("z", 0))
                destination = carla.Location(x=dest_x, y=dest_y, z=dest_z)
            else:
                loc = actors.get_location(self.actor_name)
                destination = carla.Location(x=loc.x + 100, y=loc.y, z=loc.z)

            agent.set_destination(destination)
            actors.set_controller(self.actor_name, agent)
            logging.info(f"'{self.actor_name}' driving with BehaviorAgent ({behavior})")

        elif throttle_mod is not None:
            self.use_agent = False
            if throttle_mod.name == "carla_throttle":
                self.throttle = _to_float(throttle_mod.args.get("value", 0.5))
                self.keep_straight = throttle_mod.args.get("keep_straight", False)
                if isinstance(self.keep_straight, PhysicalValue):
                    self.keep_straight = False
            else:
                self.throttle = resolve_modifier_value(throttle_mod, "throttle", 0.5)
                if isinstance(self.throttle, PhysicalValue):
                    self.throttle = self.throttle.value
                self.keep_straight = phase.get_modifier("keep_straight") is not None
            logging.info(f"'{self.actor_name}' driving with throttle={self.throttle}")

    def step(self, actors, conn):
        if self._done:
            return True

        if self.use_agent:
            agent = actors.get_controller(self.actor_name)
            if agent is None:
                return True
            try:
                control = agent.run_step()
                actors.get_carla_actor(self.actor_name).apply_control(control)
            except Exception:
                # BehaviorAgent lost route (reached destination or invalid waypoint)
                # Stop the vehicle and mark done
                if not self._done:
                    logging.info(f"'{self.actor_name}' reached end of route, stopping")
                    self._done = True
                actors.apply_vehicle_control(self.actor_name, brake=1.0)
            return self._done
        else:
            actors.apply_vehicle_control(
                self.actor_name,
                throttle=self.throttle,
                steer=0.0,
                brake=0.0,
                hand_brake=False,
            )
            return False


class WalkHandler(BaseActionHandler):
    """Implements person.walk() via physics-driven WalkerControl.

    Standard modifiers: speed(), along(heading, distance)
    Legacy modifiers: direction(x, y), until_x(value)

    Physics stays enabled: the walker is moved by applying WalkerControl
    (direction + speed) each tick, which bypasses the nav mesh (nav mesh only
    binds WalkerAIController). If the executor calls actors.pause_walker(),
    this handler applies speed=0 and stops advancing until unpaused.
    """

    def __init__(self):
        self.actor_name = None
        self.speed = 1.4
        self.direction = (-1.0, 0.0)
        self.target_x = None
        self.target_y = None
        self.target_distance = None
        self._start_loc = None
        self._done = False

    def start(self, phase, actors, conn):
        self.actor_name = phase.actor_ref

        speed_mod = phase.get_modifier("speed")
        along_mod = phase.get_modifier("along")
        direction_mod = phase.get_modifier("direction")
        until_x_mod = phase.get_modifier("until_x")

        if speed_mod:
            self.speed = resolve_modifier_value(speed_mod, "speed", 1.4)
            if isinstance(self.speed, PhysicalValue):
                self.speed = self.speed.value

        if along_mod:
            # Standard form: along(heading: 180deg, distance: 13m)
            heading_val = along_mod.args.get("heading", PhysicalValue(180, "deg"))
            heading_deg = _to_float(heading_val)
            distance_val = along_mod.args.get("distance", PhysicalValue(10, "m"))
            total_distance = _to_float(distance_val)

            heading_rad = math.radians(heading_deg)
            dx = math.cos(heading_rad)
            dy = math.sin(heading_rad)
            length = math.sqrt(dx**2 + dy**2)
            if length > 0:
                self.direction = (dx / length, dy / length)

            self._start_loc = actors.get_location(self.actor_name)
            self.target_x = self._start_loc.x + self.direction[0] * total_distance
            self.target_y = self._start_loc.y + self.direction[1] * total_distance
            self.target_distance = total_distance

        elif direction_mod:
            # Legacy form: direction(x, y) + until_x(value)
            dx = _to_float(direction_mod.args.get("x", -1.0))
            dy = _to_float(direction_mod.args.get("y", 0.0))
            length = math.sqrt(dx**2 + dy**2)
            if length > 0:
                self.direction = (dx / length, dy / length)

            if until_x_mod:
                val = resolve_modifier_value(until_x_mod, "x", None)
                if val is None and until_x_mod.positional_args:
                    val = until_x_mod.positional_args[0]
                self.target_x = _to_float(val) if val is not None else None

        actors.apply_walker_control(
            self.actor_name,
            direction=self.direction,
            speed=self.speed,
        )

        logging.info(
            f"'{self.actor_name}' walking: speed={self.speed}m/s, "
            f"dir=({self.direction[0]:.2f}, {self.direction[1]:.2f})"
        )

    def step(self, actors, conn):
        if self._done:
            return True

        # External pause (e.g. ped yielding to ego): apply speed=0 and hold.
        if actors.is_walker_paused(self.actor_name):
            actors.apply_walker_control(
                self.actor_name, direction=self.direction, speed=0.0,
            )
            return False

        loc = actors.get_location(self.actor_name)

        # Check termination: distance-based (along) or x-based (legacy)
        if self.target_distance is not None and self._start_loc is not None:
            traveled = math.sqrt(
                (loc.x - self._start_loc.x)**2 +
                (loc.y - self._start_loc.y)**2
            )
            if traveled >= self.target_distance:
                self._stop(actors)
                return True
        elif self.target_x is not None:
            if self.direction[0] < 0 and loc.x <= self.target_x:
                self._stop(actors)
                return True
            elif self.direction[0] > 0 and loc.x >= self.target_x:
                self._stop(actors)
                return True

        actors.apply_walker_control(
            self.actor_name,
            direction=self.direction,
            speed=self.speed,
        )
        return False

    def _stop(self, actors):
        actors.apply_walker_control(self.actor_name, direction=self.direction, speed=0.0)
        self._done = True
        logging.info(f"'{self.actor_name}' finished crossing")


class ChangeSpeedHandler(BaseActionHandler):
    """Implements change_speed(target: 0kph) — standard OSC2 speed change."""

    def __init__(self):
        self.actor_name = None
        self.target_speed_ms = 0.0

    def start(self, phase, actors, conn):
        self.actor_name = phase.actor_ref
        target = phase.args.get("target", PhysicalValue(0, "kph"))
        if isinstance(target, PhysicalValue):
            if target.unit == "kph":
                self.target_speed_ms = target.value / 3.6
            else:
                self.target_speed_ms = target.value
        else:
            self.target_speed_ms = _to_float(target)

        # Clear any BehaviorAgent so DriveHandler yields control
        try:
            actors.set_controller(self.actor_name, None)
        except (KeyError, AttributeError):
            pass

        if self.target_speed_ms <= 0.01:
            actors.apply_vehicle_control(
                self.actor_name, throttle=0.0, brake=1.0, hand_brake=True,
            )
            logging.info(f"'{self.actor_name}' change_speed to 0 (full brake)")

    def step(self, actors, conn):
        if self.target_speed_ms <= 0.01:
            actors.apply_vehicle_control(
                self.actor_name, throttle=0.0, brake=1.0, hand_brake=True,
            )
        return True


class StopHandler(BaseActionHandler):
    """Legacy stop() — delegates to change_speed(target: 0kph)."""

    def __init__(self):
        self.actor_name = None

    def start(self, phase, actors, conn):
        self.actor_name = phase.actor_ref
        actors.apply_vehicle_control(
            self.actor_name, throttle=0.0, brake=1.0, hand_brake=True,
        )
        logging.info(f"'{self.actor_name}' stopping (full brake)")

    def step(self, actors, conn):
        actors.apply_vehicle_control(
            self.actor_name, throttle=0.0, brake=1.0, hand_brake=True,
        )
        return True


class RemainStationaryHandler(BaseActionHandler):
    """Implements remain_stationary() — parked vehicle with handbrake."""

    def __init__(self):
        self.actor_name = None

    def start(self, phase, actors, conn):
        self.actor_name = phase.actor_ref
        managed = actors.get(self.actor_name)
        if managed.parent_type in ("vehicle",):
            actors.apply_vehicle_control(self.actor_name, hand_brake=True)
        logging.debug(f"'{self.actor_name}' remaining stationary")

    def step(self, actors, conn):
        return False


# Registry: standard + CARLA extension + legacy action names
ACTION_REGISTRY = {
    "assign_position": AssignPositionHandler,
    "drive": DriveHandler,
    "walk": WalkHandler,
    "change_speed": ChangeSpeedHandler,
    "stop": StopHandler,  # Legacy
    "remain_stationary": RemainStationaryHandler,
}


def create_handler(action_name):
    cls = ACTION_REGISTRY.get(action_name)
    if cls is None:
        raise ValueError(f"Unknown action: {action_name}")
    return cls()
