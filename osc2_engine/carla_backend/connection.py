"""
CARLA Connection Manager.

Handles client connection, synchronous mode, traffic lights, and cleanup.
"""

import logging
import time

import carla


class CarlaConnection:
    """Manages CARLA client, world, and simulation settings."""

    def __init__(self, host='localhost', port=2000):
        self.host = host
        self.port = port
        self.client = None
        self.world = None
        self.map = None
        self._original_settings = None

    def connect(self):
        """Connect to CARLA server."""
        logging.info(f"Connecting to CARLA at {self.host}:{self.port}")
        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(30.0)
        self.world = self.client.get_world()
        self.map = self.world.get_map()
        logging.info(f"Connected. Map: {self.map.name}")

    def enable_sync_mode(self, delta=0.05):
        """Enable synchronous mode with fixed delta seconds."""
        self._original_settings = self.world.get_settings()
        self._delta = delta

        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = delta
        self.world.apply_settings(settings)

        traffic_manager = self.client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)
        traffic_manager.set_hybrid_physics_mode(True)
        traffic_manager.set_hybrid_physics_radius(50.0)

        freq = 1.0 / delta
        logging.info(f"Synchronous mode enabled ({freq:.0f} Hz, delta={delta}s, hybrid physics r=50m)")

    def tick(self):
        """Advance simulation by one tick, synced to real-time."""
        start = time.time()
        self.world.tick()
        elapsed = time.time() - start
        remaining = self._delta - elapsed
        if remaining > 0:
            time.sleep(remaining)

    def set_traffic_lights(self, mode):
        """Configure traffic lights based on mode string."""
        if mode == "all_green_frozen":
            for tl in self.world.get_actors().filter('traffic.traffic_light*'):
                tl.set_state(carla.TrafficLightState.Green)
                tl.set_green_time(9999.0)
                tl.freeze(True)
            logging.info("All traffic lights set to GREEN and frozen")
        elif mode == "normal":
            logging.info("Traffic lights: normal (default cycle)")
        elif mode == "sync_opposing":
            logging.info("Traffic lights: sync_opposing (will sync after actors spawn)")

    def sync_opposing_traffic_lights(self, actor_a, actor_b, green_time=60.0, yellow_time=3.0, red_time=2.0):
        """
        Synchronize traffic lights for two actors so they share the same green phase.

        Strategy: find at least one TL, then use its group to find the
        opposing TL. Both are set to cycle together.
        """
        tl_a = self._find_traffic_light_for_actor(actor_a)
        tl_b = self._find_traffic_light_for_actor(actor_b)

        if tl_a:
            logging.info(f"sync_opposing: found TL {tl_a.id} for ego")
        if tl_b:
            logging.info(f"sync_opposing: found TL {tl_b.id} for bus")

        # If only one was found, find the other from the same group
        if tl_a and not tl_b:
            tl_b = self._find_opposing_in_group(tl_a, actor_b)
        elif tl_b and not tl_a:
            tl_a = self._find_opposing_in_group(tl_b, actor_a)

        if tl_a is None and tl_b is None:
            logging.warning("sync_opposing: no traffic lights found")
            return

        # Collect unique TLs to sync
        tls_to_sync = []
        seen_ids = set()
        for tl in [tl_a, tl_b]:
            if tl is not None and tl.id not in seen_ids:
                tls_to_sync.append(tl)
                seen_ids.add(tl.id)

        # Freeze all TLs in the group at Red first
        if tls_to_sync:
            group = tls_to_sync[0].get_group_traffic_lights()
            for gtl in group:
                gtl.set_state(carla.TrafficLightState.Red)
                gtl.freeze(True)

        # Set our TLs timing, start frozen at Red
        for tl in tls_to_sync:
            tl.set_green_time(green_time)
            tl.set_yellow_time(yellow_time)
            tl.set_red_time(5.0)
            tl.set_state(carla.TrafficLightState.Red)

        # Store TLs for delayed green switch
        self._synced_tls = tls_to_sync
        self._synced_group = group if tls_to_sync else []

        logging.info(
            f"sync_opposing: {len(tls_to_sync)} TLs synced {[tl.id for tl in tls_to_sync]} "
            f"(starting Red, will switch to Green after delay)"
        )

    def switch_synced_tls_to_green(self):
        """Switch the synced opposing TLs from Red to Green and unfreeze."""
        if hasattr(self, '_synced_tls') and self._synced_tls:
            for tl in self._synced_tls:
                tl.set_state(carla.TrafficLightState.Green)
                tl.freeze(False)
            logging.info("sync_opposing: switched to GREEN")

    def _find_opposing_in_group(self, known_tl, target_actor):
        """Find a TL in the same group as known_tl that matches target_actor's direction."""
        target_loc = target_actor.get_location()
        target_wp = self.map.get_waypoint(target_loc, project_to_road=True)
        if target_wp is None:
            return None

        target_yaw = target_wp.transform.rotation.yaw

        group = known_tl.get_group_traffic_lights()
        for tl in group:
            if tl.id == known_tl.id:
                continue
            affected = tl.get_affected_lane_waypoints()
            for wp in affected:
                yaw_diff = abs(wp.transform.rotation.yaw - target_yaw)
                if yaw_diff > 350:
                    yaw_diff = 360 - yaw_diff
                if yaw_diff < 30:  # Same direction as target actor
                    logging.info(f"sync_opposing: found opposing TL {tl.id} from group for actor")
                    return tl
        return None

    def _find_traffic_light_for_actor(self, actor):
        """Find the traffic light that affects an actor's current road."""
        # First try: CARLA's built-in method
        try:
            tl = actor.get_traffic_light()
            if tl is not None:
                return tl
        except Exception:
            pass

        # Second try: find TL by matching road/lane from waypoint
        actor_loc = actor.get_location()
        actor_wp = self.map.get_waypoint(actor_loc, project_to_road=True)
        if actor_wp is None:
            return None

        # Trace forward to find the TL near the next junction
        current = actor_wp
        for _ in range(30):  # Look up to 150m ahead
            nexts = current.next(5.0)
            if not nexts:
                break
            current = nexts[0]
            if current.is_junction:
                break

        # Now search all TLs for one that affects this road
        best_tl = None
        best_dist = float('inf')
        for tl in self.world.get_actors().filter('traffic.traffic_light*'):
            tl_loc = tl.get_location()
            dist = tl_loc.distance(current.transform.location)
            if dist < 30 and dist < best_dist:
                # Verify it affects a road near our lane direction
                affected = tl.get_affected_lane_waypoints()
                for wp in affected:
                    yaw_diff = abs(wp.transform.rotation.yaw - actor_wp.transform.rotation.yaw)
                    if yaw_diff > 350:
                        yaw_diff = 360 - yaw_diff
                    if yaw_diff < 30:  # Same direction
                        best_tl = tl
                        best_dist = dist
                        break
        return best_tl

    def restore_settings(self):
        """Restore original world settings."""
        if self._original_settings and self.world:
            try:
                self.world.apply_settings(self._original_settings)
                logging.info("World settings restored")
            except Exception as e:
                logging.warning(f"Failed to restore settings: {e}")

    def batch_destroy(self, actor_ids):
        """Batch destroy actors by ID."""
        if actor_ids:
            try:
                self.client.apply_batch(
                    [carla.command.DestroyActor(x) for x in actor_ids]
                )
            except Exception as e:
                logging.warning(f"Failed to destroy actors: {e}")

    def get_blueprint_library(self):
        return self.world.get_blueprint_library()

    def get_waypoint(self, location, project_to_road=True):
        return self.map.get_waypoint(location, project_to_road=project_to_road)

    def spawn_actor(self, blueprint, transform, attach_to=None):
        """Spawn actor, raising on failure."""
        return self.world.spawn_actor(blueprint, transform, attach_to=attach_to)

    def try_spawn_actor(self, blueprint, transform):
        """Spawn actor, returning None on failure."""
        return self.world.try_spawn_actor(blueprint, transform)

    def spawn_background_traffic(self, num_vehicles=30, num_pedestrians=15,
                                  exclude_positions=None):
        """
        Spawn autonomous background traffic using TrafficManager and AI walkers.

        Args:
            num_vehicles: Number of background vehicles with autopilot
            num_pedestrians: Number of background pedestrians with AI controller
            exclude_positions: List of (x, y) tuples to avoid spawning near
        """
        import random

        self._bg_vehicles = []
        self._bg_walkers = []
        self._bg_walker_controllers = []

        bp_library = self.world.get_blueprint_library()
        spawn_points = self.map.get_spawn_points()
        random.shuffle(spawn_points)

        exclude_radius = 70.0
        exclude_positions = exclude_positions or []

        # --- Background vehicles ---
        # Only cars (4 wheels, no buses/trucks/emergency vehicles)
        large_vehicles = {'bus', 'firetruck', 'ambulance', 'fusorosa', 'european_hgv',
                          'sprinter', 'carlacola', 'cybertruck', 'fuso', 'mitsubishi'}
        vehicle_bps = bp_library.filter('vehicle.*')
        vehicle_bps = [bp for bp in vehicle_bps
                       if int(bp.get_attribute('number_of_wheels')) == 4
                       and not any(lv in bp.id.lower() for lv in large_vehicles)]

        spawned_vehicles = 0
        for sp in spawn_points:
            if spawned_vehicles >= num_vehicles:
                break

            # Skip spawn points near scenario actors
            too_close = False
            for ex, ey in exclude_positions:
                if sp.location.distance(carla.Location(x=ex, y=ey)) < exclude_radius:
                    too_close = True
                    break
            if too_close:
                continue

            bp = random.choice(vehicle_bps)
            if bp.has_attribute('color'):
                color = random.choice(bp.get_attribute('color').recommended_values)
                bp.set_attribute('color', color)

            vehicle = self.world.try_spawn_actor(bp, sp)
            if vehicle:
                vehicle.set_autopilot(True)
                self._bg_vehicles.append(vehicle)
                spawned_vehicles += 1

        logging.info(f"Spawned {spawned_vehicles} background vehicles with autopilot")

        # --- Background pedestrians ---
        walker_bps = bp_library.filter('walker.pedestrian.*')
        walker_controller_bp = bp_library.find('controller.ai.walker')

        spawned_walkers = 0
        for _ in range(num_pedestrians * 3):  # Try more times than needed
            if spawned_walkers >= num_pedestrians:
                break

            spawn_loc = self.world.get_random_location_from_navigation()
            if spawn_loc is None:
                continue

            # Skip near scenario actors
            too_close = False
            for ex, ey in exclude_positions:
                if spawn_loc.distance(carla.Location(x=ex, y=ey)) < exclude_radius:
                    too_close = True
                    break
            if too_close:
                continue

            bp = random.choice(walker_bps)
            if bp.has_attribute('is_invincible'):
                bp.set_attribute('is_invincible', 'false')

            spawn_transform = carla.Transform(spawn_loc)
            walker = self.world.try_spawn_actor(bp, spawn_transform)
            if walker:
                self._bg_walkers.append(walker)
                spawned_walkers += 1

        # Tick to let walkers settle, then spawn AI controllers
        self.world.tick()

        for walker in self._bg_walkers:
            controller = self.world.try_spawn_actor(
                walker_controller_bp, carla.Transform(), attach_to=walker
            )
            if controller:
                self._bg_walker_controllers.append(controller)

        # Start walker AI: walk to random destinations
        self.world.tick()
        for controller in self._bg_walker_controllers:
            dest = self.world.get_random_location_from_navigation()
            if dest:
                controller.start()
                controller.go_to_location(dest)
                controller.set_max_speed(1.0 + random.random() * 0.5)

        logging.info(f"Spawned {spawned_walkers} background pedestrians with AI")

    def get_background_actor_ids(self):
        """Return all background traffic actor IDs for cleanup."""
        ids = []
        for c in getattr(self, '_bg_walker_controllers', []):
            if c.is_alive:
                c.stop()
                ids.append(c.id)
        for w in getattr(self, '_bg_walkers', []):
            ids.append(w.id)
        for v in getattr(self, '_bg_vehicles', []):
            ids.append(v.id)
        return ids
