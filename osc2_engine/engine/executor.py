"""
Scenario Executor.

Executes a ScenarioIR against a live CARLA world by traversing the phase tree
and dispatching actions to the CARLA backend.
"""

import copy
import logging
import math
import time

import carla

from grammar.ir import (
    ScenarioIR, CompositionPhase, ActionPhase, WaitPhase, PhysicalValue,
)
from engine.actor_manager import ActorManager
from engine.action_handlers import create_handler
from engine.trigger_evaluator import TriggerEvaluator
from carla_backend.sensor_manager import SensorManager, SENSOR_CONFIGS
from metrics.collector import MetricsCollector


class PhaseRunner:
    """Wraps a phase for tick-by-tick execution in parallel blocks."""

    def __init__(self, phase, executor):
        self.phase = phase
        self.executor = executor
        self.done = False
        self._handler = None
        self._sub_runners = None
        self._serial_index = 0

        # Initialize based on phase type
        if isinstance(phase, ActionPhase):
            self._handler = create_handler(phase.action_name)
            self._handler.start(phase, executor.actors, executor.conn)
        elif isinstance(phase, WaitPhase):
            pass  # Evaluated each tick
        elif isinstance(phase, CompositionPhase):
            if phase.mode == "serial":
                self._init_serial()
            elif phase.mode == "parallel":
                self._sub_runners = [
                    PhaseRunner(c, executor) for c in phase.children
                ]

    def _init_serial(self):
        """Initialize the first child of a serial block."""
        if self._serial_index < len(self.phase.children):
            child = self.phase.children[self._serial_index]
            self._sub_runners = [PhaseRunner(child, self.executor)]
        else:
            self.done = True

    def step(self):
        """Advance this phase by one tick. Returns True when done."""
        if self.done:
            return True

        if isinstance(self.phase, ActionPhase):
            self.done = self._handler.step(self.executor.actors, self.executor.conn)
            return self.done

        elif isinstance(self.phase, WaitPhase):
            result = self.executor.trigger_eval.evaluate(self.phase.condition)
            if result:
                self.done = True
            return self.done

        elif isinstance(self.phase, CompositionPhase):
            if self.phase.mode == "serial":
                return self._step_serial()
            elif self.phase.mode == "parallel":
                return self._step_parallel()

        return True

    def _step_serial(self):
        """Step through serial children one at a time."""
        if not self._sub_runners:
            self.done = True
            return True

        runner = self._sub_runners[0]
        if runner.step():
            # Current child done, move to next
            self._serial_index += 1
            if self._serial_index < len(self.phase.children):
                child = self.phase.children[self._serial_index]
                self._sub_runners = [PhaseRunner(child, self.executor)]
            else:
                self.done = True
                return True
        return False

    def _step_parallel(self):
        """Step all parallel children, done when all complete."""
        all_done = True
        for runner in self._sub_runners:
            if not runner.done:
                runner.step()
                if not runner.done:
                    all_done = False
        if all_done:
            self.done = True
        return self.done


class ScenarioExecutor:
    """Execute a ScenarioIR against a live CARLA world."""

    def __init__(self, scenario_ir, conn, view_mode="chase"):
        self.ir = scenario_ir
        self.conn = conn
        self.actors = ActorManager(conn)
        params = {
            p.name: (p.default_value.value if isinstance(p.default_value, PhysicalValue) else p.default_value)
            for p in scenario_ir.params
        }
        self.trigger_eval = TriggerEvaluator(self.actors, params=params)
        self.sensor_manager = SensorManager(conn)
        sim_freq = float(scenario_ir.get_param("sim_frequency", 20.0))
        self._delta = 1.0 / sim_freq
        self.metrics = MetricsCollector(delta=self._delta)
        self._tick_count = 0
        self._view_mode = view_mode
        self.ego_ros2 = scenario_ir.get_param("ego_control", "python_api") == "ros2"
        self._ros2_speed_pub = None
        self._aeb_directive_pub = None
        self._ped_stopped = False
        self._ego_yielded_ticks = 0     # ticks ego has been ≈stopped while ped frozen
        self._ped_committed_to_cross = False  # once True, ped never re-freezes

        # Latest AEB state published on /metrics/state (CRUISE / BRAKE / NEAR_MISS).
        # Used by _check_ped_yields to suppress mutual yield when the AEB has
        # decided it must creep forward — only one of ego/ped resumes.
        self._aeb_state = "CRUISE"

        if self.ego_ros2:
            logging.info("Ego control: ROS2 (AEB communicates with CARLA natively)")
            try:
                import rclpy
                from rclpy.node import Node
                from std_msgs.msg import Float64, String
                if not rclpy.ok():
                    rclpy.init()
                self._ros2_node = rclpy.create_node("osc2_speed_pub")
                self._ros2_speed_pub = self._ros2_node.create_publisher(Float64, "/ego/speed", 10)
                # /scenario/aeb_directive: scenario uses ground-truth ego/ped
                # positions to tell the AEB "STAY" (room for ped, mutual yield)
                # or "CREEP" (no room, ego must drive forward to clear the
                # crosswalk). Default "STAY" is fail-safe — AEB stays stopped
                # if it never receives a directive.
                self._aeb_directive_pub = self._ros2_node.create_publisher(
                    String, "/scenario/aeb_directive", 10
                )
                self._ros2_node.create_subscription(
                    String, "/metrics/state", self._on_aeb_state, 10
                )
                logging.info("Publishing ego speed on /ego/speed and aeb_directive on /scenario/aeb_directive, subscribed to /metrics/state")
            except ImportError:
                logging.warning("rclpy not available — ego speed will not be published")

    def _on_aeb_state(self, msg):
        self._aeb_state = msg.data

    def _publish_speed(self):
        """Publish ego speed on ROS2 (only in ros2 mode), and pump the node's
        subscriber callbacks so /metrics/state updates self._aeb_state."""
        if self._ros2_speed_pub is not None:
            try:
                from std_msgs.msg import Float64
                speed = self.actors.get_speed("ego")
                msg = Float64()
                msg.data = float(speed)
                self._ros2_speed_pub.publish(msg)
            except (KeyError, AttributeError):
                pass
            try:
                import rclpy
                rclpy.spin_once(self._ros2_node, timeout_sec=0.0)
            except Exception:
                pass

    def _check_ped_yields(self):
        """Make the ped stop if the ego is visible and the AEB didn't brake.

        Near miss definition:
          - The AEB did NOT detect/brake for the ped
          - The ego passed through the crosswalk without stopping
          - The ped sees the ego (bus no longer blocks view) and stops
          - The ped saved themselves — the AEB failed to protect them

        If the AEB DID brake (ego stopped for the ped), the ped stopping
        is just normal behavior, not a near miss — that's SAFE.

        Once frozen, the ped resumes walking when EITHER:
          - the ego has fully cleared (longitudinal < -3m in ego body frame), OR
          - the ego has come to a full stop near the ped (speed < 0.5 m/s,
            dist > 1m) for >1s — mutual-yield resolution, prevents deadlock
            with a yield-until-clear AEB that stops short of the crosswalk.
        """
        # After a mutual-yield resume, the ped commits to crossing — don't
        # re-evaluate the freeze trigger or we'd oscillate freeze/unfreeze
        # every tick while the ego sits stopped just before the crosswalk.
        if self._ped_committed_to_cross:
            return

        try:
            ego_loc = self.actors.get_location("ego")
            ego_speed = self.actors.get_speed("ego")
            ped_loc = self.actors.get_location("ped")
            bus_loc = self.actors.get_location("bus")
            ego_transform = self.actors.get_transform("ego")
        except (KeyError, AttributeError):
            return

        # Compute ped position in ego's body frame.
        # longitudinal > 0: ped ahead of ego. lateral > 0: ped on ego's left
        # (approaching side for this scenario — ped walks +x, ego drives +y).
        dx = ped_loc.x - ego_loc.x
        dy = ped_loc.y - ego_loc.y
        yaw_rad = math.radians(ego_transform.rotation.yaw)
        longitudinal = math.cos(yaw_rad) * dx + math.sin(yaw_rad) * dy
        lateral = -math.sin(yaw_rad) * dx + math.cos(yaw_rad) * dy

        # If already frozen, the ped resumes walking in either of two cases:
        #   1) The ego has driven past — original "ego cleared" condition
        #      (longitudinal < -3m: ego's rear is past the ped + 1m margin).
        #   2) Mutual-yield resolution — the ego has come to a full stop near
        #      the ped without passing, so the ped (who has right-of-way at
        #      the crosswalk) re-asserts it after a short confirmation dwell.
        #      Without this branch, a yield-until-clear AEB and a yield-to-
        #      ego ped deadlock indefinitely.
        if self._ped_stopped:
            if longitudinal < -3.0:
                self.actors.unpause_walker("ped")
                self._ped_stopped = False
                self._ego_yielded_ticks = 0
                logging.info(
                    f"Ped resumes walking — ego cleared "
                    f"(longitudinal={longitudinal:.1f}m)"
                )
                return

            # Ground-truth room check using the *real* ego/ped positions.
            # Ego is a Lincoln MKZ (~4.5m), so half-length = 2.25m. Add 0.30m
            # for the ped's body half-width — that's the minimum forward
            # clearance from ego front bumper to ped path for safe crossing.
            EGO_HALF_LENGTH_M = 2.25
            PED_MARGIN_M = 0.30
            ROOM_THR = EGO_HALF_LENGTH_M + PED_MARGIN_M  # 2.55m

            # Publish the AEB directive whenever ego is ≈stopped near a frozen
            # ped, so the AEB can switch into NEAR_MISS_RESUME (creep) when
            # there's no room. Perception's last_x_fwd is unreliable at close
            # range (LiDAR drops, ground-plane fallback projects to the wrong
            # spot); using ground truth here puts the decision where the
            # information actually lives.
            ego_speed = self.actors.get_speed("ego")
            if ego_speed < 0.5:
                directive = "STAY" if longitudinal > ROOM_THR else "CREEP"
                if self._aeb_directive_pub is not None:
                    from std_msgs.msg import String
                    msg = String()
                    msg.data = directive
                    self._aeb_directive_pub.publish(msg)

            # Mutual-yield is suppressed when the AEB has decided NEAR_MISS
            # (ego is going to creep forward to clear the crosswalk). In that
            # case only the ego resumes — the ped stays frozen and crosses
            # behind the ego, not in front.
            if self._aeb_state == "NEAR_MISS":
                self._ego_yielded_ticks = 0
                return
            ego_speed_for_yield = self.actors.get_speed("ego")
            dist_for_yield = self.actors.distance_between("ego", "ped")
            # Mutual yield only fires when there's actual room in front of ego
            # (ground truth). Without this gate the ped would walk straight
            # into the front bumper of an ego stopped on the crosswalk.
            if (ego_speed_for_yield < 0.5
                    and dist_for_yield > 1.0
                    and longitudinal > ROOM_THR):
                self._ego_yielded_ticks += 1
                if self._ego_yielded_ticks * self._delta > 1.0:
                    self.actors.unpause_walker("ped")
                    self._ped_stopped = False
                    self._ego_yielded_ticks = 0
                    self._ped_committed_to_cross = True
                    logging.info(
                        f"Ped resumes walking — ego yielded "
                        f"(ego_speed={ego_speed_for_yield*3.6:.1f} km/h, "
                        f"longitudinal={longitudinal:.1f}m)"
                    )
            else:
                self._ego_yielded_ticks = 0
            return

        # Ped can only see ego after ego passes the bus.
        # This is also a natural proximity gate: by scenario geometry, the
        # moment ego clears the bus it is already 6-8m from the ped — no
        # separate distance threshold needed (the ped reacts to *seeing*
        # the ego, not to computing a distance).
        if ego_loc.y < bus_loc.y:
            return

        dist = self.actors.distance_between("ego", "ped")

        # Yield only if the ped is still APPROACHING the ego's path:
        #   - longitudinal > 0: ped must be ahead of ego (not already passed)
        #   - lateral > 1.0:    ped still on the approach side, outside the
        #                       car's footprint (half-width ~1m). Tighter than
        #                       before — ped freezes closer to the vehicle.
        # lateral <= 1.0 covers both the in-path zone (|lateral|<1.0, where
        # the AEB is the thing under test) and the already-crossed zone
        # (lateral<0, ped east of ego).
        if longitudinal <= 0 or lateral <= 1.0:
            return

        # Stop the pedestrian via pause flag — WalkHandler respects this each tick
        # instead of overriding with its walking speed.
        self.actors.pause_walker("ped")
        self._ped_stopped = True

        # Classification by clearance, not by ego speed: the speed-based rule
        # mislabels low-cruise cells where AEB stops the ego with margin but
        # the ego is still rolling slowly. A 2 m clearance is the deployment-
        # facing definition of a successful AEB stop (bumper plus margin).
        NEAR_MISS_CLEARANCE_M = 2.0
        if dist < NEAR_MISS_CLEARANCE_M:
            self.metrics.near_miss = True
            self.metrics.near_miss_distance = dist
            logging.info(
                f"NEAR MISS — ped yields, dist={dist:.1f}m < "
                f"{NEAR_MISS_CLEARANCE_M:.1f}m, ego_speed={ego_speed*3.6:.0f} km/h"
            )
        else:
            logging.info(
                f"Ped yields (safe) — clearance dist={dist:.1f}m, "
                f"ego_speed={ego_speed*3.6:.0f} km/h"
            )

    def run(self):
        """Full scenario lifecycle: setup -> execute -> teardown -> metrics."""
        try:
            self._setup()
            self._execute_phase(self.ir.do_block)
            return self.metrics.finalize()
        except KeyboardInterrupt:
            logging.info("\nScenario stopped by user (Ctrl+C)")
            return self.metrics.finalize()
        finally:
            self._teardown()

    def _setup(self):
        """Apply simulation settings, register actors with keep() constraint resolution."""
        # Load the required map if specified
        required_map = self.ir.get_param("map", "")
        if required_map:
            current_map = self.conn.world.get_map().name
            # Check if we need to switch (compare by short name)
            if required_map not in current_map:
                logging.info(f"Loading map: {required_map}")
                self.conn.client.load_world(required_map)
                time.sleep(5)
                self.conn.world = self.conn.client.get_world()
                self.conn.map = self.conn.world.get_map()
                # Let the world settle
                settings = self.conn.world.get_settings()
                settings.synchronous_mode = False
                self.conn.world.apply_settings(settings)
                time.sleep(1)
                logging.info(f"Map loaded: {self.conn.map.name}")

        # Clean up leftover actors from previous runs
        for actor in self.conn.world.get_actors().filter('vehicle.*'):
            actor.destroy()
        for actor in self.conn.world.get_actors().filter('walker.*'):
            actor.destroy()
        for actor in self.conn.world.get_actors().filter('sensor.*'):
            actor.destroy()
        logging.info("Cleared leftover actors from previous runs")
        time.sleep(1)  # Let CARLA fully clean up spawn points

        # Synchronous mode
        sim_freq = self.ir.get_param("sim_frequency", 20.0)
        delta = 1.0 / float(sim_freq)
        self.conn.enable_sync_mode(delta=delta)

        # Traffic lights
        tl_mode = self.ir.get_param("traffic_lights", "")
        if tl_mode:
            self.conn.set_traffic_lights(tl_mode)

        # Build extended fields from extend declarations (from lib/carla.osc)
        extended_fields = {}
        for base_type in ("vehicle", "person"):
            extended_fields[base_type] = self.ir.get_extended_fields(base_type)

        # Register actors, resolving keep() constraints into fields
        for inst in self.ir.actor_instances:
            # Check if this is a sensor struct
            struct_decl = self.ir.get_struct_decl(inst.type_name)
            if struct_decl is not None:
                # Per-instance clone — otherwise multiple instances of the same
                # struct type (e.g. two carla_rgb_camera declarations) all share
                # one fields dict and the last keep() block silently overwrites
                # the earlier ones.
                inst.resolved_struct = copy.copy(struct_decl)
                inst.resolved_struct.fields = dict(struct_decl.fields)
                for kc in inst.constraints:
                    if kc.op == "==":
                        inst.resolved_struct.fields[kc.property_name] = kc.value
                continue

            # Standard types (vehicle, person) — use extend fields + keep constraints
            base_type = inst.type_name
            if base_type in ("vehicle", "person"):
                fields = dict(extended_fields.get(base_type, {}))
            else:
                # Legacy: custom actor_decl (actor X inherits Y)
                actor_decl = self.ir.get_actor_decl(inst.type_name)
                if actor_decl:
                    base_type = actor_decl.parent_type
                    fields = dict(actor_decl.fields)
                else:
                    logging.warning(f"Unknown type '{inst.type_name}' for '{inst.instance_name}'")
                    continue

            # Apply keep() constraints as field overrides
            for kc in inst.constraints:
                if kc.op == "==":
                    fields[kc.property_name] = kc.value

            # Remap CARLA extension field names to engine-internal names
            if "blueprint" in fields and "model" not in fields:
                fields["model"] = fields.pop("blueprint")
            if "fallback_blueprints" in fields and "fallback_models" not in fields:
                fields["fallback_models"] = fields.pop("fallback_blueprints")

            self.actors.register(
                inst.instance_name,
                parent_type=base_type,
                fields=fields,
            )

        logging.info(f"Scenario '{self.ir.name}' setup complete")

    def _spawn_background_traffic(self):
        """Spawn background vehicles and pedestrians if configured."""
        num_vehicles = int(self.ir.get_param("background_vehicles", 0))
        num_pedestrians = int(self.ir.get_param("background_pedestrians", 0))

        if num_vehicles == 0 and num_pedestrians == 0:
            return

        # Collect scenario actor positions to avoid spawning near them
        exclude = []
        for inst in self.ir.actor_instances:
            try:
                loc = self.actors.get_location(inst.instance_name)
                exclude.append((loc.x, loc.y))
            except (KeyError, AttributeError):
                pass

        self.conn.spawn_background_traffic(
            num_vehicles=num_vehicles,
            num_pedestrians=num_pedestrians,
            exclude_positions=exclude,
        )

    def _sync_traffic_lights(self):
        """Sync opposing traffic lights if mode is sync_opposing."""
        tl_mode = self.ir.get_param("traffic_lights", "")
        if tl_mode != "sync_opposing":
            return

        try:
            ego_actor = self.actors.get_carla_actor("ego")
            bus_actor = self.actors.get_carla_actor("bus")
            if ego_actor and bus_actor:
                self.conn.sync_opposing_traffic_lights(ego_actor, bus_actor)
        except (KeyError, AttributeError) as e:
            logging.warning(f"Failed to sync traffic lights: {e}")

    def _spawn_sensors(self):
        """Spawn sensors defined as struct instances, attach to ego vehicle."""
        ego_actor = self.actors.get_carla_actor("ego")
        if ego_actor is None:
            logging.warning("Cannot spawn sensors: ego vehicle not found")
            return

        for inst in self.ir.actor_instances:
            resolved = getattr(inst, "resolved_struct", None)
            if resolved is not None and resolved.name in SENSOR_CONFIGS:
                self.sensor_manager.spawn_from_struct(resolved, ego_actor)

        # Attach collision sensor for metrics
        self.metrics.attach_collision_sensor(ego_actor, self.conn)

    def _execute_phase(self, phase):
        """Execute a phase tree (recursive for serial, tick-loop for parallel)."""
        if phase is None:
            return

        if isinstance(phase, CompositionPhase):
            if phase.mode == "serial":
                for child in phase.children:
                    self._execute_phase(child)
            elif phase.mode == "parallel":
                self._execute_parallel(phase.children)
            elif phase.mode == "one_of":
                self._execute_one_of(phase.children)

        elif isinstance(phase, ActionPhase):
            self._execute_action(phase)

        elif isinstance(phase, WaitPhase):
            self._execute_wait(phase)

    def _execute_parallel(self, children):
        """Run children concurrently in a tick loop until all complete."""
        runners = [PhaseRunner(child, self) for child in children]

        # If this is the first parallel block (spawning actors), tick afterward
        # to let CARLA settle, then spawn sensors
        has_assign = any(
            isinstance(c, ActionPhase) and c.action_name == "assign_position"
            for c in children
        )

        # Step all runners once to execute immediate actions
        for r in runners:
            r.step()

        if has_assign:
            # Tick to settle actor positions, then spawn sensors and background traffic
            self.conn.tick()
            time.sleep(0.2)
            self._spawn_sensors()
            self._spawn_background_traffic()
            self._sync_traffic_lights()
            # Write ego actor ID for external AV stack (ROS2 AEB)
            if self.ego_ros2:
                try:
                    ego_id = self.actors.get_carla_actor("ego").id
                    with open("/tmp/carla_ego_actor_id", "w") as f:
                        f.write(str(ego_id))
                    logging.info(f"Ego actor ID: {ego_id} (written to /tmp/carla_ego_actor_id)")
                    logging.info("Waiting for AEB node to connect...")
                    for _ in range(10):
                        self.conn.tick()
                        time.sleep(0.5)
                    logging.info("AEB connection window complete")
                except (KeyError, AttributeError):
                    pass
            self.conn.tick()
            time.sleep(0.2)
            return  # All assign_position actions are immediate

        # Main tick loop for ongoing parallel actions. 30 sim s covers the
        # V2X yield-until-clear case (ego stops on early CAM, waits up to
        # ~10 s for the scenario time-fallback to fire, then ~6 s for the
        # ped to clear the corridor). The perception baseline completes
        # well inside 30 s — bumping from 15 doesn't change those outcomes,
        # it just gives V2X cells room to capture the ego-resume tail.
        timeout_ticks = int(1.0 / self._delta) * (30 if self.ego_ros2 else 120)
        tick_in_phase = 0
        green_switched = False
        green_delay_ticks = int(5.0 / self._delta)  # 5 seconds of red before green

        while not all(r.done for r in runners):
            self.conn.tick()
            self._tick_count += 1
            tick_in_phase += 1
            self.trigger_eval.advance_time(self._delta)

            # Switch synced TLs from Red to Green after delay
            if not green_switched and tick_in_phase >= green_delay_ticks:
                self.conn.switch_synced_tls_to_green()
                green_switched = True

            # Collect metrics, check ped yielding, update spectator
            self.metrics.tick(self.actors)
            self._publish_speed()
            self._check_ped_yields()
            self._update_spectator()

            # Step all active runners
            for runner in runners:
                if not runner.done:
                    runner.step()

            # Log state periodically (every second)
            if self._tick_count % int(1.0 / self._delta) == 0:
                self._log_state()

            # Stop when ego passes 15m beyond crosswalk (y=-12 + 15 = 3)
            if self.ego_ros2:
                try:
                    if self.actors.get_location("ego").y > 3.0:
                        logging.info("Ego passed 15m beyond crosswalk — ending phase")
                        break
                except (KeyError, AttributeError):
                    pass

            # Stop 3s after collision
            if self.metrics.collision_detected:
                if not hasattr(self, '_collision_countdown'):
                    self._collision_countdown = int(3.0 / self._delta)
                    logging.info("Collision detected — stopping in 3 seconds")
                self._collision_countdown -= 1
                if self._collision_countdown <= 0:
                    logging.info("Post-collision observation complete — ending phase")
                    break

            # Timeout protection
            if tick_in_phase >= timeout_ticks:
                logging.warning("Parallel phase timed out")
                break

    def _execute_one_of(self, children):
        """Run children, stop when first completes."""
        runners = [PhaseRunner(child, self) for child in children]

        while True:
            self.conn.tick()
            self._tick_count += 1
            self.trigger_eval.advance_time(self._delta)
            self.metrics.tick(self.actors)
            self._publish_speed()
            self._update_spectator()

            for runner in runners:
                if not runner.done:
                    if runner.step():
                        return

            if self._tick_count % int(1.0 / self._delta) == 0:
                self._log_state()

    def _execute_action(self, phase):
        """Execute a single action (used in serial context)."""
        handler = create_handler(phase.action_name)
        handler.start(phase, self.actors, self.conn)

        if handler.step(self.actors, self.conn):
            return

        timeout_ticks = 20 * 120
        tick_in_action = 0
        while not handler.step(self.actors, self.conn):
            self.conn.tick()
            self._tick_count += 1
            tick_in_action += 1
            self.trigger_eval.advance_time(self._delta)
            self.metrics.tick(self.actors)
            self._publish_speed()
            self._update_spectator()

            if self._tick_count % int(1.0 / self._delta) == 0:
                self._log_state()
            if tick_in_action >= timeout_ticks:
                logging.warning(f"Action {phase.action_name} timed out")
                break

    def _execute_wait(self, phase):
        """Tick until a wait condition becomes true."""
        timeout_ticks = 20 * 120
        tick_in_wait = 0

        while not self.trigger_eval.evaluate(phase.condition):
            self.conn.tick()
            self._tick_count += 1
            tick_in_wait += 1
            self.trigger_eval.advance_time(self._delta)
            self.metrics.tick(self.actors)
            self._publish_speed()
            self._update_spectator()

            if self._tick_count % int(1.0 / self._delta) == 0:
                self._log_state()
            if tick_in_wait >= timeout_ticks:
                logging.warning("Wait condition timed out after 120s")
                break

    def _update_spectator(self):
        """Move the CARLA spectator camera to follow the ego vehicle."""
        try:
            ego_transform = self.actors.get_transform("ego")
            yaw_rad = math.radians(ego_transform.rotation.yaw)

            if self._view_mode == "driver":
                # First-person: driver's eye position inside the vehicle
                spectator_transform = carla.Transform(
                    carla.Location(
                        x=ego_transform.location.x + 0.5 * math.cos(yaw_rad),
                        y=ego_transform.location.y + 0.5 * math.sin(yaw_rad),
                        z=ego_transform.location.z + 1.8,
                    ),
                    carla.Rotation(pitch=0.0, yaw=ego_transform.rotation.yaw),
                )
            else:
                # Chase camera: 3rd person behind and above
                spectator_transform = carla.Transform(
                    carla.Location(
                        x=ego_transform.location.x - 15.0 * math.cos(yaw_rad),
                        y=ego_transform.location.y - 15.0 * math.sin(yaw_rad),
                        z=ego_transform.location.z + 10.0,
                    ),
                    carla.Rotation(pitch=-25.0, yaw=ego_transform.rotation.yaw),
                )

            self.conn.world.get_spectator().set_transform(spectator_transform)
        except Exception:
            pass

    def _log_state(self):
        """Log current actor states with full diagnostics."""
        try:
            ego_speed = self.actors.get_speed_kmh("ego")
            ego_loc = self.actors.get_location("ego")
            ego_transform = self.actors.get_transform("ego")

            parts = [
                f"[{self._tick_count * self._delta:.1f}s]",
                f"Ego: ({ego_loc.x:.1f}, {ego_loc.y:.1f}) yaw={ego_transform.rotation.yaw:.0f} @ {ego_speed:.1f} km/h",
            ]

            # Bus state
            try:
                bus_loc = self.actors.get_location("bus")
                bus_speed = self.actors.get_speed_kmh("bus")
                bus_transform = self.actors.get_transform("bus")
                ego_bus_dist = self.actors.distance_between("ego", "bus")
                parts.append(
                    f"Bus: ({bus_loc.x:.1f}, {bus_loc.y:.1f}) yaw={bus_transform.rotation.yaw:.0f} "
                    f"@ {bus_speed:.1f} km/h dist={ego_bus_dist:.1f}m"
                )
            except (KeyError, AttributeError):
                pass

            # Pedestrian state
            try:
                ped_loc = self.actors.get_location("ped")
                ped_speed = self.actors.get_speed("ped")
                ego_ped_dist = self.actors.distance_between("ego", "ped")
                parts.append(
                    f"Ped: ({ped_loc.x:.1f}, {ped_loc.y:.1f}) "
                    f"@ {ped_speed:.1f} m/s dist={ego_ped_dist:.1f}m"
                )
            except (KeyError, AttributeError):
                pass

            # Traffic light state for ego
            try:
                ego_vehicle = self.actors.get_carla_actor("ego")
                tl = ego_vehicle.get_traffic_light()
                if tl is not None:
                    tl_state = tl.get_state()
                    parts.append(f"EgoTL: {tl_state}")
                else:
                    parts.append("EgoTL: none")
            except Exception:
                parts.append("EgoTL: ?")

            # Traffic light state for bus
            try:
                bus_vehicle = self.actors.get_carla_actor("bus")
                tl = bus_vehicle.get_traffic_light()
                if tl is not None:
                    tl_state = tl.get_state()
                    parts.append(f"BusTL: {tl_state}")
                else:
                    parts.append("BusTL: none")
            except Exception:
                parts.append("BusTL: ?")

            logging.info(" | ".join(parts))
        except (KeyError, AttributeError):
            pass

    def _teardown(self):
        """Restore settings, destroy all actors."""
        logging.info("Cleaning up...")

        # Restore settings FIRST
        self.conn.restore_settings()

        # Collect all IDs for batch destruction
        sensor_ids = self.sensor_manager.destroy_all()
        actor_ids = self.actors.get_all_actor_ids()
        collision_ids = self.metrics.get_sensor_ids()
        bg_ids = self.conn.get_background_actor_ids()

        all_ids = sensor_ids + collision_ids + actor_ids + bg_ids
        self.conn.batch_destroy(all_ids)

        time.sleep(0.5)

        # Clean up ROS2 speed publisher if used
        if hasattr(self, '_ros2_node') and self._ros2_node is not None:
            try:
                self._ros2_node.destroy_node()
            except Exception:
                pass

        logging.info("Cleanup complete")
