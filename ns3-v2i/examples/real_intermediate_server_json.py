import random
import carla
import time
import v2x.base
import v2x.generated
import typing
import socket
import json

# 1. Connect to NS-3

EOT = '\u0004'

ns3_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
ns3_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
ns3_server.bind(('localhost', 8100))
ns3_server.listen(1)
print("Listening for ns-3 on port 8100...")

ns3_conn, _ = ns3_server.accept()
print("ns-3 connected.")

# 2. Connect to Carla

client = carla.Client('127.0.0.1', 2000)
client.set_timeout(60.0) # seconds


# 3. Initialize Carla
world = client.get_world()

time_step = 1.0
# set time step
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = time_step
settings.no_rendering_mode = True
world.apply_settings(settings)

# load map
client.load_world(random.choice(client.get_available_maps()))

# load vehicles
vehicle_blueprints = world.get_blueprint_library().filter('*vehicle*')

spawn_points = world.get_map().get_spawn_points()

world.try_spawn_actor(random.choice(vehicle_blueprints), random.choice(spawn_points))

ego_bp = random.choice(vehicle_blueprints)
ego_bp.set_attribute('role_name', 'hero')
ego = world.try_spawn_actor(ego_bp, random.choice(spawn_points))

for vehicle in world.get_actors().filter('*vehicle*'):
    if isinstance(vehicle, carla.Vehicle):
        vehicle.set_autopilot(True)

objects = dict[int, v2x.base.JSONMobilityObject]()


try:
    # time tracker
    time_s = 0
    time_ns = 0

    while True:
        world.tick()

        time_s += time_step

        # create/update JSON objects
        for actor in world.get_actors().filter('*vehicle*'):
            if isinstance(actor, carla.Vehicle):
                id = actor.id
                if not id in objects: objects[id] = v2x.generated.VehicleObject(id=id)
                vehicle = objects[id]
                typing.assert_type(vehicle, v2x.generated.VehicleObject)
                location = actor.get_location()
                vehicle.pos_x = location.x
                vehicle.pos_y = location.y
                vehicle.pos_z = location.z
                velocity = actor.get_velocity()
                vehicle.vel_x = velocity.x
                vehicle.vel_y = velocity.y
                vehicle.vel_z = velocity.z
        
        for actor in world.get_actors().filter('*traffic_light*'):
            if isinstance(actor, carla.TrafficLight):
                id = actor.id
                if not id in objects: objects[id] = v2x.generated.TrafficLightObject(id=id)
                trafficlight = objects[id]
                typing.assert_type(vehicle, v2x.generated.TrafficLightObject)
                location = actor.get_location()
                trafficlight.pos_x = location.x
                trafficlight.pos_y = location.y
                trafficlight.pos_z = location.z
                state = actor.get_state()
                if state == carla.TrafficLightState.Green: trafficlight.light_status = 0
                elif state == carla.TrafficLightState.Yellow: trafficlight.light_status = 1
                elif state == carla.TrafficLightState.Red: trafficlight.light_status = 2
                else: trafficlight.light_status = 4
                trafficlight.time_remaining = actor.get_elapsed_time()

        json_objects = []
        for obj in objects.values():
            json_obj = {}
            obj.serialize(json_obj)
            json_objects.append(json_obj)        

            ns3_message = json.dumps({"time_s": time_s, "time_ns": time_ns, "objects": json_objects}) + EOT

        try:
            ns3_conn.sendall(ns3_message.encode())
            print(f"[Intermediate] INFO: Sent fake message to ns-3: {ns3_message.strip()}")
        except BrokenPipeError:
            print("[Intermediate] ERROR: ns-3 disconnected.")
            raise SystemExit(1)  # Clean exit

        # Wait for ns-3 response
        response = ''
        # TODO: handle multiple JSON messages at once?
        while not response.endswith(EOT):
            chunk = ns3_conn.recv(1024).decode()
            if not chunk:
                break
            response += chunk

        print(f"[Intermediate] Received from ns-3: {response.strip(EOT)}")

        time.sleep(time_step)

except KeyboardInterrupt:
    print("Shutting down due to Ctrl+C...")

finally:
    ns3_conn.close()
    ns3_server.close()