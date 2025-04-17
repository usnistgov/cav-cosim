import carla
import socket
import time
import numpy as np
import sys
import random

def set_spectator_camera(world, vehicle, distance=10, height=5):
    transform = vehicle.get_transform()
    yaw_rad = np.deg2rad(transform.rotation.yaw)
    spectator = world.get_spectator()

    camera_location = carla.Location(
        x=transform.location.x - distance * np.cos(yaw_rad),
        y=transform.location.y - distance * np.sin(yaw_rad),
        z=transform.location.z + height
    )

    camera_rotation = carla.Rotation(
        pitch=-15,
        yaw=transform.rotation.yaw,
        roll=0
    )

    spectator.set_transform(carla.Transform(camera_location, camera_rotation))

# === Connect to CARLA ===
client = carla.Client('localhost', 2000)
client.set_timeout(15.0)

print("Loading Town03...")
world = client.load_world('Town03')
time.sleep(5)

# === Set weather (optional) ===
weather = carla.WeatherParameters.ClearNoon
world.set_weather(weather)

# === Choose blueprint for 4-wheeled car ===
blueprints = world.get_blueprint_library().filter('vehicle.*')
car_blueprints = [bp for bp in blueprints if int(bp.get_attribute('number_of_wheels')) == 4]
vehicle_bp = random.choice(car_blueprints)

# === Spawn at custom position ===
spawn_point = carla.Transform(
    carla.Location(x=-84.98, y=-20.00, z=0.5),
    carla.Rotation(yaw=-90)
)
vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
if vehicle is None:
    print("Vehicle spawn failed.")
    sys.exit(1)

print(f"Spawned vehicle: {vehicle.type_id} at {spawn_point.location}")

# === Define traffic light waypoint ===
target_location = carla.Location(x=-82.49, y=-163.68, z=0.5)
target_waypoint = world.get_map().get_waypoint(target_location)

# === Setup Traffic Manager path ===
tm = client.get_trafficmanager()
tm_port = tm.get_port()
tm.set_synchronous_mode(False)
tm.set_path(vehicle, [target_waypoint.transform.location])

vehicle.set_autopilot(True, tm_port)

# === Connect to Intermediate Server ===
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
while True:
    try:
        sock.connect(('localhost', 7000))
        print("Connected to intermediate server.")
        break
    except ConnectionRefusedError:
        print("Waiting for intermediate server...")
        time.sleep(1)

# === Main Loop ===
try:
    while True:
        set_spectator_camera(world, vehicle)

        snapshot = world.get_snapshot()
        carla_time = snapshot.timestamp.elapsed_seconds
        seconds = int(carla_time)
        nanoseconds = int((carla_time - seconds) * 1e9)

        transform = vehicle.get_transform()
        velocity = vehicle.get_velocity()
        speed = 3.6 * np.linalg.norm([velocity.x, velocity.y, velocity.z])

        msg = f"{seconds},{nanoseconds},{transform.location.x:.2f},{transform.location.y:.2f},{transform.location.z:.2f}," \
              f"{velocity.x:.2f},{velocity.y:.2f},{velocity.z:.2f},1;"
        sock.sendall(msg.encode())

        print(f"Sent: {msg.strip()}")
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Terminated by user.")

finally:
    vehicle.destroy()
    sock.close()
    print("Resources cleaned up.")
