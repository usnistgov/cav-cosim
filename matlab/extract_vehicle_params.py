import carla
import json

def get_vehicle_parameters(vehicle):
    # Get vehicle physics control
    physics_control = vehicle.get_physics_control()
    bounding_box = vehicle.bounding_box
    mass = physics_control.mass
    center_of_mass = physics_control.center_of_mass

    # Dimensions and distances
    length = bounding_box.extent.x * 2  # Vehicle length
    width = bounding_box.extent.y * 2   # Vehicle width
    height = bounding_box.extent.z * 2  # Vehicle height
    front_axle_position = center_of_mass.x + bounding_box.extent.x  # Front axle position
    rear_axle_position = center_of_mass.x - bounding_box.extent.x  # Rear axle position

    distance_to_front_tires = front_axle_position - center_of_mass.x
    distance_to_rear_tires = center_of_mass.x - rear_axle_position

    # Approximate yaw moment of inertia
    yaw_moment_of_inertia = (1 / 12) * mass * (length**2 + width**2)

    # Wheel properties
    wheels = physics_control.wheels
    num_wheels = len(wheels)
    tire_radius = wheels[0].radius if num_wheels > 0 else None
    tire_width = wheels[0].tire_friction if num_wheels > 0 else None

    # Suspension and friction (use getattr for attributes that might not exist)
    suspension_stiffness = getattr(wheels[0], 'suspension_stiffness', None) if num_wheels > 0 else None
    suspension_damping = getattr(wheels[0], 'damping_rate', None) if num_wheels > 0 else None
    tire_friction = getattr(wheels[0], 'tire_friction', None) if num_wheels > 0 else None

    # Steering and throttle limits
    max_steering_angle = wheels[0].max_steer_angle if num_wheels > 0 else None
    max_throttle = vehicle.attributes.get('max_throttle', 1.0)  # Default if not specified
    max_brake = vehicle.attributes.get('max_brake', 1.0)        # Default if not specified

    # Tire cornering stiffness (assumed or rough estimate if unknown)
    cornering_stiffness_front = 12000  # N/rad (default)
    cornering_stiffness_rear = 11000   # N/rad (default)

    # Longitudinal acceleration tracking time constant (assume or specify)
    longitudinal_accel_time_constant = 0.5  # Default (can be tuned later)

    return {
        "mass": mass,
        "length": length,
        "width": width,
        "height": height,
        "yaw_moment_of_inertia": yaw_moment_of_inertia,
        "distance_to_front_tires": distance_to_front_tires,
        "distance_to_rear_tires": distance_to_rear_tires,
        "tire_radius": tire_radius,
        "tire_width": tire_width,
        "suspension_stiffness": suspension_stiffness,
        "suspension_damping": suspension_damping,
        "tire_friction": tire_friction,
        "max_steering_angle": max_steering_angle,
        "max_throttle": max_throttle,
        "max_brake": max_brake,
        "cornering_stiffness_front": cornering_stiffness_front,
        "cornering_stiffness_rear": cornering_stiffness_rear,
        "longitudinal_accel_time_constant": longitudinal_accel_time_constant
    }

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    world = client.get_world()
    vehicles = world.get_actors().filter('vehicle.*')

    if len(vehicles) == 0:
        print("No vehicles found in the simulation. Please spawn a vehicle.")
        return
    else:
        vehicle = vehicles[0]  # Select the first spawned vehicle

    parameters = get_vehicle_parameters(vehicle)

    # Save parameters to JSON
    with open('vehicle_params.json', 'w') as f:
        json.dump(parameters, f, indent=4)

    print("Vehicle parameters saved to vehicle_params.json")

if __name__ == "__main__":
    main()
