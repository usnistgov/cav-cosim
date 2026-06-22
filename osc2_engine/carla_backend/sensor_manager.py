"""
Sensor Manager.

Spawns and manages CARLA sensors based on OSC2 struct declarations.
"""

import logging

import carla


# Maps struct type names to CARLA sensor blueprints and attribute mappings
# Supports both standard (carla_rgb_camera) and legacy (rgb_camera_config) names
_CAMERA_CONFIG = {
    "blueprint": "sensor.camera.rgb",
    "attrs": {
        "image_size_x": "width",
        "image_size_y": "height",
        "fov": "fov",
    },
}
_LIDAR_CONFIG = {
    "blueprint": "sensor.lidar.ray_cast",
    "attrs": {
        "channels": "channels",
        "points_per_second": "points_per_second",
        "rotation_frequency": "rotation_frequency",
        "range": "range",
    },
}
SENSOR_CONFIGS = {
    "carla_rgb_camera": _CAMERA_CONFIG,
    "rgb_camera_config": _CAMERA_CONFIG,
    "carla_lidar": _LIDAR_CONFIG,
    "lidar_config": _LIDAR_CONFIG,
}


class SensorManager:
    """Spawn and manage sensors attached to actors."""

    def __init__(self, conn):
        self.conn = conn
        self.sensors = []

    def spawn_from_struct(self, struct_decl, attach_to):
        """
        Spawn a sensor based on an OSC2 struct declaration.

        Args:
            struct_decl: StructDecl with sensor config fields
            attach_to: CARLA actor to attach sensor to
        """
        config = SENSOR_CONFIGS.get(struct_decl.name)
        if config is None:
            logging.warning(f"Unknown sensor struct type: {struct_decl.name}")
            return None

        bp_library = self.conn.get_blueprint_library()
        bp = bp_library.find(config["blueprint"])

        # Set sensor-specific attributes
        applied = {}
        for carla_attr, struct_field in config["attrs"].items():
            if struct_field in struct_decl.fields:
                val = struct_decl.fields[struct_field]
                bp.set_attribute(carla_attr, str(int(val) if isinstance(val, float) and val == int(val) else val))
                applied[carla_attr] = val
        if applied:
            logging.info(f"  {struct_decl.name} attrs applied: {applied}")

        # Set ROS2 attributes
        ros_name = struct_decl.fields.get("ros_name", "")
        if ros_name:
            bp.set_attribute("ros_name", str(ros_name))
            bp.set_attribute("role_name", str(ros_name))

        # Build transform from struct fields
        x = float(struct_decl.fields.get("x", 0))
        y = float(struct_decl.fields.get("y", 0))
        z = float(struct_decl.fields.get("z", 0))
        pitch = float(struct_decl.fields.get("pitch", 0))
        yaw = float(struct_decl.fields.get("yaw", 0))
        roll = float(struct_decl.fields.get("roll", 0))

        transform = carla.Transform(
            carla.Location(x=x, y=y, z=z),
            carla.Rotation(pitch=pitch, yaw=yaw, roll=roll),
        )

        sensor = self.conn.spawn_actor(bp, transform, attach_to=attach_to)
        sensor.enable_for_ros()
        self.sensors.append(sensor)

        logging.info(
            f"Spawned {config['blueprint']} (ros_name={ros_name}) "
            f"at offset ({x}, {y}, {z})"
        )
        return sensor

    def get_actor_ids(self):
        """Return list of sensor actor IDs for cleanup."""
        return [s.id for s in self.sensors]

    def destroy_all(self):
        """Return actor IDs for batch destruction."""
        ids = self.get_actor_ids()
        self.sensors.clear()
        return ids
