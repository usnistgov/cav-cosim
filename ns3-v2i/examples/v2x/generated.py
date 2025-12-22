from .base import *

# === BEG GENERATED CLASS DECLARATION ===
class VehicleObject(JSONMobilityObject):
# === END GENERATED CLASS DECLARATION ===

# === BEG GENERATED CONSTANTS ===
    VEHICLEOBJECT_TYPE = "VehicleObject"
# === END GENERATED CONSTANTS ===

# === BEG GENERATED INITIALIZER ===
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
# === END GENERATED INITIALIZER ===

# === BEG GENERATED (DE)SERIALIZATION METHODS ===
    def serialize(self, obj):
        super().serialize(obj)
        obj[JSONObject.JSONOBJECT_TYPE] = VehicleObject.VEHICLEOBJECT_TYPE

    def deserialize(self, obj):
        super().deserialize(obj)
# === END GENERATED (DE)SERIALIZATION METHODS ===


# === BEG GENERATED CLASS DECLARATION ===
class TrafficLightObject(JSONMobilityObject):
# === END GENERATED CLASS DECLARATION ===

# === BEG GENERATED CONSTANTS ===
    TRAFFICLIGHTOBJECT_TYPE = "TrafficLightObject"
    LIGHT_STATUS_KEY = "light_status"
    TIME_REMAINING_KEY = "time_remaining"
# === END GENERATED CONSTANTS ===

# === BEG GENERATED INITIALIZER ===
    def __init__(self, light_status = None, time_remaining = None, **kwargs):
        super().__init__(**kwargs)
        self.light_status = light_status
        self.time_remaining = time_remaining
# === END GENERATED INITIALIZER ===

# === BEG GENERATED (DE)SERIALIZATION METHODS ===
    def serialize(self, obj):
        super().serialize(obj)
        obj[JSONObject.JSONOBJECT_TYPE] = TrafficLightObject.TRAFFICLIGHTOBJECT_TYPE
        if self.alive:
            obj[TrafficLightObject.LIGHT_STATUS_KEY] = self.light_status
            obj[TrafficLightObject.TIME_REMAINING_KEY] = self.time_remaining

    def deserialize(self, obj):
        super().deserialize(obj)
        if obj[JSONObject.JSONOBJECT_ID] == self.id and obj[JSONObject.JSONOBJECT_ALIVE]:
            if TrafficLightObject.LIGHT_STATUS_KEY in obj: self.light_status = obj[TrafficLightObject.LIGHT_STATUS_KEY]
            if TrafficLightObject.TIME_REMAINING_KEY in obj: self.time_remaining = obj[TrafficLightObject.TIME_REMAINING_KEY]
# === END GENERATED (DE)SERIALIZATION METHODS ===

    # must be positive
    TRAFFIC_LIGHT_0_DURATION = 28
    TRAFFIC_LIGHT_1_DURATION = 4
    TRAFFIC_LIGHT_2_DURATION = 28
    
    def update(self, time_period):
        super().update(time_period)
        self.time_remaining -= time_period
        while self.time_remaining < 0:
            if self.light_status == 0:
                self.light_status = 1
                self.time_remaining += TrafficLightObject.TRAFFIC_LIGHT_1_DURATION
            elif self.light_status == 1:
                self.light_status = 2
                self.time_remaining += TrafficLightObject.TRAFFIC_LIGHT_2_DURATION
            else:
                self.light_status = 0
                self.time_remaining += TrafficLightObject.TRAFFIC_LIGHT_0_DURATION