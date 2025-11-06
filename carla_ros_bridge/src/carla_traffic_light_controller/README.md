# CARLA Traffic Light Controller - ROS2 Package

This ROS2 package provides a traffic light controller that integrates with CARLA ROS bridge and intermediate server for V2I communication.

## Features

- **ROS2 Native**: Clean integration with CARLA ROS bridge
- **V2I Communication**: Connects to intermediate server for traffic light data
- **Kinematic Control**: Same proven kinematic braking logic as python_controller_v2i.py
- **Data Logging**: Comprehensive JSON logging with deceleration metrics
- **Configurable Parameters**: Adjustable speed limits, deceleration rates, etc.

## Package Structure

```
carla_traffic_light_controller/
├── package.xml                          # Package dependencies
├── setup.py                            # Python package setup
├── resource/carla_traffic_light_controller  # ROS2 resource marker
├── launch/
│   └── traffic_light_controller.launch.py  # Launch file with parameters
└── carla_traffic_light_controller/
    ├── __init__.py                      # Python package init
    └── traffic_light_controller_node.py # Main controller node
```

## Prerequisites

1. **ROS2** (Humble, Galactic, or Foxy)
2. **CARLA ROS Bridge** workspace built and sourced
3. **CARLA Simulator** running
4. **Intermediate Server** running (connects NS3 and CARLA)

## Building the Package

1. Navigate to your CARLA ROS bridge workspace:
   ```bash
   cd /home/hnh21/iotav/cosim/carla_ros_bridge
   ```

2. Build the package:
   ```bash
   colcon build --packages-select carla_traffic_light_controller
   ```

3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Usage

### Method 1: Using Launch File (Recommended)

```bash
# Basic usage with default parameters
ros2 launch carla_traffic_light_controller traffic_light_controller.launch.py

# With custom parameters
ros2 launch carla_traffic_light_controller traffic_light_controller.launch.py \
    role_name:=hero \
    intermediate_server_port:=9000 \
    max_speed:=11.0 \
    stopline_offset:=25.0
```

### Method 2: Direct Node Execution

```bash
ros2 run carla_traffic_light_controller traffic_light_controller_node \
    --ros-args \
    -p role_name:=hero \
    -p intermediate_server_port:=9000
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `role_name` | `hero` | Name of the vehicle in CARLA |
| `intermediate_server_host` | `localhost` | NS3/intermediate server hostname |
| `intermediate_server_port` | `9000` | NS3/intermediate server port |
| `max_speed` | `13.41` | Maximum speed in m/s (30 mph) |
| `max_comfort_decel` | `2.8` | Comfortable deceleration in m/s² |
| `max_decel` | `8.0` | Emergency deceleration in m/s² |
| `stopline_offset` | `27.0` | Distance offset to stopline in meters |
| `control_dt` | `0.01` | Control loop frequency in seconds |

## Topics

### Subscribed Topics
- `/carla/{role_name}/odometry` - Vehicle position and velocity
- `/clock` - Simulation time

### Published Topics
- `/carla/{role_name}/vehicle_control_cmd` - Vehicle control commands

## Traffic Light Status Codes

- `1` = Green Light
- `2` = Yellow Light  
- `3` = Red Light

## Data Logging

The controller automatically logs detailed data to:
```
/home/hnh21/iotav/cosim/python_controller/results/ros2_controller_result_{timestamp}.json
```

### Logged Data Fields

Each step includes:
- `velocity`, `acceleration_commanded`, `acceleration_actual`
- `deceleration`, `deceleration_actual`, `acceleration_difference`
- `distance_to_stopline`, `traffic_light_status`, `v2i_msg`
- `final_action`, `debug_state`, `vehicle_stopped`
- `ros2_time`, `system_time`, `step`

## Control Logic

The controller implements the same kinematic braking logic as your original python controller:

1. **Green Light**: Cruise control to maintain target speed
2. **Yellow Light**: 
   - If close to intersection (< 10m): Continue through
   - Otherwise: Calculate kinematic braking to stop
3. **Red Light**: Apply kinematic braking to stop at stopline

## Monitoring and Debugging

### Check Node Status
```bash
ros2 node list | grep traffic_light
ros2 node info /traffic_light_controller
```

### Monitor Topics
```bash
# Vehicle control commands
ros2 topic echo /carla/hero/vehicle_control_cmd

# Vehicle odometry
ros2 topic echo /carla/hero/odometry

# Available CARLA topics
ros2 topic list | grep carla
```

### View Logs
```bash
ros2 run rqt_console rqt_console
```

## Integration with Your Existing Setup

This ROS2 controller is designed to work seamlessly with your existing infrastructure:

- **Intermediate Server**: Same socket connection on port 9000
- **Traffic Light Data**: Same format as python_controller_v2i.py
- **Distance Calculation**: Vehicle position - traffic light position - 27m offset
- **Kinematic Logic**: Identical braking calculations
- **JSON Logging**: Same detailed format for analysis

## Troubleshooting

### Common Issues

1. **Package not found after build**
   ```bash
   source install/setup.bash
   ```

2. **No odometry data**
   ```bash
   # Check if CARLA ROS bridge is running
   ros2 topic list | grep carla
   ```

3. **Intermediate server connection failed**
   - Verify intermediate server is running on port 9000
   - Check host/port parameters

4. **Vehicle not responding**
   ```bash
   # Check if control commands are being published
   ros2 topic hz /carla/hero/vehicle_control_cmd
   ```

## Comparison with Direct API Controller

| Feature | Direct API | ROS2 Controller |
|---------|------------|-----------------|
| **Architecture** | Monolithic | Modular, distributed |
| **Integration** | Tight CARLA coupling | Loose coupling via ROS2 |
| **Monitoring** | Custom logging | ROS2 introspection tools |
| **Testing** | Harder to test | Easy to test components |
| **Deployment** | Single process | Can run on different machines |
| **Control Logic** | ✅ Same kinematic braking | ✅ Same kinematic braking |
| **Data Logging** | ✅ Same JSON format | ✅ Same JSON format |

This ROS2 implementation provides the same control performance with much better modularity and integration capabilities.