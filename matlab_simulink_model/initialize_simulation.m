% initialize_vehicle_simulation.m
% This script initializes the simulation environment for vehicle dynamics
% by loading parameters, running a Python script, and setting controller
% parameters.

%% Run the Python Script to Extract Vehicle Parameters
% Execute a Python script to generate and save vehicle parameters
system('python3 extract_vehicle_params.py');

%% Load Parameters from JSON File
% Decode the JSON file and assign vehicle parameters to MATLAB workspace variables
params = jsondecode(fileread('vehicle_params.json'));

% Assign vehicle physical parameters to workspace
assignin('base', 'mass', params.mass);
assignin('base', 'length', params.length);
assignin('base', 'width', params.width);
assignin('base', 'height', params.height);
assignin('base', 'yaw_moment_of_inertia', params.yaw_moment_of_inertia);
assignin('base', 'distance_to_front_tires', params.distance_to_front_tires);
assignin('base', 'distance_to_rear_tires', params.distance_to_rear_tires);
assignin('base', 'tire_radius', params.tire_radius);
assignin('base', 'tire_width', params.tire_width);
assignin('base', 'suspension_stiffness', params.suspension_stiffness);
assignin('base', 'suspension_damping', params.suspension_damping);
assignin('base', 'tire_friction', params.tire_friction);

% Assign vehicle performance limits to workspace
assignin('base', 'max_steering_angle', params.max_steering_angle);
assignin('base', 'max_throttle', params.max_throttle);
assignin('base', 'max_brake', params.max_brake);
assignin('base', 'cornering_stiffness_front', params.cornering_stiffness_front);
assignin('base', 'cornering_stiffness_rear', params.cornering_stiffness_rear);
assignin('base', 'longitudinal_accel_time_constant', params.longitudinal_accel_time_constant);

%% General Simulation Parameters
% Set the general simulation time step (s)
assignin('base', 'Ts', 0.1);

%% Path Following Controller Parameters
% Configure parameters for longitudinal and lateral control
assignin('base', 'tau', 0.5);                % Time constant for longitudinal dynamics (1/s/(tau*s+1))
assignin('base', 'time_gap', 1.5);           % Time gap for following distance (s)
assignin('base', 'default_spacing', 10);     % Default spacing to the leading vehicle (m)
assignin('base', 'max_ac', 2);               % Maximum acceleration (m/s^2)
assignin('base', 'min_ac', -3);              % Minimum acceleration (m/s^2)
assignin('base', 'max_steer', 0.35);         % Maximum steering angle (rad)
assignin('base', 'min_steer', -0.35);        % Minimum steering angle (rad)
assignin('base', 'PredictionHorizon', 30);   % Prediction horizon for MPC
assignin('base', 'ControlHorizon', 2);       % Control horizon for MPC
assignin('base', 'v0_ego', 14);              % Initial longitudinal velocity of ego vehicle (m/s)
assignin('base', 'tau2', 0.07);              % Longitudinal braking time constant
assignin('base', 'max_dc', -10);             % Maximum deceleration (m/s^2)
assignin('base', 'LaneWidth', single(3.85)); % Lane width (m)

%% Watchdog Braking Controller Parameters
% Parameters for safety braking in critical scenarios
assignin('base', 'PB1_decel', 3.8);          % Partial Braking stage 1 deceleration (m/s^2)
assignin('base', 'PB2_decel', 5.3);          % Partial Braking stage 2 deceleration (m/s^2)
assignin('base', 'FB_decel', 9.8);           % Full Braking deceleration (m/s^2)
assignin('base', 'headwayOffset', 3.7);      % Offset for safe headway (m)
assignin('base', 'timeMargin', 0);           % Additional time margin (s)
assignin('base', 'timeToReact', 1.2);        % Reaction time for the driver (s)
assignin('base', 'driver_decel', 4.0);       % Driver-applied braking deceleration (m/s^2)
assignin('base', 'Default_decel', 0);        % Default deceleration in normal scenarios (m/s^2)
assignin('base', 'TimeFactor', 1.2);         % Multiplier for reaction time (s)
assignin('base', 'stopVelThreshold', 0.1);   % Velocity threshold for a stopped vehicle (m/s)

%% Run the Helper Script to Create Bus Objects
% Execute an additional MATLAB script for setting up bus objects
helperCreateBus;

% End of script