clear; close all; clc;

model_name = 'Traffic_Light_ROS2_demo';
% Set Variant Condition (AFTER Model is Loaded)
BRAKING_MODE = 1;  % 1 = Perception-Based, 2 = Communication-Based
set_param([model_name,'/Controller/Braking_Controller'], 'VariantControl', 'BRAKING_MODE');

% Preload and compile Simulink model
load_system(model_name);
set_param(model_name, 'SimulationCommand', 'update');  % Precompile for faster startup


triggerTime = 10.0; % To select when to start running simulink model


% Initialize ROS2 and Subscribe to /clock
node = ros2node('/matlab_simulink_clock_node');
clockSub = ros2subscriber(node, '/clock', 'rosgraph_msgs/Clock');


%% Wait for /clock to reach triggerTime

while true
    msg = receive(clockSub, 30);  % Wait up to 10 seconds for a message

    % Safely compute time from msg (cast to double to avoid errors)
    currentTime = double(msg.clock.sec) + double(msg.clock.nanosec) * 1e-9;

    if currentTime >= triggerTime
        break
    end
end

out = sim(model_name, ...
             'ReturnWorkspaceOutputs', 'on', ...
             'SignalLogging', 'on', ...
             'SignalLoggingName', 'logsout');