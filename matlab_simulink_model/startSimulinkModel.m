% *
%  * NIST-developed software is provided by NIST as a public service. You may use,
%  * copy, and distribute copies of the software in any medium, provided that you
%  * keep intact this entire notice. You may improve, modify, and create
%  * derivative works of the software or any portion of the software, and you may
%  * copy and distribute such modifications or works. Modified works should carry
%  * a notice stating that you changed the software and should note the date and
%  * nature of any such change. Please explicitly acknowledge the National
%  * Institute of Standards and Technology as the source of the software. 
%  *
%  * NIST-developed software is expressly provided "AS IS." NIST MAKES NO WARRANTY
%  * OF ANY KIND, EXPRESS, IMPLIED, IN FACT, OR ARISING BY OPERATION OF LAW,
%  * INCLUDING, WITHOUT LIMITATION, THE IMPLIED WARRANTY OF MERCHANTABILITY,
%  * FITNESS FOR A PARTICULAR PURPOSE, NON-INFRINGEMENT, AND DATA ACCURACY. NIST
%  * NEITHER REPRESENTS NOR WARRANTS THAT THE OPERATION OF THE SOFTWARE WILL BE
%  * UNINTERRUPTED OR ERROR-FREE, OR THAT ANY DEFECTS WILL BE CORRECTED. NIST DOES
%  * NOT WARRANT OR MAKE ANY REPRESENTATIONS REGARDING THE USE OF THE SOFTWARE OR
%  * THE RESULTS THEREOF, INCLUDING BUT NOT LIMITED TO THE CORRECTNESS, ACCURACY,
%  * RELIABILITY, OR USEFULNESS OF THE SOFTWARE.
%  * 
%  * You are solely responsible for determining the appropriateness of using and
%  * distributing the software and you assume all risks associated with its use,
%  * including but not limited to the risks and costs of program errors,
%  * compliance with applicable laws, damage to or loss of data, programs or
%  * equipment, and the unavailability or interruption of operation. This software 
%  * is not intended to be used in any situation where a failure could cause risk
%  * of injury or damage to property. The software developed by NIST employees is
%  * not subject to copyright protection within the United States.
%  *
%  * Author: Hadhoum Hajjaj <hadhoum.hajjaj@nist.gov>
% */
clear; close all; clc;

model_name = 'Traffic_Light_ROS2_NS3_demo';
% Set Variant Condition (AFTER Model is Loaded)
BRAKING_MODE = 2;  % 1 = Perception-Based, 2 = Communication-Based
set_param([model_name,'/Controller/Braking_Controller'], 'VariantControl', 'BRAKING_MODE');

% Preload and compile Simulink model
load_system(model_name);
set_param(model_name, 'SimulationCommand', 'update');  % Precompile for faster startup


triggerTime = 1.0; % To select when to start running simulink model


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