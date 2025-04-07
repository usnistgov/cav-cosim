% Define the path to the custom message folder
customMsgFolder = fullfile('/home/hnh21/iotav/cosim/carla_ros_bridge/install/carla_msgs/share');

% Generate custom messages for ROS 2
ros2genmsg(customMsgFolder);

% Update MATLAB path to include the correct generated message definitions
addpath(fullfile(customMsgFolder, 'matlab_msg_gen', 'glnxa64'));

% Save the MATLAB path to a custom path file
savepath('/home/iotav/matlab_custom_pathdef.m');

% Verify that the messages are available
ros2("msg", "list");



