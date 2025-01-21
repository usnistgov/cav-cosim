% Define the path to the custom message folder
customMsgFolder = fullfile('/home/iotav/iotav/carla-ros-bridge-matlab/install/carla_msgs/share');

% Generate custom messages for ROS 2
ros2genmsg(customMsgFolder);

% Update MATLAB path to include generated message definitions

addpath(fullfile(matlabroot, 'ros2', 'share', 'matlab_msgs', 'msggen'));
savepath;

% Verify that the messages are available
ros2("msg", "list");

