function slBusOut = PoseStamped(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.
    currentlength = length(slBusOut.header);
    for iter=1:currentlength
        slBusOut.header(iter) = bus_conv_fcns.ros2.msgToBus.std_msgs.Header(msgIn.header(iter),slBusOut(1).header(iter),varargin{:});
    end
    slBusOut.header = bus_conv_fcns.ros2.msgToBus.std_msgs.Header(msgIn.header,slBusOut(1).header,varargin{:});
    currentlength = length(slBusOut.pose);
    for iter=1:currentlength
        slBusOut.pose(iter) = bus_conv_fcns.ros2.msgToBus.geometry_msgs.Pose(msgIn.pose(iter),slBusOut(1).pose(iter),varargin{:});
    end
    slBusOut.pose = bus_conv_fcns.ros2.msgToBus.geometry_msgs.Pose(msgIn.pose,slBusOut(1).pose,varargin{:});
end
