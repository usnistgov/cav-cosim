function rosmsgOut = CarlaEgoVehicleControl(slBusIn, rosmsgOut)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    rosmsgOut.header = bus_conv_fcns.ros2.busToMsg.std_msgs.Header(slBusIn.header,rosmsgOut.header(1));
    rosmsgOut.throttle = single(slBusIn.throttle);
    rosmsgOut.steer = single(slBusIn.steer);
    rosmsgOut.brake = single(slBusIn.brake);
    rosmsgOut.hand_brake = logical(slBusIn.hand_brake);
    rosmsgOut.reverse = logical(slBusIn.reverse);
    rosmsgOut.gear = int32(slBusIn.gear);
    rosmsgOut.manual_gear_shift = logical(slBusIn.manual_gear_shift);
end
