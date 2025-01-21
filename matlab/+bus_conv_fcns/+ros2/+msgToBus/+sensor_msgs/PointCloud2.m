function slBusOut = PointCloud2(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.
    currentlength = length(slBusOut.header);
    for iter=1:currentlength
        slBusOut.header(iter) = bus_conv_fcns.ros2.msgToBus.std_msgs.Header(msgIn.header(iter),slBusOut(1).header(iter),varargin{:});
    end
    slBusOut.header = bus_conv_fcns.ros2.msgToBus.std_msgs.Header(msgIn.header,slBusOut(1).header,varargin{:});
    slBusOut.height = uint32(msgIn.height);
    slBusOut.width = uint32(msgIn.width);
    maxlength = length(slBusOut.fields);
    recvdlength = length(msgIn.fields);
    currentlength = min(maxlength, recvdlength);
    if (max(recvdlength) > maxlength) && ...
            isequal(varargin{1}{1},ros.slros.internal.bus.VarLenArrayTruncationAction.EmitWarning)
        diag = MSLDiagnostic([], ...
                             message('ros:slros:busconvert:TruncatedArray', ...
                                     'fields', msgIn.MessageType, maxlength, max(recvdlength), maxlength, varargin{2}));
        reportAsWarning(diag);
    end
    slBusOut.fields_SL_Info.ReceivedLength = uint32(recvdlength);
    slBusOut.fields_SL_Info.CurrentLength = uint32(currentlength);
    for iter=1:currentlength
        slBusOut.fields(iter) = bus_conv_fcns.ros2.msgToBus.sensor_msgs.PointField(msgIn.fields(iter),slBusOut(1).fields(iter),varargin{:});
    end
    slBusOut.is_bigendian = logical(msgIn.is_bigendian);
    slBusOut.point_step = uint32(msgIn.point_step);
    slBusOut.row_step = uint32(msgIn.row_step);
    maxlength = length(slBusOut.data);
    recvdlength = length(msgIn.data);
    currentlength = min(maxlength, recvdlength);
    if (max(recvdlength) > maxlength) && ...
            isequal(varargin{1}{1},ros.slros.internal.bus.VarLenArrayTruncationAction.EmitWarning)
        diag = MSLDiagnostic([], ...
                             message('ros:slros:busconvert:TruncatedArray', ...
                                     'data', msgIn.MessageType, maxlength, max(recvdlength), maxlength, varargin{2}));
        reportAsWarning(diag);
    end
    slBusOut.data_SL_Info.ReceivedLength = uint32(recvdlength);
    slBusOut.data_SL_Info.CurrentLength = uint32(currentlength);
    slBusOut.data = uint8(msgIn.data(1:slBusOut.data_SL_Info.CurrentLength));
    if recvdlength < maxlength
    slBusOut.data(recvdlength+1:maxlength) = 0;
    end
    slBusOut.is_dense = logical(msgIn.is_dense);
end
