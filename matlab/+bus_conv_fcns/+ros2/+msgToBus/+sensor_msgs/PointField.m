function slBusOut = PointField(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.
    slBusOut.name_SL_Info.ReceivedLength = uint32(strlength(msgIn.name));
    currlen  = min(slBusOut.name_SL_Info.ReceivedLength, length(slBusOut.name));
    slBusOut.name_SL_Info.CurrentLength = uint32(currlen);
    slBusOut.name(1:currlen) = uint8(char(msgIn.name(1:currlen))).';
    slBusOut.offset = uint32(msgIn.offset);
    slBusOut.datatype = uint8(msgIn.datatype);
    slBusOut.count = uint32(msgIn.count);
end
