function rosmsgOut = MultiArrayLayout(slBusIn, rosmsgOut)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    for iter=1:slBusIn.dim_SL_Info.CurrentLength
        rosmsgOut.dim(iter) = bus_conv_fcns.ros2.busToMsg.std_msgs.MultiArrayDimension(slBusIn.dim(iter),rosmsgOut.dim(1));
    end
    if slBusIn.dim_SL_Info.CurrentLength < numel(rosmsgOut.dim)
    rosmsgOut.dim(slBusIn.dim_SL_Info.CurrentLength+1:numel(rosmsgOut.dim)) = [];
    end
    rosmsgOut.data_offset = uint32(slBusIn.data_offset);
end
