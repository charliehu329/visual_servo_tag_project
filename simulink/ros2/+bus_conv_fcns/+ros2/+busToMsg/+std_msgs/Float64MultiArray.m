function rosmsgOut = Float64MultiArray(slBusIn, rosmsgOut)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    rosmsgOut.layout = bus_conv_fcns.ros2.busToMsg.std_msgs.MultiArrayLayout(slBusIn.layout,rosmsgOut.layout(1));
    rosmsgOut.data = double(slBusIn.data(1:slBusIn.data_SL_Info.CurrentLength));
end
