function slBusOut = StereoFeatures(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.
    slBusOut.left_sequence = uint32(msgIn.left_sequence);
    slBusOut.right_sequence = uint32(msgIn.right_sequence);
    currentlength = length(slBusOut.left_capture_stamp);
    for iter=1:currentlength
        slBusOut.left_capture_stamp(iter) = bus_conv_fcns.ros2.msgToBus.builtin_interfaces.Time(msgIn.left_capture_stamp(iter),slBusOut(1).left_capture_stamp(iter),varargin{:});
    end
    slBusOut.left_capture_stamp = bus_conv_fcns.ros2.msgToBus.builtin_interfaces.Time(msgIn.left_capture_stamp,slBusOut(1).left_capture_stamp,varargin{:});
    currentlength = length(slBusOut.right_capture_stamp);
    for iter=1:currentlength
        slBusOut.right_capture_stamp(iter) = bus_conv_fcns.ros2.msgToBus.builtin_interfaces.Time(msgIn.right_capture_stamp(iter),slBusOut(1).right_capture_stamp(iter),varargin{:});
    end
    slBusOut.right_capture_stamp = bus_conv_fcns.ros2.msgToBus.builtin_interfaces.Time(msgIn.right_capture_stamp,slBusOut(1).right_capture_stamp,varargin{:});
    slBusOut.valid_left = logical(msgIn.valid_left);
    slBusOut.valid_right = logical(msgIn.valid_right);
    slBusOut.u_left = double(msgIn.u_left);
    slBusOut.v_left = double(msgIn.v_left);
    slBusOut.u_right = double(msgIn.u_right);
    slBusOut.v_right = double(msgIn.v_right);
    slBusOut.scale_left = double(msgIn.scale_left);
    slBusOut.scale_right = double(msgIn.scale_right);
end
