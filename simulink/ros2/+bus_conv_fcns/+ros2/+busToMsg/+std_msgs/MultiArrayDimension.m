function rosmsgOut = MultiArrayDimension(slBusIn, rosmsgOut)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    rosmsgOut.label = char(slBusIn.label);
    if slBusIn.label_SL_Info.CurrentLength < numel(slBusIn.label)
    rosmsgOut.label(slBusIn.label_SL_Info.CurrentLength+1:numel(slBusIn.label)) = [];
    end
    rosmsgOut.size = uint32(slBusIn.size);
    rosmsgOut.stride = uint32(slBusIn.stride);
end
