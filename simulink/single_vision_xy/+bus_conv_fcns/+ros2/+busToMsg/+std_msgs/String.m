function rosmsgOut = String(slBusIn, rosmsgOut)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    rosmsgOut.data = char(slBusIn.data);
    if slBusIn.data_SL_Info.CurrentLength < numel(slBusIn.data)
    rosmsgOut.data(slBusIn.data_SL_Info.CurrentLength+1:numel(slBusIn.data)) = [];
    end
end
