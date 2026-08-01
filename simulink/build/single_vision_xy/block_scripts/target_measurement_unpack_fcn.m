function [target_data,target_count,target_age] = fcn(data,is_new,t)
%#codegen
% 缓存带图像源时间戳的固定八元素视觉消息。
persistent last_data last_count last_time
if isempty(last_count)
    last_data=zeros(8,1);
    last_count=0;
    last_time=-1e6;
end
if is_new
    n=numel(data);
    last_data=zeros(8,1);
    for k=1:min(n,8)
        last_data(k)=double(data(k));
    end
    last_count=double(n);
    last_time=t;
end
target_data=last_data;
target_count=last_count;
target_age=max(0,t-last_time);
end
