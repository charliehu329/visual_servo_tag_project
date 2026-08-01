function [command_velocity,command_acceleration,command_count, ...
    command_age,command_is_new]=fcn(data,is_new,t)
%#codegen
% 缓存最近一次合法的7关节命令速度，并输出消息年龄和新鲜度。
persistent last_velocity last_time last_count previous_velocity initialized
if isempty(initialized)
    last_velocity=zeros(7,1); previous_velocity=zeros(7,1);
    last_time=-1e6; last_count=0; initialized=false;
end
command_velocity=last_velocity; command_acceleration=zeros(7,1);
command_count=last_count; command_age=max(0,t-last_time);
command_is_new=false;
if ~is_new || numel(data)<7, return; end

current=zeros(7,1); valid=true;
for k=1:7
    current(k)=double(data(k)); valid=valid&&isfinite(current(k));
end
if ~valid, return; end
if initialized, command_acceleration=(current-previous_velocity)*120; end
last_velocity=current; previous_velocity=current; last_time=t;
last_count=7; initialized=true;
command_velocity=current; command_count=7; command_age=0;
command_is_new=true;
end

