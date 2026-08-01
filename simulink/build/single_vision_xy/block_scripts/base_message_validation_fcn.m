function [position_valid,velocity_valid,position_C,relative_velocity_C, ...
    joint_position_out,joint_velocity_out,joint_geometry_valid, ...
    joint_motion_valid] = fcn(target_data,target_count,target_age, ...
    joint_position,joint_count,joint_velocity,joint_age, ...
    target_timeout_sec,joint_state_timeout_sec)
%#codegen
% 校验视觉测量和 JointState，并分离几何与运动有效性。
[target_position_valid,target_velocity_valid,position_C,relative_velocity_C]= ...
    parse_target_measurement(target_data,target_count,target_age,target_timeout_sec);

joint_position_out=zeros(7,1);
joint_velocity_out=zeros(7,1);
joint_geometry_valid=joint_count>=7 && numel(joint_position)>=7 && ...
    all(isfinite(joint_position(1:7))) && isfinite(joint_age) && ...
    joint_age>=0 && joint_age<=joint_state_timeout_sec;
joint_motion_valid=joint_geometry_valid && numel(joint_velocity)>=7 && ...
    all(isfinite(joint_velocity(1:7)));

if joint_geometry_valid
    joint_position_out=reshape(joint_position(1:7),7,1);
end
if joint_motion_valid
    joint_velocity_out=reshape(joint_velocity(1:7),7,1);
end

position_valid=target_position_valid&&joint_geometry_valid;
velocity_valid=target_velocity_valid&&joint_motion_valid;
end
