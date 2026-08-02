function [camera_twist_base,camera_motion_valid,T_camera2base_out] = fcn( ...
    J_camera_base,joint_velocity,T_camera2base,joint_motion_valid, ...
    joint_geometry_valid)
%#codegen
% 使用已校验的实测关节速度计算 Base 系相机 Twist。
camera_twist_base=zeros(6,1);
camera_motion_valid=false;
T_camera2base_out=zeros(4,4);

transform_ok=joint_geometry_valid>.5 && ...
    isequal(size(T_camera2base),[4 4]) && all(isfinite(T_camera2base(:)));
if transform_ok
    T_camera2base_out=T_camera2base;
end
if ~(joint_motion_valid>.5 && transform_ok)
    return
end

if isequal(size(J_camera_base),[6 7]) && numel(joint_velocity)==7 && ...
        all(isfinite(J_camera_base(:))) && all(isfinite(joint_velocity(:)))
    twist=J_camera_base*joint_velocity(:);
    if all(isfinite(twist))
        camera_twist_base=twist;
        camera_motion_valid=true;
    end
end
end
