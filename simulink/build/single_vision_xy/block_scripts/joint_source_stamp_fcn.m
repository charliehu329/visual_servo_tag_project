function [joint_source_stamp,joint_source_stamp_valid] = fcn(sec,nsec)
%#codegen
% 将 JointState Header 的 sec/nanosec 校验并转换为 ROS 源时间。
joint_source_stamp=0;
joint_source_stamp_valid=0;
sec=double(sec);
nsec=double(nsec);
ok=isfinite(sec)&&sec>=0&&sec==floor(sec)&& ...
    isfinite(nsec)&&nsec>=0&&nsec<1e9&&nsec==floor(nsec);
if ~ok
    return
end
joint_source_stamp=sec+1e-9*nsec;
joint_source_stamp_valid=double(isfinite(joint_source_stamp)&&joint_source_stamp>0);
end
