%% 单目五自由度相机跟踪（仅 vz 固定为 0）
Ts=.01; T_end=inf;
fx=2057; fy=2054; cx=1000; cy=592; Z_hat=1; rho_hat=1/Z_hat;
Kpx=1; Kpy=1; k_ff=.5;
gamma_adapt=0; sigma_adapt=0; adapt_max=0;
enable_proportional=true; enable_ekf_feedforward=true;
enable_adaptation=false; controller_enable=true;
if ~exist('USE_ROS','var'), USE_ROS=true; end
v_xy_max=2; a_xy_max=2;

% EKF: [X Y Vx Vy Ax Ay]'
P0=diag([.05 .05 .2 .2 .5 .5].^2);
Q_ekf=diag([1e-12 1e-12 1e-12 1e-12 1e-4 1e-4]);
R_ekf=diag([1e-8 1e-8]);
ekf_gate_threshold=5.991; ekf_reset_timeout_sec=.5;
required_valid_frames=5; measurement_reset_timeout=.5; ff_ramp_time=.5;
target_timeout_sec=.2; joint_state_timeout_sec=.1;

Z0=Z_hat; X0=.45; Y0=-.25; pixel_noise_std=.8;
random_seed_x=1207; random_seed_y=9053;
controller_parameters=[Ts;Z_hat;Kpx;Kpy;k_ff;gamma_adapt;sigma_adapt; ...
    adapt_max;v_xy_max;a_xy_max;ekf_gate_threshold; ...
    enable_proportional;enable_ekf_feedforward;enable_adaptation; ...
    controller_enable;ekf_reset_timeout_sec];
gamma_x=gamma_adapt; gamma_y=gamma_adapt; v_adapt_max=adapt_max;

%% FR3 URDF + 相机外参
urdf_path=fullfile(fileparts(mfilename('fullpath')),'fr3.urdf');
assert(isfile(urdf_path),'找不到 URDF: %s',urdf_path);
fr3_camera_robot=importrobot(urdf_path); fr3_camera_robot.DataFormat='column';
for body=["fr3_leftfinger","fr3_rightfinger"]
    if any(strcmp(fr3_camera_robot.BodyNames,body)), removeBody(fr3_camera_robot,body); end
end
T_link8_camera=[ ...
 .685367986922  .727986036940  .017522914211 -.0495; ...
-.727707409066  .683826802626  .053130319028  .0191; ...
 .026695491993 -.049165374297  .998433831897  .1396; ...
 0 0 0 1];
camera_body=rigidBody('camera_link');
camera_body.Joint=rigidBodyJoint('camera_fixed_joint','fixed');
setFixedTransform(camera_body.Joint,T_link8_camera);
addBody(fr3_camera_robot,camera_body,'fr3_link8');
camera_velocity_feedback_timeout_sec=.5;
