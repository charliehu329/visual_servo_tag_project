%% 单目 XY Base 系四状态 CV KF
Ts=.01;
T_end=inf;
fx=2057; fy=2054; cx=1000; cy=592; %#ok<NASGU>
Z_hat=1; rho_hat=1/Z_hat; %#ok<NASGU>
Kpx=1; Kpy=1; k_ff=.5;
enable_proportional=true;
enable_ekf_feedforward=true;
controller_enable=true;
USE_ROS=true;
v_xy_max=2;
a_xy_max=2;

P0=diag([.05 .05 .20 .20].^2);
sigma_acc_cv=.10;
Q_kf=sigma_acc_cv^2*[ ...
    Ts^4/4 0 Ts^3/2 0; ...
    0 Ts^4/4 0 Ts^3/2; ...
    Ts^3/2 0 Ts^2 0; ...
    0 Ts^3/2 0 Ts^2];

sigma_position_meas=.005;
sigma_velocity_meas=.10;
R_position=diag([sigma_position_meas sigma_position_meas].^2);
R_velocity=diag([sigma_velocity_meas sigma_velocity_meas].^2);
nis_position_threshold=5.991;
nis_velocity_threshold=5.991;

required_valid_frames=5;
measurement_reset_timeout=.5;
ff_ramp_time=.5;
target_timeout_sec=.20;
joint_state_timeout_sec=.10;
command_velocity_timeout_sec=.10; %#ok<NASGU>

joint_buffer_capacity=512;
pending_visual_capacity=16;
sync_max_bracket_span_sec=.02;
sync_max_visual_wait_sec=.10;
sync_clock_reset_threshold_sec=.50;
sync_allow_extrapolation=false;
sync_required_for_control=true;
allow_legacy_untimestamped=false;

assert(Ts>0 && sigma_acc_cv>=0 && Z_hat>0)
assert(isequal(size(P0),[4 4]))
assert(isequal(size(Q_kf),[4 4]))
assert(isequal(size(R_position),[2 2]))
assert(isequal(size(R_velocity),[2 2]))
assert(norm(P0-P0','fro')<1e-10)
assert(norm(Q_kf-Q_kf','fro')<1e-10)
assert(norm(R_position-R_position','fro')<1e-10)
assert(norm(R_velocity-R_velocity','fro')<1e-10)
assert(min(eig(P0))>=-1e-12)
assert(min(eig(Q_kf))>=-1e-12)
assert(min(eig(R_position))>0)
assert(min(eig(R_velocity))>0)
assert(all(diag(R_velocity)>diag(R_position)))
assert(nis_position_threshold>0)
assert(nis_velocity_threshold>0)
assert(measurement_reset_timeout>0)
assert(joint_state_timeout_sec>0)
assert(required_valid_frames>=1)
assert(joint_buffer_capacity>=4 && joint_buffer_capacity<=512)
assert(pending_visual_capacity>=1 && pending_visual_capacity<=16)
assert(sync_max_bracket_span_sec>0)
assert(sync_max_visual_wait_sec>0)
assert(sync_clock_reset_threshold_sec>0)
assert(sync_allow_extrapolation==false)
assert(sync_required_for_control==true)
assert(allow_legacy_untimestamped==false)

Z0=Z_hat; X0=.45; Y0=-.25; pixel_noise_std=.8; %#ok<NASGU>
random_seed_x=1207; random_seed_y=9053; %#ok<NASGU>

%% FR3 URDF 与 camera_link 固定外参
urdf_path=fullfile(fileparts(mfilename('fullpath')),'fr3.urdf');
assert(isfile(urdf_path),'URDF not found: %s',urdf_path)
fr3_camera_robot=importrobot(urdf_path);
fr3_camera_robot.DataFormat='column';
for body=["fr3_leftfinger","fr3_rightfinger"]
    if any(strcmp(fr3_camera_robot.BodyNames,body))
        removeBody(fr3_camera_robot,body);
    end
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
