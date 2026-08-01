%% 单目 XY 四状态匀速 Kalman Filter 初始化
% 状态：[X_C;Y_C;V_tx,C;V_ty,C]；位置单位 m，速度单位 m/s。
Ts=.01; T_end=inf;
fx=2057; fy=2054; cx=1000; cy=592; Z_hat=1; rho_hat=1/Z_hat;
Kpx=1; Kpy=1; k_ff=.5;
enable_proportional=true; enable_ekf_feedforward=true;
controller_enable=true; USE_ROS=true;
v_xy_max=2; a_xy_max=2;

% 四状态初始协方差：位置 m^2，速度 (m/s)^2。
P0=diag([.05 .05 .20 .20].^2);
% 白噪声加速度标准差，单位 m/s^2。
sigma_acc_cv=.10;
Q_ekf=sigma_acc_cv^2*[ ...
    Ts^4/4 0 Ts^3/2 0; ...
    0 Ts^4/4 0 Ts^3/2; ...
    Ts^3/2 0 Ts^2 0; ...
    0 Ts^3/2 0 Ts^2];
% 相机系二维位置测量噪声，单位 m^2；沿用原模型数值。
R_ekf=diag([1e-8 1e-8]);
ekf_gate_threshold=5.991;
ekf_reset_timeout_sec=.5; % 兼容原参数向量，第10项保留。
required_valid_frames=5;
measurement_reset_timeout=.5; % s，最近有效视觉测量超时复位。
ff_ramp_time=.5;              % s，速度前馈渐入时间。

target_timeout_sec=.20;       % s，视觉目标消息超时。
joint_state_timeout_sec=.10;  % s，关节状态消息超时。
command_velocity_timeout_sec=.10; % s，最终关节命令速度超时。

assert(Ts>0 && sigma_acc_cv>=0,'Ts和sigma_acc_cv必须合法');
assert(command_velocity_timeout_sec>=Ts,'命令速度超时必须不小于Ts');
assert(isequal(size(P0),[4 4]) && isequal(size(Q_ekf),[4 4]));
assert(isequal(size(R_ekf),[2 2]));

% 离线兼容参数。
Z0=Z_hat; X0=.45; Y0=-.25; pixel_noise_std=.8;
random_seed_x=1207; random_seed_y=9053;

%% FR3 URDF 与 camera_link 固定外参
urdf_path=fullfile(fileparts(mfilename('fullpath')),'fr3.urdf');
assert(isfile(urdf_path),'找不到 URDF: %s',urdf_path);
fr3_camera_robot=importrobot(urdf_path);
fr3_camera_robot.DataFormat='column';
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

