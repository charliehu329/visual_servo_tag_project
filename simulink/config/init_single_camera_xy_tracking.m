%% Single-camera XY target tracking parameters
% This is the only initialization file required by the delivered model.
% Values marked as placeholders must be replaced with measured calibration
% data before commanding hardware.

Ts = 0.01;
T_end = inf;


% Fixed-focus pinhole camera calibration (placeholder values).
fx = 2057;
fy = 2054;
cx = 1000;
cy = 592;

% Fixed working-plane depth.
Z_hat = 0.5;
rho_hat = 1/Z_hat;

% Proportional + EKF feedforward + leaky adaptive residual controller.
Kpx = 1.0;
Kpy = 1.0;
k_ff = 0.5;

required_valid_frames = 5;
measurement_reset_timeout = 2.0;
ff_ramp_time = 0.20;

enable_proportional = true;
enable_ekf_feedforward = true;
controller_enable = true;
USE_ROS = true;

% Camera-frame XY command limits.
v_xy_max = 2.0;
a_xy_max = 2.0;

% EKF state order: [X;Y;Vx;Vy;Ax;Ay] in a local fixed XY frame.
P0 = diag([0.05^2, 0.05^2, 0.20^2, 0.20^2, 0.50^2, 0.50^2]);
Q_ekf = diag([1e-12, 1e-12, 1e-12, 1e-12, 1e-4, 1e-4]);
R_ekf = diag([1e-8, 1e-8]);
ekf_gate_threshold = 5.991;
ekf_reset_timeout_sec = 0.50;


% Input watchdogs.
target_timeout_sec = 1.0;
joint_state_timeout_sec = 0.50;

% Offline plant and camera sensor.
Z0 = Z_hat;
X0 = 0.45;
Y0 = -0.25;
pixel_noise_std = 0.8;
random_seed_x = 1207;
random_seed_y = 9053;

% Fixed-size vector consumed by the embedded MATLAB Function controller.
controller_parameters = [ ...
    Ts;
    Z_hat;
    Kpx;
    Kpy;
    k_ff;
    ekf_gate_threshold;
    double(enable_proportional);
    double(enable_ekf_feedforward);
    double(controller_enable);
    ekf_reset_timeout_sec];



%% FR3相机速度反馈运动学模型

urdf_path = fullfile( ...
    fileparts(mfilename('fullpath')), ...
    '..', ...
    '..', ...
    'config', ...
    'urdf', ...
    'fr3.urdf');

fr3_camera_robot = importrobot(urdf_path);
fr3_camera_robot.DataFormat = 'column';
bodyNames = fr3_camera_robot.BodyNames;

if ismember('fr3_leftfinger',bodyNames)
    removeBody(fr3_camera_robot,'fr3_leftfinger');
end

if ismember('fr3_rightfinger',bodyNames)
    removeBody(fr3_camera_robot,'fr3_rightfinger');
end
% T_link8_camera：
% 从相机坐标系到fr3_link8坐标系的齐次变换。
T_link8_camera = [ ...
     0.685367986922,  0.727986036940,  0.017522914211, -0.0495; ...
    -0.727707409066,  0.683826802626,  0.053130319028,  0.0191; ...
     0.026695491993, -0.049165374297,  0.998433831897,  0.1396; ...
     0.0,             0.0,             0.0,             1.0];

camera_body = rigidBody('camera_link');

camera_joint = rigidBodyJoint( ...
    'camera_fixed_joint', ...
    'fixed');

setFixedTransform( ...
    camera_joint, ...
    T_link8_camera);

camera_body.Joint = camera_joint;

addBody( ...
    fr3_camera_robot, ...
    camera_body, ...
    'fr3_link8');

camera_velocity_feedback_timeout_sec = 0.5;
