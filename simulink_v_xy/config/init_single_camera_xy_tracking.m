%% Single-camera XY target tracking parameters
% This is the only initialization file required by the delivered model.
% Values marked as placeholders must be replaced with measured calibration
% data before commanding hardware.

Ts = 0.01;
T_end = inf;


%% Camera calibration and fixed working depth

% Fixed-focus pinhole camera calibration.
fx = 2057;
fy = 2054;
cx = 1000;
cy = 592;

% Fixed working-plane depth.
Z_hat = 0.5;
rho_hat = 1/Z_hat;


%% Proportional feedback and KF velocity feedforward

Kpx = 1.0;
Kpy = 1.0;
k_ff = 0.5;

% The KF must accept this many consecutive measurements before v_ff is enabled.
required_valid_frames = 5;

% Reset the KF when no measurement has been accepted for this duration.
% KF reset only disables v_ff; it does not directly disable a valid v_p.
measurement_reset_timeout = 2.0;

% First-order low-pass filter time constant for v_ff.
% Set to 0 to disable the v_ff low-pass filter.
ff_filter_tau = 0.20;

enable_proportional = true;
enable_ekf_feedforward = true;
controller_enable = true;
USE_ROS = true;


%% Camera-frame XY command limits

v_xy_max = 2.0;
a_xy_max = 1.0;


%% Four-state KF parameters

% KF state order:
% x_hat = [X_B; Y_B; Vx_B; Vy_B]

P0 = diag([ ...
    0.05^2, ...
    0.05^2, ...
    0.20^2, ...
    0.20^2]);

% Constant-velocity model with white acceleration process noise.
% process_accel_std controls how quickly the KF velocity is allowed to change.
process_accel_std = 1.0;

Q_ekf = process_accel_std^2 * [ ...
    Ts^4/4, 0,        Ts^3/2, 0; ...
    0,        Ts^4/4, 0,        Ts^3/2; ...
    Ts^3/2, 0,        Ts^2,   0; ...
    0,        Ts^3/2, 0,        Ts^2];

R_ekf = diag([1e-8, 1e-8]);

% 95% chi-square threshold for a two-dimensional measurement.
ekf_gate_threshold = 5.991;


%% Input watchdogs

% safe_valid continues to include the target timeout check.
target_timeout_sec = 0.5;

% Joint state must also remain fresh.
joint_state_timeout_sec = 0.50;


%% Offline plant and camera sensor

Z0 = Z_hat;
X0 = 0.45;
Y0 = -0.25;

pixel_noise_std = 0.8;
random_seed_x = 1207;
random_seed_y = 9053;


%% Fixed-size vector consumed by the MATLAB Function controller

% parameters(1)  = Ts
% parameters(2)  = Z_hat
% parameters(3)  = Kpx
% parameters(4)  = Kpy
% parameters(5)  = k_ff
% parameters(6)  = NIS gate
% parameters(7)  = enable_p
% parameters(8)  = enable_ff
% parameters(9)  = controller_enable
% parameters(10) = reserved
% parameters(11) = v_xy_max
% parameters(12) = a_xy_max

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
    0.0;
    v_xy_max;
    a_xy_max];


%% FR3 camera-velocity feedback kinematic model

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

% T_link8_camera:
% Homogeneous transform from camera coordinates to fr3_link8 coordinates.
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