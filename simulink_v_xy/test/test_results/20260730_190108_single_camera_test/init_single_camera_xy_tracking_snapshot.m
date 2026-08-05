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
Z_hat = 0.75;
rho_hat = 1/Z_hat;

% Proportional + EKF feedforward + leaky adaptive residual controller.
Kpx = 2.5;
Kpy = 2.5;
k_ff = 1.0;
gamma_adapt = 0.30;
sigma_adapt = 0.50;
adapt_max = 0.05;

enable_proportional = true;
enable_ekf_feedforward = true;
enable_adaptation = true;
controller_enable = true;
USE_ROS = true;

% Camera-frame XY command limits.
v_xy_max = 0.70;
a_xy_max = 0.80;

% EKF state order: [X;Y;Vx;Vy;Ax;Ay] in a local fixed XY frame.
P0 = diag([0.05^2, 0.05^2, 0.20^2, 0.20^2, 0.50^2, 0.50^2]);
Q_ekf = diag([1e-6, 1e-6, 1e-4, 1e-4, 1e-2, 1e-2]);
R_ekf = diag([0.003^2, 0.003^2]);
ekf_gate_threshold = 13.82;
ekf_reset_timeout_sec = 0.50;
% 允许连续漏检的视觉帧数。
% 推荐先使用2帧。
target_loss_hold_frames = 3;

% Input watchdogs.
target_timeout_sec = 0.20;
joint_state_timeout_sec = 0.10;

% Offline plant and camera sensor.
Z0 = Z_hat;
X0 = 0.45;
Y0 = -0.25;
pixel_noise_std = 0.8;
random_seed_x = 1207;
random_seed_y = 9053;

% Fixed-size vector consumed by the embedded MATLAB Function controller.
controller_parameters = [ ...
    Ts; Z_hat; Kpx; Kpy; k_ff; gamma_adapt; sigma_adapt; adapt_max; ...
    v_xy_max; a_xy_max; ekf_gate_threshold; ...
    double(enable_proportional); double(enable_ekf_feedforward); ...
    double(enable_adaptation); double(controller_enable); ...
    ekf_reset_timeout_sec];

% Compatibility aliases used only by the existing plotting script.
gamma_x = gamma_adapt;
gamma_y = gamma_adapt;
v_adapt_max = adapt_max;
