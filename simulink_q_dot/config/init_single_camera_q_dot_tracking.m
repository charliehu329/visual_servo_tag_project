%% 单目相机 XY 目标跟踪与 FR3 关节速度直发模型参数
% 本脚本是 single_camera_joint_velocity_direct.slx 唯一需要运行的初始化文件。
% 模型会直接向 JointVelocityExampleController 发布 7 维关节角速度。
%
% 注意：
% 1. T_link8_camera 必须是“相机坐标系到 fr3_link8 坐标系”的齐次变换；
% 2. 首次真机测试应保持低速、工作空间无障碍且急停可触及；
% 3. 先确认关节顺序、图像 XY 方向和零命令，再逐步提高控制参数。


%% 1. 仿真与控制周期

% 模型固定步长和控制周期，单位：s。
Ts = 0.01;

% 仿真结束时间。inf 表示手动停止。
T_end = inf;


%% 2. 相机内参与名义工作深度

% 固定焦距针孔相机内参，焦距和主点单位均为 pixel。
fx = 2057;
fy = 2054;
cx = 1000;
cy = 592;

% 名义工作深度，单位：m。
% 实时有效深度 z 存在时，控制器会优先使用实时 z。
Z_hat = 0.5;
rho_hat = 1.0 / Z_hat;


%% 3. 比例反馈与四状态 KF 速度前馈

% 图像归一化误差的比例反馈增益。
Kpx = 1.0;
Kpy = 1.0;

% KF 目标速度前馈系数。
k_ff = 0.0;

task_velocity_filter_tau = 0.04;

% KF 连续接受达到该帧数后，才允许启用速度前馈。
required_valid_frames = 5;

% 连续这么长时间没有接受到有效测量时，重置 KF，单位：s。
% KF 重置只会关闭速度前馈，不会直接关闭仍然有效的比例反馈。
measurement_reset_timeout = 2.0;

% 控制器使能。
enable_proportional = true;
enable_ekf_feedforward = false;
controller_enable = true;

% 保留与原模型一致的 ROS 模式标志。
USE_ROS = true;


%% 4. 四状态匀速 KF 参数

% KF 状态顺序：
% x_hat = [X_B; Y_B; Vx_B; Vy_B]
% 位置单位：m；速度单位：m/s；状态均在机器人 Base 坐标系中。

P0 = diag([ ...
    0.05^2, ...
    0.05^2, ...
    0.20^2, ...
    0.20^2]);

% 匀速模型的白噪声加速度标准差，单位：m/s^2。
% 数值越大，KF 允许估计速度变化得越快，但输出也会更敏感。
process_accel_std = 1.0;

Q_ekf = process_accel_std^2 * [ ...
    Ts^4/4, 0,        Ts^3/2, 0; ...
    0,        Ts^4/4, 0,        Ts^3/2; ...
    Ts^3/2, 0,        Ts^2,   0; ...
    0,        Ts^3/2, 0,        Ts^2];

% 目标在 Base 坐标系 XY 平面中的位置测量噪声协方差，单位：m^2。
R_ekf = diag([1e-8, 1e-8]);

% 二维测量的 95% 卡方门限。
ekf_gate_threshold = 5.991;


%% 5. 输入超时与目标短时丢失保持

% 视觉消息最大允许年龄，单位：s。
% 同时用于比例项保持最后一次有效误差的最长时间：
% 超过该时间后，比例项目标变为零；KF 前馈仍由
% measurement_reset_timeout 独立管理。
target_timeout_sec = 0.50;

% JointState 最大允许年龄，单位：s。
joint_state_timeout_sec = 0.50;

% 相机运动学反馈最大允许年龄，单位：s。
camera_velocity_feedback_timeout_sec = 0.50;


%% 6. 完整视觉雅可比的阻尼与奇异保护

% 正常区域使用的最小阻尼。
visual_damping_min = 0.015;

% 接近奇异区域时使用的最大阻尼。
visual_damping_max = 0.20;

% 最小奇异值不大于该值时，总关节速度缩放为零。
singularity_sigma_stop = 0.025;

% 最小奇异值不小于该值时，不进行奇异度降速。
% stop 与 warn 之间采用平滑缩放。
singularity_sigma_warn = 0.080;


% sigma_min ≤ singularity_sigma_stop     完全停止
% singularity_sigma_stop～singularity_sigma_warn         平滑降速
% sigma_min ≥ singularity_sigma_warn     正常速度


%% 7. 零空间关节姿态与深度安全区

% 零空间关节中心回避增益。
nullspace_posture_gain = 0.08;

% 目标深度安全区，单位：m。
% 深度处于该区间内时，深度安全任务不主动工作。
depth_safe_min = 0.30;
depth_safe_max = 1.50;

% 目标深度超出安全区时的恢复增益。
depth_safety_gain = 0.02;

% 视觉雅可比允许使用的最小深度，单位：m。
% z 不大于该值时，当前视觉求解不会继续使用该深度。
visual_min_depth = 0.10;


%% 8. 最终 7 维关节速度命令保护


% 接近建议关节位置边界时的整体降速距离，单位：rad。
% 关节位置边界大于 0.15 rad：正常速度；
joint_soft_margin = 0.15;

% 零空间任务使用的名义关节构型，单位：rad。
joint_nominal_configuration = [ ...
    0.0; ...
   -pi/4; ...
    0.0; ...
   -3*pi/4; ...
    0.0; ...
    pi/2; ...
    pi/4];

% FR3 建议的矩形关节位置工作区，单位：rad。
% 这是本模型的软件工作边界，不是机器人的硬件绝对极限。
joint_position_min = [ ...
   -2.3476; ...
   -1.5454; ...
   -2.4937; ...
   -2.7714; ...
   -2.5100; ...
    0.7773; ...
   -2.7045];

joint_position_max = [ ...
    2.3476; ...
    1.5454; ...
    2.4937; ...
   -0.4226; ...
    2.5100; ...
    4.2841; ...
    2.7045];

% 首次真机验证使用的保守软件关节速度上限，单位：rad/s。
% 这不是 FR3 的硬件最大速度。
joint_velocity_limit = [ ...
    0.50; ...
    0.50; ...
    0.50; ...
    0.50; ...
    0.75; ...
    0.75; ...
    0.75];

% 首次真机验证使用的保守软件关节加速度上限，单位：rad/s^2。
joint_acceleration_limit = 1.50 * ones(7,1);


%% 9. 控制器内部 10 维参数顺序说明

% 模型内部由 10 个 Constant 块和 Concatenate 块自动组成 parameters：
%
% parameters(1)  = Ts
% parameters(2)  = Z_hat
% parameters(3)  = Kpx
% parameters(4)  = Kpy
% parameters(5)  = k_ff
% parameters(6)  = ekf_gate_threshold
% parameters(7)  = enable_proportional
% parameters(8)  = enable_ekf_feedforward
% parameters(9)  = controller_enable
% parameters(10) = target_timeout_sec
% parameters(11) = task_velocity_filter_tau
%
% 当前模型不读取工作区变量 controller_parameters，因此这里不再创建
% 一个重复向量，避免以后再次出现 10 维与 12 维接口不一致的问题。



%% 10. 导入 FR3 运动学模型

% 按当前项目目录结构定位 URDF：
% 当前初始化脚本目录/../../config/urdf/fr3.urdf
init_file_directory = fileparts(mfilename('fullpath'));
urdf_path = fullfile( ...
    init_file_directory, ...
    '..', ...
    '..', ...
    'config', ...
    'urdf', ...
    'fr3.urdf');

assert(isfile(urdf_path), ...
    '找不到 FR3 URDF 文件：%s', urdf_path);

fr3_camera_robot = importrobot(urdf_path);
fr3_camera_robot.DataFormat = 'column';

bodyNames = fr3_camera_robot.BodyNames;

% 当前控制模型只使用机械臂七个关节，不使用夹爪手指关节。
if ismember('fr3_leftfinger', bodyNames)
    removeBody(fr3_camera_robot, 'fr3_leftfinger');
end

if ismember('fr3_rightfinger', bodyNames)
    removeBody(fr3_camera_robot, 'fr3_rightfinger');
end

assert(ismember('fr3_link8', fr3_camera_robot.BodyNames), ...
    'URDF 中不存在父坐标系 fr3_link8。');


%% 11. 添加手眼标定得到的相机光学坐标系

% T_link8_camera = ^{link8}T_{camera}
% 含义：把相机光学坐标系中的点转换到 fr3_link8 坐标系：
%
% p_link8 = T_link8_camera * p_camera
%
% 该方向与下面“父节点 fr3_link8、子节点 camera_link”的
% setFixedTransform 用法一致，不需要取逆。
T_link8_camera = [ ...
     0.685367986922,  0.727986036940,  0.017522914211, -0.0495; ...
    -0.727707409066,  0.683826802626,  0.053130319028,  0.0191; ...
     0.026695491993, -0.049165374297,  0.998433831897,  0.1396; ...
     0.0,             0.0,             0.0,             1.0];

% 对手眼标定矩阵做基本合法性检查。
R_link8_camera = T_link8_camera(1:3,1:3);
assert(all(isfinite(T_link8_camera(:))) && ...
    norm(R_link8_camera' * R_link8_camera - eye(3), 'fro') < 1e-3 && ...
    abs(det(R_link8_camera) - 1.0) < 1e-3 && ...
    norm(T_link8_camera(4,:) - [0 0 0 1]) < 1e-9, ...
    'T_link8_camera 不是有效的刚体齐次变换矩阵。');

% 防止同一 rigidBodyTree 中重复添加同名相机坐标系。
assert(~ismember('camera_link', fr3_camera_robot.BodyNames), ...
    'URDF 中已经存在 camera_link，请检查是否重复添加相机坐标系。');

camera_body = rigidBody('camera_link');
camera_joint = rigidBodyJoint('camera_fixed_joint', 'fixed');

setFixedTransform(camera_joint, T_link8_camera);
camera_body.Joint = camera_joint;

addBody( ...
    fr3_camera_robot, ...
    camera_body, ...
    'fr3_link8');


%% 12. 初始化结果检查

assert(Ts > 0.0, 'Ts 必须大于 0。');
assert(Z_hat > visual_min_depth, ...
    'Z_hat 必须大于 visual_min_depth。');
assert(depth_safe_min > visual_min_depth && ...
    depth_safe_max > depth_safe_min, ...
    '深度安全区参数不合法。');
assert(singularity_sigma_warn > singularity_sigma_stop && ...
    singularity_sigma_stop >= 0.0, ...
    '奇异值阈值参数不合法。');
assert(all(joint_position_max > joint_position_min), ...
    '关节位置上下限不合法。');
assert(all(joint_nominal_configuration > joint_position_min) && ...
    all(joint_nominal_configuration < joint_position_max), ...
    'joint_nominal_configuration 必须位于软件关节位置范围内。');
assert(all(joint_velocity_limit > 0.0) && ...
    all(joint_acceleration_limit > 0.0), ...
    '关节速度和加速度上限必须大于 0。');

