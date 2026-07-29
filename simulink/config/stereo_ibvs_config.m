%% Stereo IBVS V2 deployment configuration
% 配置作用：
% 将已验证的 arm_stereo_ibvs_ekf_v1（V2）算法迁移到
% stereo_ibvs_core，同时保留 ROS 2 包装和 Python 安全转发所需参数。
%
% 统一接口：
% 1. 焦距反馈 Topic 直接发布 [fL_mm; fR_mm]；
% 2. Core 内仅在 02_Camera_and_Zoom_Model 中执行 mm -> pixel 焦距换算；
% 3. Zoom 命令直接使用 [fDotL_mm_s; fDotR_mm_s]，不再使用 step/s；
% 4. 启动后收到第一帧有效焦距前，cameraModelValid 必须为 false；
% 5. 不包含目标轨迹、关节积分、相机投影、真值和噪声注入等仿真对象。

%% 0. 日常功能开关
% 日常运行只需要修改本节的四个开关，其他算法逻辑由 Core 固定管理：
% 1. Arm 开启后，只要左相机目标有效，就执行左相机中心主任务；
% 2. Depth 开启后，在中心主任务之后加入双目逆深度次任务；
% 3. Zoom 开启后，同时启用 Zoom 控制和对应的优先级调度；
% 4. Nullspace 开启后，在视觉任务零空间内执行关节中位回中。
%
% 四个功能仍分别受输入有效性、ROS 2 watchdog、标定许可和 09 安全
% 模块约束。修改本节只能选择功能，不能绕过任何真机安全锁。

% 机械臂控制总开关：
% 1 = 允许Core生成机械臂关节速度；0 = 机械臂速度最终归零。
% 真正输出还要求 controllerEnableSafe、左目标、相机模型、运动学和
% Arm标定许可同时有效。本开关不能绕过09安全模块。
armControlEnable = false;

% 双目逆深度任务开关：
% 1 = 在中心主任务之后加入逆深度次任务；
% 0 = 不使用双目深度控制。
% 开启后还需要 validLeft、validRight、validStereoQualified、
% ekfPredictionValid 和 Depth标定许可有效。只有左相机时建议设为0。
depthTaskEnable = false;

% 关节中位零空间任务开关：
% 1 = 在不破坏高优先级视觉任务的零空间内，将关节拉向cfg.qMid；
% 0 = 不执行关节中位姿态优化。
% 开启后由cfg.kNull决定回中强度；cfg.kNull必须大于0。
% 当前cfg.kNull=0.05，因此本开关为1时零空间任务会实际生效。
nullspaceEnable = false;

% Zoom控制总开关：
% 1 = 允许生成左右镜头焦距速度命令；0 = Zoom命令最终归零。
% 开启后还需要实时焦距有效且新鲜、Zoom标定许可有效，并确认底层
% 接口正确接收mm/s、正负方向正确、限速正确且停止可靠。
zoomControlEnable = false;

%% 0.1 标定状态与安全许可
% 本节集中显示当前项目还缺少哪些标定或接口验证，方便启动前检查。
% 这些变量不是日常功能开关，而是"对应数据已经真实测量、写入配置，
% 并完成方向、单位和安全验证"的声明。
%
% 必须先完成对应工作，再把变量改为 true。不能仅为了得到非零速度
% 而打开，否则配置中的占位参数可能直接用于真机控制。
%
% Arm 控制许可依赖：
%   cameraMountCalibrated
%   cameraIntrinsicsCalibrated
%   pixelPitchCalibrated
% Depth 控制在 Arm 许可基础上还依赖：
%   stereoCalibrationValid
% Zoom 控制在 Depth 许可基础上还依赖：
%   focalRateCommandInterfaceValidated
% 最终许可关系在第 15 节统一计算。

% 左相机相对fr3_link8的手眼变换已经标定并写入cfg.T_link8_CL。
cameraMountCalibrated = true;

% 左右相机外参和baseline已经完成双目标定并写入配置。
stereoCalibrationValid = false;

% 等效输出像元尺寸已经确认并写入outputPixelPitchXmm/Ymm。
pixelPitchCalibrated = false;

% 主点、畸变和相机内参已经完成正式标定。
cameraIntrinsicsCalibrated = false;

% Zoom底层接口已经验证mm/s单位、正负方向、限速和可靠停止。
focalRateCommandInterfaceValidated = false;

%% 1. 路径、配置结构体与版本
% 本节只负责定位工程目录并创建唯一的配置结构体cfg。
% 后续Simulink Constant块统一读取cfg.xxx，不生成cfg_xxx散变量。
projectDir = fileparts(fileparts(mfilename('fullpath')));
repoDir = fileparts(projectDir);

cfg = struct();
cfg.configurationName = 'v2_full_deployment_mm_interface';
cfg.configurationVersion = 3;
cfg.stage = 5;

%% 2. 常用调参区
% =========================================================================
% 日常调试时，优先只看本节。
%
% 推荐调参顺序：
% 1. 先确认第0节功能开关和第0.1节标定许可；
% 2. 再确认控制周期、期望深度和期望尺度；
% 3. 先调中心任务Kc，再调深度任务kRho；
% 4. 使用Zoom时，再调Kf和优先级调度阈值；
% 5. 最后根据真机响应逐步放宽速度和加速度限制。
%
% 除非正在排查特定底层问题，否则不建议修改第3节之后的参数。
% =========================================================================

%% 2.1 控制周期
% Core控制频率。修改后cfg.Ts、ROS超时帧数等派生量会自动更新。
% 一般应与视觉数据的稳定更新频率一致；当前按60 Hz运行。
cfg.cameraFps = 60;
cfg.visionRateHz = cfg.cameraFps;
cfg.controlRateHz = 60;
cfg.Ts = 1 / cfg.controlRateHz;

% Simulink按Core周期发布，Python安全层以120 Hz保持并转发最新命令。
% pythonSafetyRateHz通常不作为控制器调参项，仅需与转发节点配置一致。
cfg.simulinkPublishRateHz = cfg.controlRateHz;
cfg.pythonSafetyRateHz = 120;

%% 2.2 视觉任务目标
% 归一化图像中心期望值[xd; yd]。
% [0;0]表示将目标中心移动到相机主点。
cfg.centerDesired = [0; 0];

% 目标工作深度，单位m；rhoD是由Zd自动计算的期望逆深度，单位1/m。
% targetDepthMin/Max是允许的目标深度范围，不是期望值。
cfg.targetDepthMin = 0.50;
cfg.targetDepthMax = 1.00;
cfg.Zd = 0.60;
cfg.rhoD = 1 / cfg.Zd;

% AprilTag尺度定义为"四角面积的平方根"，单位pixel。
% scaleDesired = [leftDesired; rightDesired]。
% 当前值是临时期望值，应根据实际工作距离和成像效果确定。
cfg.targetCharacteristicSize = 0.10;
cfg.scaleDesired = [700; 700];

%% 2.3 Arm中心任务、Depth任务与零空间任务
% Kc：左相机二维中心误差比例增益。
% 增大后回中更快，但过大会引起抖动、超调或频繁触发限速。
cfg.Kc = diag([2.5, 2.5]);

% kRho：双目逆深度次任务增益。
% 仅在depthTaskEnable=true且双目数据与标定许可有效时生效。
cfg.kRho = 1.5;

% lambdaC/lambdaRho：阻尼伪逆系数。
% 增大可提高接近奇异位置时的稳定性，但会降低任务跟踪能力。
cfg.lambdaC = 0.02;
cfg.lambdaRho = 0.02;

% betaC/betaRho：鲁棒补偿强度；设为0表示关闭对应补偿。
% epsilonC/epsilonRho：鲁棒项平滑参数，避免误差接近0时不连续。
cfg.betaC = 0;
cfg.betaRho = 0;
cfg.epsilonC = 1e-3;
cfg.epsilonRho = 1e-3;

% kNull：关节中位零空间回中强度。
% 仅在nullspaceEnable=true时生效；应明显小于主视觉任务增益。
cfg.kNull = 0.05;

% 将第0节的日常开关写入cfg，供Core统一读取。
cfg.armControlEnable = armControlEnable;
cfg.depthTaskEnable = depthTaskEnable;
cfg.nullspaceEnable = nullspaceEnable;

%% 2.4 Zoom控制器
% Kf：左右镜头尺度误差控制增益。
% 增大后变焦响应更快，但可能造成尺度振荡或频繁切换调度模式。
cfg.Kf = diag([1.5, 1.5]);

% betaF：Zoom鲁棒补偿强度；[0;0]表示左右镜头均关闭。
% epsilonF：对应平滑参数。
cfg.betaF = [0; 0];
cfg.epsilonF = [1e-3; 1e-3];
cfg.zoomControlEnable = zoomControlEnable;

%% 2.5 Zoom优先级调度
% 尺度误差进入阈值必须大于退出阈值，以形成迟滞并避免模式抖动。
cfg.scaleErrorEnterThreshold = 0.04;
cfg.scaleErrorExitThreshold = 0.015;

% 尺度进入稳定区后需要保持的时间，单位s。
cfg.scaleSettledHoldTime = 0.25;

% 连续检测到扰动后才确认切换的时间，单位s。
cfg.disturbanceConfirmTime = 0.10;

% 仅执行Zoom任务的最长持续时间，单位s。
cfg.zoomOnlyMaxTime = 1.00;

% Arm深度任务从0平滑恢复到完整权重所需时间，单位s。
cfg.armDepthRampTime = 0.50;

% 接近焦距工作边界时预留的比例，避免命令长期顶在限位上。
cfg.zoomLimitMarginFraction = 0.02;

%% 2.6 焦距工作范围与Zoom速度
% 硬件极限：镜头理论允许范围，不能由控制器突破，单位mm。
cfg.focalLengthHardwareMinMm = [5; 5];
cfg.focalLengthHardwareMaxMm = [99; 99];

% 控制器工作范围：应严格位于硬件极限内部，单位mm。
% 日常实验优先修改工作范围，不建议修改硬件极限。
cfg.focalLengthWorkingMinMm = [10; 10];
cfg.focalLengthWorkingMaxMm = [90; 90];

% 已验证可持续达到的速度与绝对速度上限，单位mm/s。
cfg.focalRateGuaranteedMmPerSec = 15.5;
cfg.focalRateAbsoluteMaxMmPerSec = 18.75;
cfg.focalRateUnit = 'mm/s';

% 设计速度利用系数。实际设计速度=etaZoom×保证速度。
% 初次真机测试应从更小值开始逐步增加。
cfg.etaZoom = 0.60;
cfg.focalRateDesignMmPerSec = ...
    cfg.etaZoom * cfg.focalRateGuaranteedMmPerSec;
cfg.rightReacquireZoomRateMmPerSec = ...
    cfg.focalRateDesignMmPerSec;

%% 2.7 Arm算法层限速
% qDotAlgorithmMax和qDDotAlgorithmMax的实际cfg赋值位于FR3 URDF读取后，
% 因为还需要与URDF硬件上限比较。日常调节的是下面两个标量。
%
% 关节速度上限，单位rad/s。09安全模块会对七维命令整组同比例缩放。
qDotAlgorithmLimitRadPerSec = 0.03;

% 关节加速度上限，单位rad/s^2，用于限制相邻周期的速度变化。
qDDotAlgorithmLimitRadPerSec2 = 0.20;

%% 3. ROS 2接口与Topic
% 本节是通信接口配置。只有Topic、消息类型或ROS节点接口变化时才修改。
cfg.jointStateTopic = '/franka/joint_states';
cfg.jointStateMessageType = 'sensor_msgs/JointState';
cfg.expectedJointNames = {
    'fr3_joint1'
    'fr3_joint2'
    'fr3_joint3'
    'fr3_joint4'
    'fr3_joint5'
    'fr3_joint6'
    'fr3_joint7'
};

cfg.stereoFeatureTopic = '/vision_double/stereo_features';
cfg.stereoFeatureMessageType = ...
    'velocity_servo_tag_interfaces/StereoFeatures';
cfg.stereoMaxPairSkewSec = 0.05;

cfg.resetTopic = '/simulink/reset';
cfg.resetMessageType = 'std_msgs/Bool';

cfg.jointVelocityCommandTopic = ...
    '/simulink/target_joints_velocities';
cfg.jointVelocityCommandMessageType = ...
    'std_msgs/Float64MultiArray';

cfg.controllerStatusTopic = '/simulink/controller_status';
cfg.controllerStatusMessageType = ...
    'std_msgs/Float64MultiArray';

% 焦距反馈：
% data = [leftFocalLengthMm; rightFocalLengthMm]
cfg.focalLengthTopic = '/stereo/focal_length';
cfg.focalLengthMessageType = 'std_msgs/Float64MultiArray';
cfg.focalLengthMessageLength = 2;
cfg.focalLengthRateHz = 60;
cfg.focalLengthInputUnit = 'mm';

% 连续 5 个 Core 周期没有新焦距消息时：
% 保持最近一次有效焦距，但 focalLengthFresh=false，Zoom 命令归零。
cfg.focalLengthTimeoutFrames = 5;
cfg.focalLengthTimeoutSec = ...
    cfg.focalLengthTimeoutFrames / cfg.controlRateHz;

% Zoom 命令：
% data = [leftFocalRateMmPerSec; rightFocalRateMmPerSec]
cfg.focalRateCommandTopic = '/simulink/focal_rate_cmd';
cfg.focalRateCommandMessageType = 'std_msgs/Float64MultiArray';
cfg.focalRateMessageLength = 2;
cfg.focalRateCommandUnit = 'mm/s';

% 状态记忆块的安全初值。该值不是有效焦距，不能使 cameraModelValid=true。
cfg.focalLengthStateInitialMm = zeros(2,1);

%% 4. FR3 URDF 与机器人模型
cfg.urdfPath = fullfile(repoDir,'velocity_servo_tag', 'config', 'urdf', 'fr3.urdf');

if ~isfile(cfg.urdfPath)
    error('StereoIBVS:URDFNotFound', ...
        '找不到 FR3 URDF 文件：%s', cfg.urdfPath);
end

fr3 = parseFr3UrdfForController(cfg.urdfPath);

cfg.fr3RobotName = fr3.robotName;
cfg.fr3OriginXYZ = fr3.originXYZ;
cfg.fr3OriginRPY = fr3.originRPY;
cfg.fr3Axis = fr3.axis;
cfg.fr3Joint8OriginXYZ = fr3.joint8OriginXYZ;
cfg.fr3Joint8OriginRPY = fr3.joint8OriginRPY;
cfg.fr3QMinURDF = fr3.qMin;
cfg.fr3QMaxURDF = fr3.qMax;
cfg.fr3QDotMaxURDF = fr3.qDotMax;

cfg.robot = importrobot(cfg.urdfPath);
cfg.robot.DataFormat = 'column';
cfg.robot.Gravity = [0 0 -9.81];

if any(strcmp(cfg.robot.BodyNames, 'fr3_leftfinger'))
    removeBody(cfg.robot, 'fr3_leftfinger');
end

if any(strcmp(cfg.robot.BodyNames, 'fr3_rightfinger'))
    removeBody(cfg.robot, 'fr3_rightfinger');
end

if numel(homeConfiguration(cfg.robot)) ~= 7
    error('StereoIBVS:RobotDOFMismatch', ...
        '移除夹爪手指分支后，FR3 机器人模型仍不是 7 自由度。');
end

cfg.robotBaseName = cfg.robot.BaseName;
cfg.cameraBodyName = 'left_camera_optical';
cfg.cameraParentBodyName = 'fr3_link8';

cfg.jointNames = {
    'fr3_joint1'
    'fr3_joint2'
    'fr3_joint3'
    'fr3_joint4'
    'fr3_joint5'
    'fr3_joint6'
    'fr3_joint7'
};

cfg.q0 = [
    0
    -pi/4
    0
    -3*pi/4
    0
    pi/2
    pi/4
];

cfg.qMin = fr3.qMin;
cfg.qMax = fr3.qMax;
cfg.qMid = 0.5 * (cfg.qMin + cfg.qMax);
cfg.qDotMax = fr3.qDotMax;
cfg.jointTorqueMax = fr3.jointTorqueMax;

% 当前工程兼容别名。
cfg.qInitial = cfg.q0;

if any(cfg.q0 <= cfg.qMin) || any(cfg.q0 >= cfg.qMax)
    error('StereoIBVS:InvalidInitialConfiguration', ...
        'cfg.q0 中至少有一个关节不在 URDF 关节限位内。');
end

%% 5. 世界、左相机与右相机安装关系
% T_A_B 表示 B 坐标系相对于 A 坐标系的位姿。
% 是机器人基座坐标系 {B} 到世界坐标系 {W} 的齐次变换：
cfg.T_W_B = eye(4);

% 左相机相对于 fr3_link8 的安装变换。
% 当前为占位值，手眼标定后替换。

cfg.T_link8_CL = [ ...
     0.685367986922,  0.727986036940,  0.017522914211, -0.0495; ...
    -0.727707409066,  0.683826802626,  0.053130319028,  0.0191; ...
     0.026695491993, -0.049165374297,  0.998433831897,  0.1396; ...
     0,               0,               0,               1       ...
];
cfg.T_CL2L8 = cfg.T_link8_CL;
cfg.cameraMountCalibrated = cameraMountCalibrated;
cfg.cameraMountIsPlaceholder = ~cfg.cameraMountCalibrated;

% 双目外参。V2 使用校正后的平行双目模型。
cfg.baseline = 0.12;
cfg.B = cfg.baseline;
cfg.stereoBaseline = cfg.baseline;
cfg.R_CL_CR = eye(3);
cfg.p_CL_CR = [cfg.baseline; 0; 0];
cfg.T_CL_CR = [
    cfg.R_CL_CR, cfg.p_CL_CR
    0 0 0 1
];
cfg.stereoCalibrationValid = stereoCalibrationValid;

cameraBody = rigidBody(cfg.cameraBodyName);
cameraJoint = rigidBodyJoint('left_camera_fixed_joint', 'fixed');
setFixedTransform(cameraJoint, cfg.T_link8_CL);
cameraBody.Joint = cameraJoint;
addBody(cfg.robot, cameraBody, cfg.cameraParentBodyName);

%% 6. 图像尺寸、主点与实时焦距换算
cfg.imageWidthPx = 1920;
cfg.imageHeightPx = 1080;

% 兼容 V2 旧命名。
cfg.imageWidth = cfg.imageWidthPx;
cfg.imageHeight = cfg.imageHeightPx;

% 暂时使用图像中心，正式相机标定后替换。
cfg.cxL = cfg.imageWidthPx / 2;
cfg.cyL = cfg.imageHeightPx / 2;
cfg.cxR = cfg.imageWidthPx / 2;
cfg.cyR = cfg.imageHeightPx / 2;

% 等效输出像元尺寸，单位 mm/pixel。
% 当前 2.90e-3 为占位值，确认实际值后替换并设为 true。
cfg.outputPixelPitchXmm = 0.00252795;  % mm/pixel
cfg.outputPixelPitchYmm = 0.00253165;  % mm/pixel
cfg.pixelPitchCalibrated = pixelPitchCalibrated;

% 主点、畸变和成像模型是否已正式标定。
cfg.cameraIntrinsicsCalibrated = cameraIntrinsicsCalibrated;

cfg.focalMmToPixelsIsPlaceholder = ...
    ~cfg.pixelPitchCalibrated;

cfg.cameraIntrinsicsArePlaceholder = ...
    ~cfg.cameraIntrinsicsCalibrated || ...
    cfg.focalMmToPixelsIsPlaceholder;

% 02 模块中的唯一实时换算：
% fxMeasuredPx = focalLengthMeasuredMm / outputPixelPitchXmm
% fyMeasuredPx = focalLengthMeasuredMm / outputPixelPitchYmm
%
% 不设置虚假的 focalLength0Mm、fx0Px、fy0Px。
% 第一帧真实焦距到来前，相机模型保持无效。

%% 7. 焦距范围校验与Zoom接口许可
% 焦距范围与速度的常用设置已经集中到第2.6节。
% 本节只校验范围关系，并生成Zoom底层接口相关的许可状态。
if any(cfg.focalLengthHardwareMinMm >= ...
        cfg.focalLengthHardwareMaxMm) || ...
        any(cfg.focalLengthHardwareMinMm > ...
        cfg.focalLengthWorkingMinMm) || ...
        any(cfg.focalLengthWorkingMinMm >= ...
        cfg.focalLengthWorkingMaxMm) || ...
        any(cfg.focalLengthWorkingMaxMm > ...
        cfg.focalLengthHardwareMaxMm)
    error('StereoIBVS:InvalidFocalLengthRanges', ...
        '焦距硬件范围和工作范围不一致。');
end

% Python 底层是否已经验证：
% 1. 接收 mm/s；
% 2. 正负方向正确；
% 3. 速度限制正确；
% 4. 停止命令可靠。
cfg.focalRateCommandInterfaceValidated = ...
    focalRateCommandInterfaceValidated;
cfg.zoomCalibrationValid = ...
    cfg.focalRateCommandInterfaceValidated;
cfg.zoomCalibrationIsPlaceholder = ...
    ~cfg.zoomCalibrationValid;
cfg.zoomRateLimitIsPlaceholder = ...
    ~cfg.focalRateCommandInterfaceValidated;

%% 8. 视觉测量、可见性与双目逆深度
cfg.numericalEpsilon = 1e-8;
cfg.visibilityEpsilon = 1e-6;
cfg.visibilityZMin = 0.10;

% EKF及控制允许使用的逆深度估计范围，单位1/m。
cfg.rhoEstimateMin = 0.80;
cfg.rhoEstimateMax = 2.20;

% V2旧模块继续读取rhoMin/rhoMax；数值只由上面两个主参数派生。
cfg.rhoMin = cfg.rhoEstimateMin;
cfg.rhoMax = cfg.rhoEstimateMax;

% 双目视差的数值下限，防止接近0时除零或产生极大深度。
cfg.disparityMin = 1e-4;

% 右相机可见性边界、迟滞宽度和重新捕获所需连续有效帧数。
cfg.rightVisibilityMarginPx = 80;
cfg.rightVisibilityHysteresisPx = 20;
cfg.rightReacquireValidSamples = 5;

%% 9. 控制器内部状态初值
% 常用控制增益和任务开关已经集中到第2.3节。
% 以下两个初值只用于打断Core内部反馈环，正常调参时不修改。
% 用于打断 Core 内反馈环的两个 Unit Delay 初值。
cfg.qDotAppliedInitial = zeros(7,1);
cfg.depthErrorInitial = 0;

%% 10. Target EKF
% 表示检测噪声，不向真实测量主动注入噪声。
cfg.pixelNoiseStd = 0.5;
cfg.Rpixel = cfg.pixelNoiseStd^2 * eye(4);

cfg.sigmaAcceleration = 0.5;
cfg.sigmaJerk = 1.0;
cfg.ekfCovarianceJitter = 1e-12;
cfg.ekfSConditionMin = 1e-12;

% 首次有效双目测量初始化；Core 固定只使用真实新双目测量进行校正。
cfg.ekfInitializationMode = 1;

% 测量协方差由 05 模块根据实时 fxMeasuredPx、fyMeasuredPx 和 baseline 计算。

% 首次测量到来前的安全后备状态。
T_W_CL0 = getTransform( ...
    cfg.robot, ...
    cfg.q0, ...
    cfg.cameraBodyName, ...
    cfg.robotBaseName);

cfg.pWCL0 = T_W_CL0(1:3,4);
cfg.RWCL0 = T_W_CL0(1:3,1:3);
cfg.target0CL = [0; 0; cfg.Zd];
cfg.target0 = cfg.pWCL0 + cfg.RWCL0 * cfg.target0CL;

cfg.ekfX0 = [
    cfg.target0
    0
    0
    0
    0
    0
    0
];

cfg.ekfP0 = diag([
    0.02^2
    0.02^2
    0.05^2
    0.20^2
    0.20^2
    0.20^2
    0.50^2
    0.50^2
    0.50^2
]);

%% 11. 控制器诊断阈值
% 第2.4节和第2.5节已经包含Zoom控制器及优先级调度的常用参数。
% 以下阈值只用于日志和响应判定，通常不参与日常性能调参。
cfg.depthErrorLoggingThreshold = 0.02;
cfg.zoomResponseThreshold = 1e-3;
cfg.armDepthResponseThreshold = 1e-4;

%% 12. V2安全与饱和
% qLimitSoftMargin：距离URDF关节硬限位的软保护余量。
% cartesianLinearSpeedMax：相机笛卡尔线速度的最终保护上限，单位m/s。
% 两者属于安全边界，除非完成风险评估，否则不要作为性能参数放宽。
cfg.qLimitSoftMargin = 5*pi/180;
cfg.cartesianLinearSpeedMax = 2.0;

% FR3 官方全局关节速度上限为
% [2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26] rad/s；
% cfg.qDotMax 从 URDF 读取并继续作为硬件能力边界。
% Core 的 0.03 rad/s 是算法层低速上限，09 安全模块按七维整组同比例缩放。
cfg.qDotAlgorithmMax = ...
    min(cfg.qDotMax, ...
        qDotAlgorithmLimitRadPerSec * ones(7,1));

% Core 算法层关节加速度上限；七维速度增量按同一个比例缩小。
cfg.qDDotAlgorithmMax = ...
    qDDotAlgorithmLimitRadPerSec2 * ones(7,1);

%% 13. ROS 2输入监督与消息尺寸
cfg.jointStateTimeoutSec = 0.10;
cfg.visionTimeoutSec = 0.10;
cfg.visionTimeoutFrames = max(1,ceil(cfg.visionTimeoutSec/cfg.Ts));

cfg.targetLossFrameLimit = 3;
cfg.targetRecoveryFrameCount = 3;

cfg.visionMessageLength = 8;
cfg.jointPositionMessageLength = 7;
cfg.jointVelocityMessageLength = 7;
cfg.controllerStatusMessageLength = 13;
cfg.jointStateTimeoutFrames = ...
    max(1,ceil(cfg.jointStateTimeoutSec/cfg.Ts));

% controllerStatus 固定顺序：
% [inputDataValid;
%  cameraModelValid;
%  focalLengthFresh;
%  kinematicsValid;
%  validLeft;
%  validRight;
%  validStereoQualified;
%  ekfPredictionValid;
%  ekfMeasurementUpdated;
%  depthTaskWeight;
%  schedulerMode;
%  safetyValid;
%  controllerEnableSafe]
cfg.controllerStatusOrder = {
    'inputDataValid'
    'cameraModelValid'
    'focalLengthFresh'
    'kinematicsValid'
    'validLeft'
    'validRight'
    'validStereoQualified'
    'ekfPredictionValid'
    'ekfMeasurementUpdated'
    'depthTaskWeight'
    'schedulerMode'
    'safetyValid'
    'controllerEnableSafe'
};

% data = [validL; validR; uL; vL; uR; vR; scaleL; scaleR]
cfg.visionFeatureOrder = {
    'validL'
    'validR'
    'uL'
    'vL'
    'uR'
    'vR'
    'scaleL'
    'scaleR'
};

%% 14. 标定许可与真机安全锁
cfg.cameraModelCalibrationReady = ...
    cfg.cameraIntrinsicsCalibrated && ...
    cfg.pixelPitchCalibrated;

cfg.armControlCalibrationReady = ...
    cfg.cameraMountCalibrated && ...
    cfg.cameraModelCalibrationReady;

cfg.depthControlCalibrationReady = ...
    cfg.armControlCalibrationReady && ...
    cfg.stereoCalibrationValid;

cfg.zoomControlCalibrationReady = ...
    cfg.depthControlCalibrationReady && ...
    cfg.focalRateCommandInterfaceValidated;

cfg.fullDeploymentReady = ...
    (~logical(cfg.armControlEnable) || ...
        cfg.armControlCalibrationReady) && ...
    (~logical(cfg.depthTaskEnable) || ...
        cfg.depthControlCalibrationReady) && ...
    (~logical(cfg.zoomControlEnable) || ...
        cfg.zoomControlCalibrationReady);

% 当前工程兼容别名。
cfg.stage1CalibrationReady = ...
    cfg.armControlCalibrationReady;
cfg.controllerCalibrationReady = ...
    cfg.fullDeploymentReady;

%% 15. 写入MATLAB基础工作区
% 整个项目统一只维护 cfg 结构体。
% Simulink Constant 块使用 cfg.xxx，不再生成 cfg_xxx 独立变量。
assignin('base', 'cfg', cfg);

fprintf('\nStereo IBVS V2 部署配置加载完成。\n');
fprintf('配置：%s\n', cfg.configurationName);
fprintf('Core周期：%.6f s（%.1f Hz）\n', ...
    cfg.Ts, cfg.controlRateHz);
fprintf('焦距反馈：%s，单位%s\n', ...
    cfg.focalLengthTopic, cfg.focalLengthInputUnit);
fprintf('Zoom命令：%s，单位%s\n', ...
    cfg.focalRateCommandTopic, cfg.focalRateCommandUnit);
fprintf('Controller状态长度：%d\n', ...
    cfg.controllerStatusMessageLength);
fprintf('Arm标定许可：%d\n', ...
    double(cfg.armControlCalibrationReady));
fprintf('Depth标定许可：%d\n', ...
    double(cfg.depthControlCalibrationReady));
fprintf('Zoom标定许可：%d\n', ...
    double(cfg.zoomControlCalibrationReady));
fprintf('完整真机许可：%d\n', ...
    double(cfg.fullDeploymentReady));

clear projectDir repoDir fr3 cameraBody cameraJoint T_W_CL0 ...
    armControlEnable depthTaskEnable ...
    nullspaceEnable zoomControlEnable ...
    cameraMountCalibrated stereoCalibrationValid ...
    pixelPitchCalibrated cameraIntrinsicsCalibrated ...
    focalRateCommandInterfaceValidated ...
    qDotAlgorithmLimitRadPerSec ...
    qDDotAlgorithmLimitRadPerSec2;


function fr3 = parseFr3UrdfForController(urdfFile)
% 从 URDF 提取 V2 固定尺寸运动学和关节限制参数。

doc = xmlread(urdfFile);
root = doc.getDocumentElement;
fr3.robotName = char(root.getAttribute('name'));

jointNodes = root.getElementsByTagName('joint');
numberOfJoints = jointNodes.getLength;

jointTemplate = struct( ...
    'name', '', ...
    'type', '', ...
    'originXYZ', zeros(3,1), ...
    'originRPY', zeros(3,1), ...
    'axis', zeros(3,1), ...
    'lower', NaN, ...
    'upper', NaN, ...
    'velocity', NaN, ...
    'effort', NaN);

allJoints = repmat( ...
    jointTemplate, ...
    numberOfJoints, ...
    1);

for jointIndex = 1:numberOfJoints
    node = jointNodes.item(jointIndex-1);

    currentJoint = jointTemplate;
    currentJoint.name = ...
        char(node.getAttribute('name'));
    currentJoint.type = ...
        char(node.getAttribute('type'));

    currentJoint.originXYZ = parseUrdfVector( ...
        readChildAttribute( ...
        node, 'origin', 'xyz', '0 0 0'));

    currentJoint.originRPY = parseUrdfVector( ...
        readChildAttribute( ...
        node, 'origin', 'rpy', '0 0 0'));

    currentJoint.axis = parseUrdfVector( ...
        readChildAttribute( ...
        node, 'axis', 'xyz', '0 0 0'));

    currentJoint.lower = parseUrdfScalar( ...
        readChildAttribute( ...
        node, 'limit', 'lower', 'NaN'));

    currentJoint.upper = parseUrdfScalar( ...
        readChildAttribute( ...
        node, 'limit', 'upper', 'NaN'));

    currentJoint.velocity = parseUrdfScalar( ...
        readChildAttribute( ...
        node, 'limit', 'velocity', 'NaN'));

    currentJoint.effort = parseUrdfScalar( ...
        readChildAttribute( ...
        node, 'limit', 'effort', 'NaN'));

    allJoints(jointIndex) = currentJoint;
end

armJoints = repmat(jointTemplate, 7, 1);

for armIndex = 1:7
    expectedName = ...
        sprintf('fr3_joint%d', armIndex);

    foundIndex = find( ...
        strcmp({allJoints.name}, expectedName), ...
        1);

    if isempty(foundIndex)
        error('StereoIBVS:JointNotFound', ...
            'URDF 中找不到关节：%s', ...
            expectedName);
    end

    armJoints(armIndex) = ...
        allJoints(foundIndex);

    if ~strcmp( ...
            armJoints(armIndex).type, ...
            'revolute')
        error('StereoIBVS:InvalidJointType', ...
            '%s 类型为 %s，应为 revolute。', ...
            expectedName, ...
            armJoints(armIndex).type);
    end
end

joint8Index = find( ...
    strcmp({allJoints.name}, 'fr3_joint8'), ...
    1);

if isempty(joint8Index)
    error('StereoIBVS:Joint8NotFound', ...
        'URDF 中找不到 fr3_joint8。');
end

joint8 = allJoints(joint8Index);

fr3.originXYZ = ...
    reshape([armJoints.originXYZ], 3, 7);
fr3.originRPY = ...
    reshape([armJoints.originRPY], 3, 7);
fr3.axis = ...
    reshape([armJoints.axis], 3, 7);
fr3.qMin = ...
    reshape([armJoints.lower], 7, 1);
fr3.qMax = ...
    reshape([armJoints.upper], 7, 1);
fr3.qDotMax = ...
    reshape([armJoints.velocity], 7, 1);
fr3.jointTorqueMax = ...
    reshape([armJoints.effort], 7, 1);
fr3.joint8OriginXYZ = ...
    joint8.originXYZ;
fr3.joint8OriginRPY = ...
    joint8.originRPY;

if any(~isfinite(fr3.qMin)) || ...
        any(~isfinite(fr3.qMax)) || ...
        any(~isfinite(fr3.qDotMax)) || ...
        any(~isfinite(fr3.jointTorqueMax))
    error('StereoIBVS:InvalidURDFLimits', ...
        'FR3 URDF 中至少有一个关节限制不是有限值。');
end
end


function value = readChildAttribute( ...
        node, ...
        tagName, ...
        attributeName, ...
        defaultValue)

children = node.getElementsByTagName(tagName);

if children.getLength == 0
    value = defaultValue;
    return;
end

value = ...
    char(children.item(0).getAttribute(attributeName));

if isempty(value)
    value = defaultValue;
end
end


function vector = parseUrdfVector(textValue)
vector = sscanf(textValue, '%f');

if numel(vector) ~= 3
    error('StereoIBVS:URDFVectorParse', ...
        '无法解析三维向量：%s', ...
        textValue);
end

vector = reshape(vector, 3, 1);
end


function scalar = parseUrdfScalar(textValue)
scalar = sscanf(textValue, '%f', 1);

if isempty(scalar)
    scalar = NaN;
end
end