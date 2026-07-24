function [qDotCenter,centerError,centerTaskValid,jConditionMetric] = ...
    core_stage1_controller(cLMeasured,JL,validLeft,cameraModelValid,kinematicsValid, ...
    controllerEnable,centerDesired,rhoStage1,Kc,lambdaC)
%#codegen
% 代码作用：
% 根据左相机归一化中心误差计算Stage 1关节速度。
% 当前使用固定逆深度，不包含目标速度前馈、深度次任务和零空间任务。

% 输入参数含义：
% cLMeasured：
% 左相机目标归一化坐标，[xL; yL]。
%
% JL：
% 左相机光心雅可比，尺寸为6×7。
% 速度排列为[线速度; 角速度]，并在左相机坐标系表达。
%
% validLeft：
% 左相机目标是否有效。
%
% cameraModelValid：
% 当前相机内参模型是否有效。
%
% kinematicsValid：
% 当前机械臂运动学计算是否有效。
%
% controllerEnable：
% 控制器总使能信号。
%
% centerDesired：
% 期望归一化中心坐标，通常为[0;0]。
%
% rhoStage1：
% Stage 1使用的固定逆深度，rho=1/Z。
%
% Kc：
% 左相机中心任务反馈增益，尺寸为2×2。
%
% lambdaC：
% 中心任务阻尼伪逆系数。

% 输出参数含义：
% qDotCenter：
% 左相机中心任务产生的7维关节速度，单位rad/s。
%
% centerError：
% 当前归一化中心误差。
%
% centerTaskValid：
% 本次中心任务计算是否有效，有效为1，无效为0。
%
% jConditionMetric：
% 中心任务雅可比的条件指标。
% 数值越接近0，说明当前任务越接近奇异状态。

qDotCenter = zeros(7,1);
centerError = zeros(2,1);
centerTaskValid = 0;
jConditionMetric = 0;

cL = reshape(cLMeasured,2,1);
cLd = reshape(centerDesired,2,1);
JLeft = reshape(JL,6,7);
gainC = reshape(Kc,2,2);

% 检查所有输入是否为有限值。
if any(~isfinite(cL)) || ...
        any(~isfinite(cLd)) || ...
        any(~isfinite(JLeft(:))) || ...
        any(~isfinite(gainC(:))) || ...
        ~isfinite(rhoStage1) || ...
        ~isfinite(lambdaC)
    return;
end

% 检查控制器及相关输入是否有效。
if validLeft <= 0.5 || ...
        cameraModelValid <= 0.5 || ...
        kinematicsValid <= 0.5 || ...
        controllerEnable <= 0.5
    return;
end

if rhoStage1 <= 0 || lambdaC < 0
    return;
end

xL = cL(1);
yL = cL(2);
rho = rhoStage1;

centerError = cL - cLd;

% 左相机归一化点特征交互矩阵。
Lc = [
    -rho, 0, xL*rho, xL*yL, -(1+xL*xL), yL
    0, -rho, yL*rho, 1+yL*yL, -xL*yL, -xL
];

% 将相机速度任务映射到7维关节空间。
Jc = Lc * JLeft;

% Stage 1暂时不使用目标运动前馈。
nuC = -gainC * centerError;

% 阻尼最小二乘矩阵。
G = Jc * Jc.' + lambdaC^2 * eye(2);

% 计算简单的条件指标。
detG = G(1,1)*G(2,2) - G(1,2)*G(2,1);
traceG = G(1,1) + G(2,2);

jConditionMetric = detG / ...
    (traceG*traceG + 1e-12);

if detG <= 0 || any(~isfinite(G(:)))
    return;
end

% 阻尼伪逆：
% qDot = Jc' * inv(Jc*Jc' + lambda^2*I) * nuC
qDotCenter = Jc.' * (G \ nuC);

if any(~isfinite(qDotCenter))
    qDotCenter = zeros(7,1);
    return;
end

centerTaskValid = 1;
end