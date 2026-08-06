function build_visual_servo_dashboard(modelFile)
%BUILD_VISUAL_SERVO_DASHBOARD 为 single_camera_q_dot 建立调试面板组件。
%
% 用法：
%   build_visual_servo_dashboard
%
% 或：
%   build_visual_servo_dashboard('single_camera_q_dot.slx')
%
% 功能：
%   1. 根据当前模型的真实模块路径，批量创建 Dashboard Lamp、Display、
%      Dashboard Scope、Slider、Toggle Switch 和 Edit 组件；
%   2. 自动连接关键使能状态、控制速度、关节速度、EKF状态和保护缩放信号；
%   3. 自动连接 Kpx、Kpy、k_ff 的在线调参滑块；
%   4. 重复运行时，先删除名称以 DBG_ 开头的旧调试组件，再重新创建；
%   5. 不改动控制算法和信号线，不自动保存模型。
%
% 说明：
%   R2025b 没有公开、稳定的脚本接口，可以把 Dashboard 组件直接分配到
%   已建立的 Panel 选项卡。因此，本脚本先在模型根画布右侧创建四组组件。
%   脚本运行完成后，进入“面板 -> 编辑面板”，框选每组组件并拖入：
%       Status / Control / EKF / Tuning
%
% 重要：
%   模型中 Kpx、Kpy、K_ff 使用 Manual Switch 在工作区变量和调试常数之间切换。
%   本脚本把三个 Manual Switch 设为 sw='0'，即选择第二输入，使滑块生效。
%   要恢复使用初始化脚本中的 Kpx、Kpy、k_ff，请把三个 Manual Switch
%   双击切回第一输入，或在命令行执行：
%       set_param('single_camera_q_dot/Planar Target KF and Task Command/Manual Switch','sw','1')
%       set_param('single_camera_q_dot/Planar Target KF and Task Command/Manual Switch1','sw','1')
%       set_param('single_camera_q_dot/Planar Target KF and Task Command/Manual Switch2','sw','1')
%
% 适配模型：
%   single_camera_q_dot.slx
%   依据模型当前输出定义：
%       EKF状态 = [X_B; Y_B; Vx_B; Vy_B]，尺寸为 4x1
%       EKF协方差尺寸为 4x4
%
% 生成日期：2026-08-06

%% 0. 基本设置
if nargin < 1 || isempty(modelFile)
    modelFile = 'single_camera_q_dot.slx';
end

modelFile = char(modelFile);
[modelFolder, mdl, ext] = fileparts(modelFile);

if isempty(ext)
    ext = '.slx';
    modelFile = [modelFile ext];
end

if isempty(modelFolder)
    modelFolder = pwd;
    modelFile = fullfile(modelFolder, modelFile);
end

if ~isfile(modelFile) && ~bdIsLoaded(mdl)
    error('找不到模型文件：%s', modelFile);
end

% 调参控件范围，可根据实验需要修改。
KPX_LIMITS = [0 0.5 6];
KPY_LIMITS = [0 0.5 6];
KFF_LIMITS = [0 0.1 2];

%% 1. 加载并打开模型
if ~bdIsLoaded(mdl)
    load_system(modelFile);
end
open_system(mdl);

% 检查关键模块，防止脚本误用于其他版本模型。
requiredBlocks = {
    [mdl '/Message Validation']
    [mdl '/Kinematics Feedback']
    [mdl '/Planar Target KF and Task Command']
    [mdl '/Redundant Visual Velocity Solver']
    [mdl '/Final Joint Command Guard']
    };

for i = 1:numel(requiredBlocks)
    if getSimulinkBlockHandle(requiredBlocks{i}) < 0
        error('模型中缺少关键模块：%s', requiredBlocks{i});
    end
end

%% 2. 删除旧的 DBG_ 调试组件
oldBlocks = find_system(mdl, ...
    'SearchDepth', 1, ...
    'Type', 'block', ...
    'RegExp', 'on', ...
    'Name', '^DBG_');

for i = 1:numel(oldBlocks)
    delete_block(oldBlocks{i});
end

%% 3. 确定调试组件放置区域
rootBlocks = find_system(mdl, 'SearchDepth', 1, 'Type', 'block');
maxX = 0;

for i = 1:numel(rootBlocks)
    p = get_param(rootBlocks{i}, 'Position');
    if isnumeric(p) && numel(p) == 4
        maxX = max(maxX, p(3));
    end
end

% 四组组件统一放到模型最右侧，避免覆盖现有模型。
baseX = maxX + 220;
statusX  = baseX;
tuningX  = baseX + 930;
controlX = baseX;
ekfX     = baseX + 930;

statusY  = 100;
tuningY  = 100;
controlY = 1050;
ekfY     = 1050;

%% 4. 信号源模块路径
msgValidation = [mdl '/Message Validation'];
targetSub     = [mdl '/Target ROS2 Subscriber'];
kinFeedback   = [mdl '/Kinematics Feedback'];
kfTask        = [mdl '/Planar Target KF and Task Command'];
solver        = [mdl '/Redundant Visual Velocity Solver'];
guard         = [mdl '/Final Joint Command Guard'];
normImage     = [mdl '/Normalized Image Coordinates'];

%% 5. STATUS：使能与有效性状态灯
% 每行：组件名、源模块、输出端口、灯逻辑。
% normal：0红、1绿；warning：0绿、1黄。
statusSignals = {
    'safe_valid',            msgValidation, 1,  'normal'
    'target_visible',        msgValidation, 6,  'normal'
    'target_is_new',         targetSub,     1,  'normal'
    'kinematics_valid',      kinFeedback,   2,  'normal'
    'ekf_initialized',       kfTask,        7,  'normal'
    'measurement_accepted',  kfTask,        9,  'normal'
    'controller_ok',         kfTask,       10,  'normal'
    'control_active',        kfTask,       13,  'normal'
    'solver_valid',          solver,        5,  'normal'
    'guard_valid',           guard,         5,  'normal'
    };

lampW = 115;
lampH = 95;
lampGapX = 25;
lampGapY = 35;
nLampCols = 5;

for i = 1:size(statusSignals, 1)
    row = floor((i-1) / nLampCols);
    col = mod(i-1, nLampCols);
    pos = [ ...
        statusX + col*(lampW+lampGapX), ...
        statusY + row*(lampH+lampGapY), ...
        statusX + col*(lampW+lampGapX) + lampW, ...
        statusY + row*(lampH+lampGapY) + lampH];

    blockName = ['DBG_STATUS_' statusSignals{i,1}];
    dashPath = addDashboardBlock( ...
        {'simulink_hmi_blocks/Lamp'}, mdl, blockName, pos);

    bindSignal(dashPath, statusSignals{i,2}, statusSignals{i,3});
    configureLamp(dashPath, statusSignals{i,4});
    trySet(dashPath, 'LabelPosition', 'Bottom');
end

%% 6. STATUS：关键数值显示
statusValues = {
    'innovation_nis',      kfTask, 8
    'sigma_min',           solver, 2
    'damping_used',        solver, 3
    'singularity_scale',   solver, 4
    'velocity_scale',      guard,  2
    'acceleration_scale',  guard,  3
    'position_scale',      guard,  4
    };

displayW = 170;
displayH = 78;
displayGapX = 25;
displayStartY = statusY + 2*(lampH+lampGapY) + 20;

for i = 1:size(statusValues, 1)
    row = floor((i-1) / 4);
    col = mod(i-1, 4);
    pos = [ ...
        statusX + col*(displayW+displayGapX), ...
        displayStartY + row*(displayH+25), ...
        statusX + col*(displayW+displayGapX) + displayW, ...
        displayStartY + row*(displayH+25) + displayH];

    blockName = ['DBG_STATUS_' statusValues{i,1}];
    dashPath = addDashboardBlock( ...
        {'simulink_hmi_blocks/Display'}, mdl, blockName, pos);

    bindSignal(dashPath, statusValues{i,2}, statusValues{i,3});
    trySet(dashPath, 'LabelPosition', 'Bottom');
end

%% 7. CONTROL：误差、速度分量与关节命令
scopeW = 390;
scopeH = 225;
scopeGapX = 35;
scopeGapY = 45;

% 7.1 图像误差 e = [e_x; e_y]
scopeError = addDashboardBlock( ...
    {'simulink_hmi_blocks/Dashboard Scope'}, ...
    mdl, 'DBG_CONTROL_tracking_error', ...
    [controlX, controlY, controlX+scopeW, controlY+scopeH]);

bindScope(scopeError, {
    normImage, 1
    });
trySet(scopeError, 'LabelPosition', 'Bottom');

% 7.2 控制速度：v_p、v_ff、v_task_raw、v_issued
% 四个二维向量共8个通道，正好不超过 Dashboard Scope 的8通道限制。
scopeTask = addDashboardBlock( ...
    {'simulink_hmi_blocks/Dashboard Scope'}, ...
    mdl, 'DBG_CONTROL_task_velocity', ...
    [controlX+scopeW+scopeGapX, controlY, ...
     controlX+2*scopeW+scopeGapX, controlY+scopeH]);

bindScope(scopeTask, {
    kfTask, 4
    kfTask, 5
    kfTask, 12
    kfTask, 6
    });
trySet(scopeTask, 'LabelPosition', 'Bottom');

% 7.3 原始关节速度 qdot_raw，7通道
scopeQdotRaw = addDashboardBlock( ...
    {'simulink_hmi_blocks/Dashboard Scope'}, ...
    mdl, 'DBG_CONTROL_qdot_raw', ...
    [controlX, controlY+scopeH+scopeGapY, ...
     controlX+scopeW, controlY+2*scopeH+scopeGapY]);

bindScope(scopeQdotRaw, {
    solver, 1
    });
trySet(scopeQdotRaw, 'LabelPosition', 'Bottom');

% 7.4 最终关节速度 qdot_command，7通道
scopeQdotCmd = addDashboardBlock( ...
    {'simulink_hmi_blocks/Dashboard Scope'}, ...
    mdl, 'DBG_CONTROL_qdot_command', ...
    [controlX+scopeW+scopeGapX, controlY+scopeH+scopeGapY, ...
     controlX+2*scopeW+scopeGapX, controlY+2*scopeH+scopeGapY]);

bindScope(scopeQdotCmd, {
    guard, 1
    });
trySet(scopeQdotCmd, 'LabelPosition', 'Bottom');

%% 8. EKF：四状态、目标速度、NIS与协方差
% 源码中的状态定义为：
%   ekf_state = [X_B; Y_B; Vx_B; Vy_B]
scopeEkfState = addDashboardBlock( ...
    {'simulink_hmi_blocks/Dashboard Scope'}, ...
    mdl, 'DBG_EKF_state_XYVxVy', ...
    [ekfX, ekfY, ekfX+scopeW, ekfY+scopeH]);

bindScope(scopeEkfState, {
    kfTask, 1
    });
trySet(scopeEkfState, 'LabelPosition', 'Bottom');

scopeTargetVelocity = addDashboardBlock( ...
    {'simulink_hmi_blocks/Dashboard Scope'}, ...
    mdl, 'DBG_EKF_target_velocity', ...
    [ekfX+scopeW+scopeGapX, ekfY, ...
     ekfX+2*scopeW+scopeGapX, ekfY+scopeH]);

bindScope(scopeTargetVelocity, {
    kfTask, 3
    });
trySet(scopeTargetVelocity, 'LabelPosition', 'Bottom');

scopeNis = addDashboardBlock( ...
    {'simulink_hmi_blocks/Dashboard Scope'}, ...
    mdl, 'DBG_EKF_innovation_nis_history', ...
    [ekfX, ekfY+scopeH+scopeGapY, ...
     ekfX+scopeW, ekfY+2*scopeH+scopeGapY]);

bindScope(scopeNis, {
    kfTask, 8
    });
trySet(scopeNis, 'LabelPosition', 'Bottom');

covDisplay = addDashboardBlock( ...
    {'simulink_hmi_blocks/Display'}, ...
    mdl, 'DBG_EKF_covariance_4x4', ...
    [ekfX+scopeW+scopeGapX, ekfY+scopeH+scopeGapY, ...
     ekfX+2*scopeW+scopeGapX, ekfY+2*scopeH+scopeGapY]);

bindSignal(covDisplay, kfTask, 2);
trySet(covDisplay, 'LabelPosition', 'Bottom');

%% 9. TUNING：Kpx、Kpy、k_ff 在线调参
paramSys = [mdl '/Planar Target KF and Task Command'];

% 选择三个 Manual Switch 的第二输入，让数值调试常数生效。
% MathWorks定义：sw='0' 连接第二输入，sw='1' 连接第一输入。
set_param([paramSys '/Manual Switch'],  'sw', '0');
set_param([paramSys '/Manual Switch1'], 'sw', '0');
set_param([paramSys '/Manual Switch2'], 'sw', '0');

sliderW = 360;
sliderH = 90;
sliderGapY = 35;

sliderKpx = addDashboardBlock( ...
    {'simulink_hmi_blocks/Slider'}, ...
    mdl, 'DBG_TUNING_Kpx', ...
    [tuningX, tuningY, tuningX+sliderW, tuningY+sliderH]);
bindParameter(sliderKpx, [paramSys '/Kpx'], 'Value', '');
trySet(sliderKpx, 'Limits', KPX_LIMITS);
trySet(sliderKpx, 'LabelPosition', 'Bottom');

sliderKpy = addDashboardBlock( ...
    {'simulink_hmi_blocks/Slider'}, ...
    mdl, 'DBG_TUNING_Kpy', ...
    [tuningX, tuningY+sliderH+sliderGapY, ...
     tuningX+sliderW, tuningY+2*sliderH+sliderGapY]);
bindParameter(sliderKpy, [paramSys '/Kpy'], 'Value', '');
trySet(sliderKpy, 'Limits', KPY_LIMITS);
trySet(sliderKpy, 'LabelPosition', 'Bottom');

sliderKff = addDashboardBlock( ...
    {'simulink_hmi_blocks/Slider'}, ...
    mdl, 'DBG_TUNING_k_ff', ...
    [tuningX, tuningY+2*(sliderH+sliderGapY), ...
     tuningX+sliderW, tuningY+3*sliderH+2*sliderGapY]);
bindParameter(sliderKff, [paramSys '/K_ff'], 'Value', '');
trySet(sliderKff, 'Limits', KFF_LIMITS);
trySet(sliderKff, 'LabelPosition', 'Bottom');

%% 10. TUNING：控制使能开关
% 连接到三个 Constant 块的 Value 参数。
toggleX = tuningX + sliderW + 60;
toggleW = 150;
toggleH = 100;

toggleP = addDashboardBlock( ...
    {'simulink_hmi_blocks/Toggle Switch'}, ...
    mdl, 'DBG_TUNING_enable_proportional', ...
    [toggleX, tuningY, toggleX+toggleW, tuningY+toggleH]);
bindParameter(toggleP, [paramSys '/Constant11'], 'Value', '');
trySet(toggleP, 'LabelPosition', 'Bottom');

toggleFF = addDashboardBlock( ...
    {'simulink_hmi_blocks/Toggle Switch'}, ...
    mdl, 'DBG_TUNING_enable_feedforward', ...
    [toggleX, tuningY+toggleH+35, ...
     toggleX+toggleW, tuningY+2*toggleH+35]);
bindParameter(toggleFF, [paramSys '/Constant12'], 'Value', '');
trySet(toggleFF, 'LabelPosition', 'Bottom');

toggleController = addDashboardBlock( ...
    {'simulink_hmi_blocks/Toggle Switch'}, ...
    mdl, 'DBG_TUNING_controller_enable', ...
    [toggleX, tuningY+2*(toggleH+35), ...
     toggleX+toggleW, tuningY+3*toggleH+70]);
bindParameter(toggleController, [paramSys '/Constant14'], 'Value', '');
trySet(toggleController, 'LabelPosition', 'Bottom');

%% 11. TUNING：EKF关键参数精确输入
% Edit 组件用于输入精确数值，避免用滑块猜测数量级。
editX = tuningX;
editY = tuningY + 3*(sliderH+sliderGapY) + 45;
editW = 175;
editH = 65;
editGapX = 25;
editGapY = 35;

ekfTuneItems = {
    'R_xx',                   [paramSys '/R EKF'],     'Value', '(1,1)'
    'R_yy',                   [paramSys '/R EKF'],     'Value', '(2,2)'
    'Q_X',                    [paramSys '/Q EKF'],     'Value', '(1,1)'
    'Q_Y',                    [paramSys '/Q EKF'],     'Value', '(2,2)'
    'Q_Vx',                   [paramSys '/Q EKF'],     'Value', '(3,3)'
    'Q_Vy',                   [paramSys '/Q EKF'],     'Value', '(4,4)'
    'gate_threshold',         [paramSys '/Constant10'],'Value', ''
    'required_valid_frames',  [paramSys '/Constant7'], 'Value', ''
    'measurement_reset_time', [paramSys '/Constant8'], 'Value', ''
    };

for i = 1:size(ekfTuneItems, 1)
    row = floor((i-1)/3);
    col = mod(i-1,3);

    pos = [ ...
        editX + col*(editW+editGapX), ...
        editY + row*(editH+editGapY), ...
        editX + col*(editW+editGapX) + editW, ...
        editY + row*(editH+editGapY) + editH];

    editPath = addDashboardBlock( ...
        {'simulink_hmi_blocks/Edit'}, ...
        mdl, ['DBG_TUNING_' ekfTuneItems{i,1}], pos);

    bindParameter(editPath, ekfTuneItems{i,2}, ...
        ekfTuneItems{i,3}, ekfTuneItems{i,4});
    trySet(editPath, 'LabelPosition', 'Bottom');
end

%% 12. 尝试更新模型，检查连接
try
    set_param(mdl, 'SimulationCommand', 'update');
catch ME
    warning(['调试组件已创建，但模型更新未完成。通常是初始化变量、ROS2环境' ...
        '或机器人模型尚未加载。先运行初始化脚本，再按 Ctrl+D 更新模型。\n原因：%s'], ...
        ME.message);
end

%% 13. 打开根画布并提示后续操作
open_system(mdl);

fprintf('\n============================================================\n');
fprintf('Visual Servo Dashboard 调试组件已建立。\n');
fprintf('模型：%s\n', mdl);
fprintf('创建位置：模型根画布最右侧，x >= %.0f\n', baseX);
fprintf('\n请按以下方式放入现有面板：\n');
fprintf('  1. 顶部“面板” -> “编辑面板”；\n');
fprintf('  2. 框选 DBG_STATUS_* 组件，拖入 Status；\n');
fprintf('  3. 框选 DBG_CONTROL_* 组件，拖入 Control；\n');
fprintf('  4. 框选 DBG_EKF_* 组件，拖入 EKF；\n');
fprintf('  5. 框选 DBG_TUNING_* 组件，拖入 Tuning；\n');
fprintf('  6. 确认后手动保存模型。\n');
fprintf('\n注意：Kpx、Kpy、k_ff 的 Manual Switch 已切到第二输入，滑块已生效。\n');
fprintf('============================================================\n\n');

end


%% ========================================================================
% 本文件的局部辅助函数
% =========================================================================

function dashPath = addDashboardBlock(libraryCandidates, mdl, blockName, position)
% 尝试从候选库路径创建 Dashboard 组件。
dashPath = [mdl '/' blockName];
lastError = [];

for i = 1:numel(libraryCandidates)
    try
        add_block(libraryCandidates{i}, dashPath, 'Position', position);
        trySet(dashPath, 'ShowName', 'on');
        return;
    catch ME
        lastError = ME;
    end
end

if isempty(lastError)
    error('无法创建 Dashboard 组件：%s', blockName);
else
    error('无法创建 Dashboard 组件 %s：%s', blockName, lastError.message);
end
end


function bindSignal(dashboardPath, sourceBlockPath, outputPort)
% 将 Dashboard 显示组件连接到一个信号。
spec = Simulink.HMI.SignalSpecification;
spec.BlockPath = Simulink.BlockPath(sourceBlockPath);
spec.OutputPortIndex = outputPort;
set_param(dashboardPath, 'Binding', spec);
end


function bindScope(dashboardPath, sources)
% 将 Dashboard Scope 连接到多个信号。
% sources 为 N×2 cell：{源模块路径, 输出端口}
specs = cell(1, size(sources,1));

for i = 1:size(sources,1)
    spec = Simulink.HMI.SignalSpecification;
    spec.BlockPath = Simulink.BlockPath(sources{i,1});
    spec.OutputPortIndex = sources{i,2};
    specs{i} = spec;
end

set_param(dashboardPath, 'Binding', specs);
end


function bindParameter(dashboardPath, sourceBlockPath, parameterName, element)
% 将 Dashboard 控制组件连接到模块参数或参数元素。
info = Simulink.HMI.ParamSourceInfo;
info.BlockPath = Simulink.BlockPath(sourceBlockPath);
info.ParamName = parameterName;

if nargin >= 4 && ~isempty(element)
    info.Element = element;
end

try
    set_param(dashboardPath, 'Binding', info);
catch ME
    warning('参数连接失败：%s -> %s.%s%s\n原因：%s', ...
        dashboardPath, sourceBlockPath, parameterName, element, ME.message);
end
end


function configureLamp(lampPath, mode)
% 设置 Lamp 的状态颜色。
state0.Value = 0;
state1.Value = 1;

switch lower(mode)
    case 'warning'
        state0.Color = [0.15 0.75 0.20];  % 正常：绿色
        state1.Color = [1.00 0.65 0.05];  % 激活：黄色
    otherwise
        state0.Color = [0.85 0.12 0.12];  % 失效：红色
        state1.Color = [0.15 0.75 0.20];  % 有效：绿色
end

set_param(lampPath, ...
    'StateColors', [state0 state1], ...
    'ColorDefault', [0.72 0.72 0.72]);
end


function trySet(blockPath, parameterName, parameterValue)
% 某些外观参数在不同版本名称可能不同，设置失败时不终止主脚本。
try
    set_param(blockPath, parameterName, parameterValue);
catch
    % 外观参数设置失败不影响组件创建和连接。
end
end
