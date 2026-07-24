function build_stereo_ibvs_sim_v2_1(projectDir)
% 代码作用：
% 生成Stereo IBVS V2.1离线闭环仿真模型。
% 模型使用虚拟FR3关节积分器和双目针孔相机，将Core输出的关节速度
% 反馈为新的关节角与图像特征，用于验证中心视觉任务能否收敛。
%
% 输入：
% projectDir：工程根目录或simulink目录，可省略并自动定位。
%
% 输出：
% 本函数没有返回值。成功后生成：
% simulink/sim/stereo_ibvs_sim_v2_1.slx
%
% 调用方法：
% build_stereo_ibvs_sim_v2_1
% build_stereo_ibvs_sim_v2_1('/path/to/visual_servo_tag_project')
%
% 仿真方法：
% 仅在仿真工作区临时启用Arm中心任务；Depth、Zoom和Nullspace关闭。
% 固定世界坐标系中的目标点，使用FR3正运动学和双目针孔投影生成
% 每周期视觉测量，再对Core速度命令积分形成离散闭环。

if nargin < 1
    projectDir = '';
end

%% 1. 定位并加载工程文件
[coreFile, configFile, simDir] = locateProjectFiles(projectDir);
coreDir = fileparts(coreFile);
configDir = fileparts(configFile);
addpath(coreDir);
addpath(configDir);

run(configFile);
cfgLocal = evalin('base', 'cfg');
validateConfiguration(cfgLocal);
cfgLocal = makeSimulationConfiguration(cfgLocal);
assignin('base', 'cfg', cfgLocal);

[~, coreModelName] = fileparts(coreFile);
if bdIsLoaded(coreModelName)
    loadedCoreFile = get_param(coreModelName, 'FileName');
    if ~samePath(loadedCoreFile, coreFile)
        close_system(coreModelName, 0);
    end
end
load_system(coreFile);

% Core自己的InitFcn会重新载入真机配置。V2.1只在当前MATLAB会话中
% 暂停该回调，避免覆盖仿真专用许可；不会保存或修改Core文件。
set_param(coreModelName, 'InitFcn', '');

%% 2. 计算固定目标的世界坐标
T_W_CL0 = getTransform( ...
    cfgLocal.robot, cfgLocal.q0, cfgLocal.cameraBodyName);
targetInitialCL = [0.08; 0.05; 1.00; 1];
cfgLocal.simV21TargetWorld = ...
    T_W_CL0 * targetInitialCL;
cfgLocal.simV21TargetWorld = ...
    cfgLocal.simV21TargetWorld(1:3);
cfgLocal.simV21FocalLengthMm = [10; 10];
cfgLocal.simV21TagSizeM = 0.06;
assignin('base', 'cfg', cfgLocal);

%% 3. 新建V2.1模型
modelName = 'stereo_ibvs_sim_v2_1';
modelFile = fullfile(simDir, [modelName '.slx']);

if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
if isfile(modelFile)
    delete(modelFile);
end

new_system(modelName);
open_system(modelName);

try
    configureModel(modelName);
    configureCallbacks(modelName);
    addDescription(modelName);

    plantPath = addVirtualPlant(modelName);
    cameraPath = addVirtualStereoCamera(modelName);
    inputPath = addCoreInputSignals(modelName);
    corePath = addCoreReference(modelName, coreModelName);
    addLogging(modelName);
    connectClosedLoop( ...
        modelName, plantPath, cameraPath, inputPath, corePath);

    save_system(modelName, modelFile);
    set_param(modelName, 'SimulationCommand', 'update');
    save_system(modelName, modelFile);
    set_param(modelName, 'ZoomFactor', 'FitSystem');
catch ME
    try
        save_system(modelName, modelFile);
    catch
    end
    fprintf(2, '\nV2.1模型生成失败：\n%s\n', ...
        getReport(ME, 'extended', 'hyperlinks', 'off'));
    rethrow(ME);
end

fprintf('\nV2.1离线闭环模型生成成功：\n%s\n', modelFile);
fprintf('运行：sim(''%s'');\n', modelName);
fprintf('初始目标相对左相机：[0.08; 0.05; 1.00] m。\n');
fprintf('0.5秒后开启中心视觉任务，观察像素误差是否收敛。\n');
end


function cfgLocal = makeSimulationConfiguration(cfgLocal)
% 代码作用：
% 只为当前离线仿真建立中心任务所需许可，不修改部署配置文件。

cfgLocal.armControlEnable = true;
cfgLocal.depthTaskEnable = false;
cfgLocal.zoomControlEnable = false;
cfgLocal.nullspaceEnable = false;
cfgLocal.cameraMountCalibrated = true;
cfgLocal.cameraIntrinsicsCalibrated = true;
cfgLocal.pixelPitchCalibrated = true;
cfgLocal.cameraModelCalibrationReady = true;
cfgLocal.armControlCalibrationReady = true;
cfgLocal.stage1CalibrationReady = true;
cfgLocal.fullDeploymentReady = true;
cfgLocal.controllerCalibrationReady = true;
end


function configureModel(modelName)
% 代码作用：
% 设置60 Hz固定步长离线仿真。

set_param(modelName, ...
    'StartTime', '0', ...
    'StopTime', '8', ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', 'cfg.Ts', ...
    'ReturnWorkspaceOutputs', 'on', ...
    'SignalLogging', 'on', ...
    'SignalLoggingName', 'logsout', ...
    'UnconnectedInputMsg', 'error', ...
    'UnconnectedOutputMsg', 'error', ...
    'AlgebraicLoopMsg', 'error');
end


function configureCallbacks(modelName)
% 代码作用：
% 使生成模型换电脑后仍能定位Core和配置，并重建仿真专用参数。

callback = strjoin({ ...
    'simFile__=get_param(bdroot,''FileName'');'
    'simDir__=fileparts(simFile__);'
    'simulinkDir__=fileparts(simDir__);'
    'coreDir__=fullfile(simulinkDir__,''core'');'
    'configDir__=fullfile(simulinkDir__,''config'');'
    'coreFile__=fullfile(coreDir__,''stereo_ibvs_core.slx'');'
    'configFile__=fullfile(configDir__,''stereo_ibvs_config.m'');'
    'addpath(coreDir__); addpath(configDir__);'
    'if ~bdIsLoaded(''stereo_ibvs_core''), load_system(coreFile__); end'
    'set_param(''stereo_ibvs_core'',''InitFcn'','''');'
    'run(configFile__);'
    'cfg.armControlEnable=true; cfg.depthTaskEnable=false;'
    'cfg.zoomControlEnable=false; cfg.nullspaceEnable=false;'
    'cfg.cameraMountCalibrated=true;'
    'cfg.cameraIntrinsicsCalibrated=true;'
    'cfg.pixelPitchCalibrated=true;'
    'cfg.cameraModelCalibrationReady=true;'
    'cfg.armControlCalibrationReady=true;'
    'cfg.stage1CalibrationReady=true;'
    'cfg.fullDeploymentReady=true;'
    'cfg.controllerCalibrationReady=true;'
    'T_W_CL0__=getTransform(cfg.robot,cfg.q0,cfg.cameraBodyName);'
    'targetCL__=[0.08;0.05;1.00;1];'
    'targetW__=T_W_CL0__*targetCL__;'
    'cfg.simV21TargetWorld=targetW__(1:3);'
    'cfg.simV21FocalLengthMm=[10;10];'
    'cfg.simV21TagSizeM=0.06;'
    'assignin(''base'',''cfg'',cfg);'
    'clear simFile__ simDir__ simulinkDir__ coreDir__ configDir__'
    'clear coreFile__ configFile__ T_W_CL0__ targetCL__ targetW__' ...
    }, newline);

set_param(modelName, 'InitFcn', callback);
end


function addDescription(modelName)
% 代码作用：
% 添加V2.1闭环结构和测试范围说明。

text = sprintf([ ...
    'Stereo IBVS V2.1 离线闭环仿真\n' ...
    'q0 -> FR3关节积分 -> 双目投影 -> Core -> qDot -> FR3关节积分\n' ...
    '0~0.5 s：控制器关闭；0.5~8 s：中心任务闭环\n' ...
    '第一版：固定目标、固定焦距、无噪声、无延迟、无Depth/Zoom/Nullspace']);
annotation = Simulink.Annotation(modelName, text);
annotation.Position = [35 15 720 95];
end


function subsystemPath = addVirtualPlant(modelName)
% 代码作用：
% 创建以Core关节速度为输入的FR3离散关节积分器。

subsystemPath = [modelName '/Virtual FR3 Plant'];
createEmptySubsystem(subsystemPath, [70 180 300 350]);
addInport(subsystemPath, 'qDotCmd', 1, '[7 1]', [30 80 60 94]);
add_block('simulink/Discrete/Discrete-Time Integrator', ...
    [subsystemPath '/Joint Position Integrator'], ...
    'Position', [145 60 315 115], ...
    'gainval', '1', ...
    'SampleTime', 'cfg.Ts', ...
    'InitialCondition', 'cfg.q0', ...
    'LimitOutput', 'on', ...
    'UpperSaturationLimit', 'cfg.qMax', ...
    'LowerSaturationLimit', 'cfg.qMin');
addOutport(subsystemPath, 'q', 1, '[7 1]', [395 65 425 85]);
addOutport(subsystemPath, 'qDotMeasured', 2, '[7 1]', [395 125 425 145]);
safeAddLine(subsystemPath, 'qDotCmd/1', 'Joint Position Integrator/1');
safeAddLine(subsystemPath, 'Joint Position Integrator/1', 'q/1');
safeAddLine(subsystemPath, 'qDotCmd/1', 'qDotMeasured/1');
end


function subsystemPath = addVirtualStereoCamera(modelName)
% 代码作用：
% 创建FR3正运动学与左右针孔相机投影，输出Core的8维视觉接口。

subsystemPath = [modelName '/Virtual Stereo Camera'];
createEmptySubsystem(subsystemPath, [390 130 690 400]);
addInport(subsystemPath, 'q', 1, '[7 1]', [25 70 55 84]);

constants = {
    'Target World', 'cfg.simV21TargetWorld'
    'Focal Length Mm', 'cfg.simV21FocalLengthMm'
    'FR3 Origin XYZ', 'cfg.fr3OriginXYZ'
    'FR3 Origin RPY', 'cfg.fr3OriginRPY'
    'FR3 Joint Axis', 'cfg.fr3Axis'
    'Joint8 Origin XYZ', 'cfg.fr3Joint8OriginXYZ'
    'Joint8 Origin RPY', 'cfg.fr3Joint8OriginRPY'
    'Link8 To Left Camera', 'cfg.T_link8_CL'
    'Left To Right Camera', 'cfg.T_CL_CR'
    'World To Robot Base', 'cfg.T_W_B'
    'Camera Parameters', ...
        '[cfg.outputPixelPitchXmm;cfg.outputPixelPitchYmm;' ...
        'cfg.cxL;cfg.cyL;cfg.cxR;cfg.cyR;' ...
        'cfg.imageWidthPx;cfg.imageHeightPx;cfg.simV21TagSizeM]'
};

for index = 1:size(constants, 1)
    y = 115 + 42*(index-1);
    add_block('simulink/Sources/Constant', ...
        [subsystemPath '/' constants{index,1}], ...
        'Value', constants{index,2}, ...
        'SampleTime', '-1', ...
        'VectorParams1D', 'off', ...
        'Position', [25 y 180 y+25]);
end

functionPath = [subsystemPath '/FR3 Stereo Projection'];
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    functionPath, 'Position', [265 70 535 460]);
setMatlabFunctionCode(functionPath, projectionCode());

safeAddLine(subsystemPath, 'q/1', 'FR3 Stereo Projection/1');
for index = 1:size(constants, 1)
    safeAddLine(subsystemPath, ...
        [constants{index,1} '/1'], ...
        sprintf('FR3 Stereo Projection/%d', index+1));
end

outputNames = {
    'visionFeatureRaw'
    'centerErrorPx'
    'targetDepthM'
    'visionValid'
};
outputDimensions = {'[8 1]'; '[2 1]'; '1'; '1'};
outputY = [90 155 220 285];
for index = 1:4
    addOutport(subsystemPath, ...
        outputNames{index}, index, outputDimensions{index}, ...
        [635 outputY(index) 665 outputY(index)+20]);
    safeAddLine(subsystemPath, ...
        sprintf('FR3 Stereo Projection/%d', index), ...
        sprintf('%s/1', outputNames{index}));
end
end


function subsystemPath = addCoreInputSignals(modelName)
% 代码作用：
% 生成固定焦距、每周期新测量、使能和复位信号。

subsystemPath = [modelName '/Core Input Signals'];
createEmptySubsystem(subsystemPath, [390 455 690 665]);

specs = {
    'Focal Length Source', 'Focal Length', ...
        'cfg.simV21FocalLengthMm', '[2 1]'
    'Vision Measurement New Source', 'Vision Measurement New', '1', '1'
    'Focal Length Is New Source', 'Focal Length Is New', '1', '1'
    'Reset Source', 'Reset', '0', '1'
    'Left Measurement New Source', 'Left Measurement New', '1', '1'
    'Stereo Measurement New Source', 'Stereo Measurement New', '1', '1'
};

for index = 1:size(specs, 1)
    y = 25 + 38*(index-1);
    add_block('simulink/Sources/Constant', ...
        [subsystemPath '/' specs{index,1}], ...
        'Value', specs{index,3}, ...
        'SampleTime', 'cfg.Ts', ...
        'VectorParams1D', 'off', ...
        'Position', [25 y 185 y+24]);
    addOutport(subsystemPath, specs{index,2}, ...
        index, specs{index,4}, [385 y+4 415 y+18]);
    safeAddLine(subsystemPath, ...
        [specs{index,1} '/1'], ...
        sprintf('%s/1', specs{index,2}));
end

add_block('simulink/Sources/Step', ...
    [subsystemPath '/Controller Enable Source'], ...
    'Time', '0.5', ...
    'Before', '0', ...
    'After', '1', ...
    'SampleTime', 'cfg.Ts', ...
    'Position', [225 250 335 280]);
addOutport(subsystemPath, ...
    'Controller Enable', 7, '1', [385 255 415 269]);
safeAddLine(subsystemPath, ...
    'Controller Enable Source/1', 'Controller Enable/1');
end


function corePath = addCoreReference(modelName, coreModelName)
% 代码作用：
% 创建并检查10输入、3输出的Core模型引用。

corePath = [modelName '/Stereo IBVS Core'];
add_block('simulink/Ports & Subsystems/Model', ...
    corePath, ...
    'ModelName', coreModelName, ...
    'SimulationMode', 'Normal', ...
    'Position', [825 135 1085 535]);
Simulink.BlockDiagram.refreshBlocks(modelName);
handles = get_param(corePath, 'PortHandles');
if numel(handles.Inport) ~= 10 || numel(handles.Outport) ~= 3
    error('StereoIBVSSimV21:CorePortMismatch', ...
        'Core接口应为10输入3输出，实际为%d输入%d输出。', ...
        numel(handles.Inport), numel(handles.Outport));
end
end


function addLogging(modelName)
% 代码作用：
% 添加收敛判断需要的工作区记录，不额外生成绘图文件。

logs = {
    'Log Joint Position', 'simV21JointPosition', [1180 120 1360 150]
    'Log Joint Velocity', 'simV21JointVelocity', [1180 175 1360 205]
    'Log Center Error', 'simV21CenterErrorPx', [1180 230 1360 260]
    'Log Target Depth', 'simV21TargetDepthM', [1180 285 1360 315]
    'Log Vision Valid', 'simV21VisionValid', [1180 340 1360 370]
    'Log Controller Status', 'simV21ControllerStatus', [1180 395 1360 425]
};
for index = 1:size(logs, 1)
    add_block('simulink/Sinks/To Workspace', ...
        [modelName '/' logs{index,1}], ...
        'VariableName', logs{index,2}, ...
        'SaveFormat', 'Timeseries', ...
        'MaxDataPoints', 'inf', ...
        'Decimation', '1', ...
        'Position', logs{index,3});
end
end


function connectClosedLoop(modelName, plantPath, cameraPath, inputPath, corePath)
% 代码作用：
% 连接虚拟机械臂、相机、Core和记录模块，形成离散闭环。

% Plant q -> Camera和Core qRaw
connectPorts(modelName, plantPath, 1, cameraPath, 1);
connectPorts(modelName, plantPath, 1, corePath, 1);
connectPorts(modelName, plantPath, 2, corePath, 8);

% Camera视觉特征 -> Core
connectPorts(modelName, cameraPath, 1, corePath, 2);

% 固定与事件输入 -> Core
inputToCore = [3 4 5 7 9 10];
for index = 1:6
    connectPorts(modelName, inputPath, index, ...
        corePath, inputToCore(index));
end
connectPorts(modelName, inputPath, 7, corePath, 6);

% Core qDot -> Plant，Unit Delay由Plant积分状态打断代数环。
connectPorts(modelName, corePath, 1, plantPath, 1);

% 记录
connectPorts(modelName, plantPath, 1, ...
    [modelName '/Log Joint Position'], 1);
connectPorts(modelName, corePath, 1, ...
    [modelName '/Log Joint Velocity'], 1);
connectPorts(modelName, cameraPath, 2, ...
    [modelName '/Log Center Error'], 1);
connectPorts(modelName, cameraPath, 3, ...
    [modelName '/Log Target Depth'], 1);
connectPorts(modelName, cameraPath, 4, ...
    [modelName '/Log Vision Valid'], 1);
connectPorts(modelName, corePath, 3, ...
    [modelName '/Log Controller Status'], 1);

% Zoom输出在本版本关闭，但用Terminator显式处理。
add_block('simulink/Sinks/Terminator', ...
    [modelName '/Unused Focal Rate'], ...
    'Position', [1160 490 1180 510]);
connectPorts(modelName, corePath, 2, ...
    [modelName '/Unused Focal Rate'], 1);
end


function code = projectionCode()
% 代码作用：
% 返回虚拟FR3双目投影MATLAB Function源码。

code = strjoin({ ...
    'function [visionFeature,centerErrorPx,targetDepthM,visionValid] = fcn( ...'
    '    q,targetWorld,focalLengthMm,originXYZ,originRPY,jointAxis, ...'
    '    joint8XYZ,joint8RPY,TLink8CL,TCLCR,TWB,cameraParameters)'
    '%#codegen'
    '% 代码作用：计算FR3左右相机位姿，并将固定世界目标投影为双目像素特征。'
    'visionFeature = zeros(8,1);'
    'centerErrorPx = zeros(2,1);'
    'targetDepthM = 0;'
    'visionValid = 0;'
    'qSafe = reshape(q,7,1);'
    'targetW = reshape(targetWorld,3,1);'
    'focalMm = reshape(focalLengthMm,2,1);'
    'if ~all(isfinite(qSafe)) || ~all(isfinite(targetW)) || ...'
    '        ~all(isfinite(focalMm)) || any(focalMm <= 0)'
    '    return;'
    'end'
    'camera = reshape(cameraParameters,9,1);'
    'if ~all(isfinite(camera)) || any(camera(1:2) <= 0) || ...'
    '        any(camera(7:9) <= 0)'
    '    return;'
    'end'
    'T = reshape(TWB,4,4);'
    'for jointIndex = 1:7'
    '    T = T * makeTransform(originXYZ(:,jointIndex),originRPY(:,jointIndex));'
    '    T = T * axisRotation(jointAxis(:,jointIndex),qSafe(jointIndex));'
    'end'
    'TWL8 = T * makeTransform(joint8XYZ,joint8RPY);'
    'TWCL = TWL8 * reshape(TLink8CL,4,4);'
    'TWCR = TWCL * reshape(TCLCR,4,4);'
    'pCL = TWCL(1:3,1:3)'' * (targetW-TWCL(1:3,4));'
    'pCR = TWCR(1:3,1:3)'' * (targetW-TWCR(1:3,4));'
    'targetDepthM = pCL(3);'
    'pitchX = camera(1); pitchY = camera(2);'
    'fxL = focalMm(1)/pitchX; fyL = focalMm(1)/pitchY;'
    'fxR = focalMm(2)/pitchX; fyR = focalMm(2)/pitchY;'
    'cxL = camera(3); cyL = camera(4);'
    'cxR = camera(5); cyR = camera(6);'
    'imageWidth = camera(7); imageHeight = camera(8);'
    'tagSize = camera(9);'
    'if pCL(3) <= 0.05 || pCR(3) <= 0.05'
    '    return;'
    'end'
    'uL = cxL + fxL*pCL(1)/pCL(3);'
    'vL = cyL + fyL*pCL(2)/pCL(3);'
    'uR = cxR + fxR*pCR(1)/pCR(3);'
    'vR = cyR + fyR*pCR(2)/pCR(3);'
    'scaleL = fxL*tagSize/pCL(3);'
    'scaleR = fxR*tagSize/pCR(3);'
    'inside = uL >= 0 && uL <= imageWidth && ...'
    '    vL >= 0 && vL <= imageHeight && ...'
    '    uR >= 0 && uR <= imageWidth && ...'
    '    vR >= 0 && vR <= imageHeight;'
    'if ~inside || ~all(isfinite([uL;vL;uR;vR;scaleL;scaleR]))'
    '    return;'
    'end'
    'visionValid = 1;'
    'visionFeature = [1;1;uL;vL;uR;vR;scaleL;scaleR];'
    'centerErrorPx = [uL-cxL;vL-cyL];'
    'end'
    ''
    'function T = makeTransform(xyz,rpy)'
    '% 代码作用：由平移和固定轴RPY角生成齐次变换。'
    'roll=rpy(1); pitch=rpy(2); yaw=rpy(3);'
    'cr=cos(roll); sr=sin(roll); cp=cos(pitch); sp=sin(pitch);'
    'cy=cos(yaw); sy=sin(yaw);'
    'R=[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr; ...'
    '   sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr; ...'
    '   -sp,cp*sr,cp*cr];'
    'T=[R,reshape(xyz,3,1);0 0 0 1];'
    'end'
    ''
    'function T = axisRotation(axisLocal,angle)'
    '% 代码作用：使用Rodrigues公式生成关节轴旋转齐次变换。'
    'axisSafe=reshape(axisLocal,3,1);'
    'axisSafe=axisSafe/max(norm(axisSafe),1e-12);'
    'K=[0,-axisSafe(3),axisSafe(2);axisSafe(3),0,-axisSafe(1); ...'
    '   -axisSafe(2),axisSafe(1),0];'
    'R=eye(3)+sin(angle)*K+(1-cos(angle))*(K*K);'
    'T=[R,zeros(3,1);0 0 0 1];'
    'end' ...
    }, newline);
end


function setMatlabFunctionCode(blockPath, code)
% 代码作用：
% 将源码写入MATLAB Function模块。

root = sfroot;
chart = root.find('-isa', 'Stateflow.EMChart', 'Path', blockPath);
if isempty(chart)
    error('StereoIBVSSimV21:ChartNotFound', ...
        '找不到MATLAB Function模块：%s', blockPath);
end
chart.Script = code;
end


function createEmptySubsystem(path, position)
% 代码作用：
% 创建并清空子系统。

add_block('simulink/Ports & Subsystems/Subsystem', ...
    path, 'Position', position);
Simulink.SubSystem.deleteContents(path);
end


function addInport(parent, name, port, dimensions, position)
% 代码作用：
% 创建指定尺寸的输入端口。

add_block('simulink/Ports & Subsystems/In1', ...
    [parent '/' name], ...
    'Port', num2str(port), ...
    'PortDimensions', dimensions, ...
    'Position', position);
end


function addOutport(parent, name, port, dimensions, position)
% 代码作用：
% 创建指定尺寸的输出端口。

add_block('simulink/Ports & Subsystems/Out1', ...
    [parent '/' name], ...
    'Port', num2str(port), ...
    'PortDimensions', dimensions, ...
    'Position', position);
end


function connectPorts(rootModel, sourcePath, sourcePort, destinationPath, destinationPort)
% 代码作用：
% 使用端口句柄连接顶层或跨子系统信号。

sourceHandles = get_param(sourcePath, 'PortHandles');
destinationHandles = get_param(destinationPath, 'PortHandles');
add_line(rootModel, ...
    sourceHandles.Outport(sourcePort), ...
    destinationHandles.Inport(destinationPort), ...
    'autorouting', 'on');
end


function safeAddLine(parent, source, destination)
% 代码作用：
% 在子系统内部建立连线并输出明确错误。

try
    add_line(parent, source, destination, 'autorouting', 'on');
catch ME
    error('StereoIBVSSimV21:LineFailed', ...
        '连接失败：%s -> %s\n%s', source, destination, ME.message);
end
end


function validateConfiguration(cfgLocal)
% 代码作用：
% 检查V2.1闭环仿真依赖的配置字段和机器人模型。

required = {
    'robot'; 'Ts'; 'q0'; 'qMin'; 'qMax'; 'cameraBodyName';
    'fr3OriginXYZ'; 'fr3OriginRPY'; 'fr3Axis';
    'fr3Joint8OriginXYZ'; 'fr3Joint8OriginRPY';
    'T_link8_CL'; 'T_CL_CR'
};
for index = 1:numel(required)
    if ~isfield(cfgLocal, required{index})
        error('StereoIBVSSimV21:MissingCfgField', ...
            'cfg缺少字段：%s', required{index});
    end
end
if numel(cfgLocal.q0) ~= 7
    error('StereoIBVSSimV21:Q0Size', 'cfg.q0必须包含7个关节角。');
end
end


function [coreFile, configFile, simDir] = locateProjectFiles(projectDir)
% 代码作用：
% 自动定位simulink/core、config和sim目录。

scriptDir = fileparts(mfilename('fullpath'));
candidates = {};
if ~isempty(projectDir)
    candidates{end+1,1} = char(projectDir);
end
candidates = [candidates; {
    scriptDir
    fileparts(scriptDir)
    fileparts(fileparts(scriptDir))
    fileparts(fileparts(fileparts(scriptDir)))
    pwd
}];

coreFile = ''; %#ok<NASGU>
configFile = ''; %#ok<NASGU>
simDir = ''; %#ok<NASGU>
for index = 1:numel(candidates)
    root = candidates{index};
    if isempty(root) || ~isfolder(root)
        continue;
    end
    layouts = {
        fullfile(root, 'simulink')
        root
    };
    for layoutIndex = 1:numel(layouts)
        simulinkDir = layouts{layoutIndex};
        coreCandidate = fullfile( ...
            simulinkDir, 'core', 'stereo_ibvs_core.slx');
        configCandidate = fullfile( ...
            simulinkDir, 'config', 'stereo_ibvs_config.m');
        if isfile(coreCandidate) && isfile(configCandidate)
            coreFile = coreCandidate;
            configFile = configCandidate;
            simDir = fullfile(simulinkDir, 'sim');
            if ~isfolder(simDir)
                mkdir(simDir);
            end
            return;
        end
    end
end
error('StereoIBVSSimV21:ProjectNotFound', ...
    '无法定位stereo_ibvs_core.slx和stereo_ibvs_config.m。');
end


function result = samePath(pathA, pathB)
% 代码作用：
% 判断两个路径是否指向同一文件。

canonicalA = char(java.io.File(pathA).getCanonicalPath());
canonicalB = char(java.io.File(pathB).getCanonicalPath());
result = strcmp(canonicalA, canonicalB);
end
