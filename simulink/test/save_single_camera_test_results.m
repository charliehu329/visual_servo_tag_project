%% save_single_camera_ekf_tuning_results.m
% 单目XY视觉伺服：EKF调参数据保存与可视化脚本
%
% 使用方法：
% 1. 先运行初始化脚本并完成一次Simulink测试；
% 2. 停止仿真后，在MATLAB命令窗口设置：
%
%       testName = 'ekf_static_01';
%       testType = 'static';
%       save_single_camera_ekf_tuning_results
%
% testType可选：
%   'static'        机器人不动、目标不动：主要测R和静态速度噪声
%   'moving_target' 机器人不动、目标移动：主要判断Q造成的滞后/抖动
%   'moving_camera' 目标不动、机器人移动：主要检查相机运动补偿
%   'closed_loop'   正常闭环跟踪：评价EKF前馈是否改善控制
%   'generic'       通用分析
%
% 本脚本不会自动修改P0、Q、R或NIS门限，只负责：
% - 保存完整测试数据与参数快照；
% - 正确统计"新视觉帧中的测量接受率"；
% - 统计NIS分布；
% - 静态试验中估算视觉测量标准差与R；
% - 绘制EKF状态、协方差、相机速度补偿和控制器分量；
% - 生成summary.txt，便于不同参数测试之间比较。

%% 1. 用户设置

if ~exist('testName','var') || isempty(testName)
    testName = 'single_camera_ekf_test';
end

if ~exist('testType','var') || isempty(testType)
    testType = 'generic';
end

validTestTypes = { ...
    'static', ...
    'moving_target', ...
    'moving_camera', ...
    'closed_loop', ...
    'generic'};

testType = lower(char(testType));

if ~ismember(testType,validTestTypes)
    error( ...
        'testType必须是：%s', ...
        strjoin(validTestTypes,', '));
end

% 忽略开始阶段的初始化瞬态。
analysisStartSec = 1.0;

% 仅用于从位置测量离线估计粗略速度参考，不参与控制。
offlineVelocitySmoothingSec = 0.15;

showFigures = true;
closeFiguresAfterSave = false;
saveCompleteSimulationOutput = true;

resultsRoot = fullfile(pwd,'test_results');

%% 2. 创建本次测试文件夹

safeTestName = regexprep(char(testName),'[^a-zA-Z0-9_\-]','_');
safeTestType = regexprep(char(testType),'[^a-zA-Z0-9_\-]','_');
timeStamp = char(datetime('now','Format','yyyyMMdd_HHmmss'));

runFolderName = sprintf( ...
    '%s_%s_%s', ...
    timeStamp, ...
    safeTestType, ...
    safeTestName);

runFolder = fullfile(resultsRoot,runFolderName);
figureFolder = fullfile(runFolder,'figures');

if ~exist(resultsRoot,'dir')
    mkdir(resultsRoot);
end

mkdir(runFolder);
mkdir(figureFolder);

fprintf('\n============================================\n');
fprintf('开始保存EKF调参测试：%s\n',safeTestName);
fprintf('测试类型：%s\n',safeTestType);
fprintf('结果目录：%s\n',runFolder);
fprintf('============================================\n');

%% 3. 查找Simulink SimulationOutput

simulationOutput = [];
simulationOutputName = '';

candidateOutputNames = {'out','simOut','ans'};

for candidateIndex = 1:numel(candidateOutputNames)
    candidateName = candidateOutputNames{candidateIndex};

    variableExists = evalin( ...
        'base', ...
        sprintf('exist(''%s'',''var'')',candidateName));

    if variableExists
        candidateValue = evalin('base',candidateName);

        if isa(candidateValue,'Simulink.SimulationOutput')
            simulationOutput = candidateValue;
            simulationOutputName = candidateName;
            break;
        end
    end
end

if isempty(simulationOutput)
    fprintf('未找到SimulationOutput，将读取基础工作区中的log_*变量。\n');
else
    fprintf('找到SimulationOutput：%s\n',simulationOutputName);
end

%% 4. 读取关键日志
%
% 上传模型中已经包含以下调参关键日志：
% - 视觉测量：log_z_meas、log_error/log_e
% - EKF：log_ekf_state、log_ekf_covariance、log_ekf_velocity
% - 新帧/NIS：log_measurement_is_new、log_innovation_nis、
%              log_measurement_accepted
% - 相机运动补偿：log_v_camera_measured、
%                  log_v_camera_measured_valid
% - 控制器：v_p、v_ff、v_adapt、v_norm_limited、v_issued等

requestedLogs = { ...
    'log_z_meas', ...
    'log_error', ...
    'log_e', ...
    'log_ekf_state', ...
    'log_ekf_covariance', ...
    'log_ekf_velocity', ...
    'log_v_p', ...
    'log_v_ff', ...
    'log_v_adapt', ...
    'log_v_norm_limited', ...
    'log_v_issued', ...
    'log_camera_velocity', ...
    'log_v_camera_measured', ...
    'log_v_camera_measured_valid', ...
    'log_innovation_nis', ...
    'log_measurement_is_new', ...
    'log_measurement_accepted', ...
    'log_saturation', ...
    'log_acceleration_limit', ...
    'log_safe_valid', ...
    'log_controller_ok', ...
    'log_ekf_initialized', ...
    'log_u', ...
    'log_v', ...
    'log_joint_position', ...
    'log_joint_velocity'};

logs = struct();
missingLogs = {};

for logIndex = 1:numel(requestedLogs)
    logName = requestedLogs{logIndex};

    signal = getLoggedSignal( ...
        logName, ...
        simulationOutput);

    if isempty(signal)
        missingLogs{end+1} = logName; %#ok<SAGROW>
    else
        logs.(logName) = signal;
    end
end

% log_error与log_e通常是同一个误差信号，自动建立兼容别名。
if ~isfield(logs,'log_error') && isfield(logs,'log_e')
    logs.log_error = logs.log_e;
    missingLogs(strcmp(missingLogs,'log_error')) = [];
end

availableLogNames = fieldnames(logs);

if isempty(availableLogNames)
    error([ ...
        '没有找到任何所需日志。请确认仿真已经结束，并检查To Workspace块、' ...
        'Signal Logging或SimulationOutput中的变量名称。']);
end

fprintf('找到%d个关键日志。\n',numel(availableLogNames));

if ~isempty(missingLogs)
    fprintf('以下日志未找到，将跳过相关分析：\n');
    fprintf('  %s\n',strjoin(missingLogs,', '));
end

%% 5. 保存控制和滤波参数快照

parameterNames = { ...
    'Ts', ...
    'T_end', ...
    'fx', ...
    'fy', ...
    'cx', ...
    'cy', ...
    'Z_hat', ...
    'rho_hat', ...
    'Kpx', ...
    'Kpy', ...
    'k_ff', ...
    'gamma_adapt', ...
    'sigma_adapt', ...
    'adapt_max', ...
    'enable_proportional', ...
    'enable_ekf_feedforward', ...
    'enable_adaptation', ...
    'controller_enable', ...
    'USE_ROS', ...
    'v_xy_max', ...
    'a_xy_max', ...
    'P0', ...
    'Q_ekf', ...
    'R_ekf', ...
    'ekf_gate_threshold', ...
    'ekf_reset_timeout_sec', ...
    'target_timeout_sec', ...
    'joint_state_timeout_sec', ...
    'camera_velocity_feedback_timeout_sec', ...
    'controller_parameters'};

parameters = struct();

for parameterIndex = 1:numel(parameterNames)
    parameterName = parameterNames{parameterIndex};

    variableExists = evalin( ...
        'base', ...
        sprintf('exist(''%s'',''var'')',parameterName));

    if variableExists
        parameters.(parameterName) = evalin( ...
            'base', ...
            parameterName);
    end
end

%% 6. 计算统一分析指标

analysisOptions = struct();
analysisOptions.testType = testType;
analysisOptions.analysisStartSec = analysisStartSec;
analysisOptions.offlineVelocitySmoothingSec = ...
    offlineVelocitySmoothingSec;

metrics = calculateMetrics( ...
    logs, ...
    parameters, ...
    analysisOptions);

%% 7. 保存MAT数据

save( ...
    fullfile(runFolder,'selected_logs_parameters_metrics.mat'), ...
    'logs', ...
    'parameters', ...
    'metrics', ...
    'missingLogs', ...
    'safeTestName', ...
    'safeTestType', ...
    'timeStamp', ...
    'analysisOptions', ...
    '-v7.3');

if saveCompleteSimulationOutput && ~isempty(simulationOutput)
    save( ...
        fullfile(runFolder,'complete_simulation_output.mat'), ...
        'simulationOutput', ...
        '-v7.3');
end

%% 8. 保存初始化文件和YAML配置副本

copyConfigurationFiles(runFolder);

%% 9. 绘制图像

visibilityValue = 'off';

if showFigures
    visibilityValue = 'on';
end

createdFigures = {};

% ---------------------------------------------------------
% 图1：归一化误差与像素误差
% ---------------------------------------------------------
if isfield(logs,'log_error')
    [time,errorData] = signalToTimeData(logs.log_error);

    fig = figure('Visible',visibilityValue);
    tiledlayout(2,1,'TileSpacing','compact');

    nexttile;
    plot(time,errorData,'LineWidth',1.1);
    grid on;
    xlabel('Time (s)');
    ylabel('Normalized error');
    legend(makeChannelLabels('e',size(errorData,2)),'Location','best');
    title('Normalized Image Error');

    nexttile;

    pixelError = errorData;

    if isfield(parameters,'fx') && size(pixelError,2) >= 1
        pixelError(:,1) = pixelError(:,1)*parameters.fx;
    end

    if isfield(parameters,'fy') && size(pixelError,2) >= 2
        pixelError(:,2) = pixelError(:,2)*parameters.fy;
    end

    plot(time,pixelError,'LineWidth',1.1);
    grid on;
    xlabel('Time (s)');
    ylabel('Pixel error (px)');
    legend(makeChannelLabels('e_{px}',size(pixelError,2)),'Location','best');
    title('Image Error in Pixels');

    saveTestFigure(fig,figureFolder,'01_image_error_normalized_and_pixels');
    createdFigures{end+1} = ...
        '01_image_error_normalized_and_pixels'; %#ok<SAGROW>

    if closeFiguresAfterSave
        close(fig);
    end
end

% ---------------------------------------------------------
% 图2：视觉位置测量与EKF位置状态
% ---------------------------------------------------------
if isfield(logs,'log_z_meas') || isfield(logs,'log_ekf_state')
    fig = figure('Visible',visibilityValue);
    tiledlayout(2,1,'TileSpacing','compact');

    nexttile;
    if isfield(logs,'log_z_meas')
        [tz,zData] = signalToTimeData(logs.log_z_meas);
        plot(tz,zData(:,1:min(2,size(zData,2))),'LineWidth',1.1);
        legend({'z_{meas,x}','z_{meas,y}'},'Location','best');
    else
        text(0.1,0.5,'log\_z\_meas unavailable');
    end
    grid on;
    xlabel('Time (s)');
    ylabel('Relative position (m)');
    title('Visual Position Measurement');

    nexttile;
    if isfield(logs,'log_ekf_state')
        [tx,xData] = signalToTimeData(logs.log_ekf_state);
        plot(tx,xData(:,1:min(2,size(xData,2))),'LineWidth',1.1);
        legend({'\hat p_x','\hat p_y'},'Location','best');
    else
        text(0.1,0.5,'log\_ekf\_state unavailable');
    end
    grid on;
    xlabel('Time (s)');
    ylabel('Fixed-frame position (m)');
    title('EKF Position State');

    saveTestFigure(fig,figureFolder,'02_measurement_and_ekf_position');
    createdFigures{end+1} = ...
        '02_measurement_and_ekf_position'; %#ok<SAGROW>

    if closeFiguresAfterSave
        close(fig);
    end
end

% ---------------------------------------------------------
% 图3：EKF目标速度及其2-sigma不确定度
% ---------------------------------------------------------
if isfield(logs,'log_ekf_velocity')
    [tv,velocityData] = signalToTimeData(logs.log_ekf_velocity);

    sigmaVelocity = [];

    if isfield(logs,'log_ekf_covariance')
        [tp,pDiag] = covarianceToDiagonal( ...
            logs.log_ekf_covariance, ...
            6);

        if size(pDiag,2) >= 4
            sigmaVelocity = sqrt(max(pDiag(:,3:4),0));
            sigmaVelocity = interpolateData( ...
                tp, ...
                sigmaVelocity, ...
                tv, ...
                'linear');
        end
    end

    fig = figure('Visible',visibilityValue);
    tiledlayout(2,1,'TileSpacing','compact');

    axisLabels = {'X','Y'};

    for axisIndex = 1:min(2,size(velocityData,2))
        nexttile;
        plot(tv,velocityData(:,axisIndex),'LineWidth',1.2);
        hold on;

        if ~isempty(sigmaVelocity)
            plot( ...
                tv, ...
                velocityData(:,axisIndex) + ...
                    2*sigmaVelocity(:,axisIndex), ...
                '--', ...
                'LineWidth',0.9);

            plot( ...
                tv, ...
                velocityData(:,axisIndex) - ...
                    2*sigmaVelocity(:,axisIndex), ...
                '--', ...
                'LineWidth',0.9);

            legend( ...
                sprintf('\\hat v_%s',lower(axisLabels{axisIndex})), ...
                '+2\sigma', ...
                '-2\sigma', ...
                'Location','best');
        else
            legend( ...
                sprintf('\\hat v_%s',lower(axisLabels{axisIndex})), ...
                'Location','best');
        end

        yline(0,':');
        grid on;
        xlabel('Time (s)');
        ylabel('Velocity (m/s)');
        title(sprintf('EKF Target Velocity - %s',axisLabels{axisIndex}));
    end

    saveTestFigure(fig,figureFolder,'03_ekf_velocity_with_uncertainty');
    createdFigures{end+1} = ...
        '03_ekf_velocity_with_uncertainty'; %#ok<SAGROW>

    if closeFiguresAfterSave
        close(fig);
    end
end

% ---------------------------------------------------------
% 图4：相机实测速度与反馈有效性
% ---------------------------------------------------------
if isfield(logs,'log_v_camera_measured')
    [tc,vcData] = signalToTimeData(logs.log_v_camera_measured);

    fig = figure('Visible',visibilityValue);
    tiledlayout(2,1,'TileSpacing','compact');

    nexttile;
    plot(tc,vcData(:,1:min(2,size(vcData,2))),'LineWidth',1.1);
    grid on;
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    legend({'v_{cam,x}','v_{cam,y}'},'Location','best');
    title('Measured Camera XY Velocity');

    nexttile;
    if isfield(logs,'log_v_camera_measured_valid')
        [tvalid,validData] = signalToTimeData( ...
            logs.log_v_camera_measured_valid);

        stairs(tvalid,validData(:,1),'LineWidth',1.1);
    else
        text(0.1,0.5,'log\_v\_camera\_measured\_valid unavailable');
    end
    ylim([-0.1 1.1]);
    grid on;
    xlabel('Time (s)');
    ylabel('Valid');
    title('Camera Velocity Feedback Validity');

    saveTestFigure(fig,figureFolder,'04_camera_velocity_feedback');
    createdFigures{end+1} = ...
        '04_camera_velocity_feedback'; %#ok<SAGROW>

    if closeFiguresAfterSave
        close(fig);
    end
end

% ---------------------------------------------------------
% 图5：NIS时序、门限和分布
% ---------------------------------------------------------
if isfield(logs,'log_innovation_nis')
    [tnis,nisData] = signalToTimeData(logs.log_innovation_nis);
    nisVector = nisData(:,1);

    finiteMask = isfinite(nisVector);

    if isfield(logs,'log_measurement_is_new')
        newFrame = interpolateBooleanSignal( ...
            logs.log_measurement_is_new, ...
            tnis);

        finiteMask = finiteMask & newFrame;
    end

    if isfield(logs,'log_safe_valid')
        safeMask = interpolateBooleanSignal( ...
            logs.log_safe_valid, ...
            tnis);

        finiteMask = finiteMask & safeMask;
    end

    finiteNis = nisVector(finiteMask);

    fig = figure('Visible',visibilityValue);
    tiledlayout(2,1,'TileSpacing','compact');

    nexttile;
    plot(tnis,nisVector,'LineWidth',1.0);
    hold on;

    if isfield(parameters,'ekf_gate_threshold')
        yline(parameters.ekf_gate_threshold,'--','NIS gate');
    end

    grid on;
    xlabel('Time (s)');
    ylabel('NIS');
    title('NIS on New Valid Measurements');

    nexttile;
    if ~isempty(finiteNis)
        histogram( ...
            finiteNis, ...
            'Normalization','pdf', ...
            'NumBins',max(10,min(50,round(sqrt(numel(finiteNis))))));
        hold on;

        xMax = max(finiteNis);

        if isfield(parameters,'ekf_gate_threshold')
            xMax = max(xMax,1.5*parameters.ekf_gate_threshold);
            xline(parameters.ekf_gate_threshold,'--','NIS gate');
        end

        xGrid = linspace(0,max(xMax,1),300);

        % 二维测量时，理想NIS近似服从自由度2的卡方分布。
        % df=2的卡方概率密度为0.5*exp(-x/2)，无需统计工具箱。
        theoreticalPdf = 0.5*exp(-xGrid/2);
        plot(xGrid,theoreticalPdf,'LineWidth',1.2);

        legend('Measured NIS','NIS gate','\chi^2(2) reference', ...
            'Location','best');
    else
        text(0.1,0.5,'No finite NIS samples');
    end

    grid on;
    xlabel('NIS');
    ylabel('Probability density');
    title('NIS Distribution');

    saveTestFigure(fig,figureFolder,'05_nis_time_and_distribution');
    createdFigures{end+1} = ...
        '05_nis_time_and_distribution'; %#ok<SAGROW>

    if closeFiguresAfterSave
        close(fig);
    end
end

% ---------------------------------------------------------
% 图6：新帧、接受状态和安全状态
% ---------------------------------------------------------
statusSignals = { ...
    'log_measurement_is_new', ...
    'log_measurement_accepted', ...
    'log_safe_valid', ...
    'log_ekf_initialized', ...
    'log_controller_ok'};

availableStatusSignals = statusSignals(isfield(logs,statusSignals));

if ~isempty(availableStatusSignals)
    fig = figure('Visible',visibilityValue);
    hold on;

    statusLabels = cell(1,numel(availableStatusSignals));

    for statusIndex = 1:numel(availableStatusSignals)
        signalName = availableStatusSignals{statusIndex};
        [statusTime,statusData] = signalToTimeData(logs.(signalName));

        stairs( ...
            statusTime, ...
            statusData(:,1) + 1.2*(statusIndex-1), ...
            'LineWidth',1.0);

        statusLabels{statusIndex} = strrep(signalName,'log_','');
    end

    grid on;
    xlabel('Time (s)');
    ylabel('State with vertical offsets');
    legend(statusLabels,'Interpreter','none','Location','best');
    title('New Frame, Acceptance and Validity States');

    saveTestFigure(fig,figureFolder,'06_measurement_and_status_states');
    createdFigures{end+1} = ...
        '06_measurement_and_status_states'; %#ok<SAGROW>

    if closeFiguresAfterSave
        close(fig);
    end
end

% ---------------------------------------------------------
% 图7：EKF各状态标准差
% ---------------------------------------------------------
if isfield(logs,'log_ekf_covariance')
    [tp,pDiag] = covarianceToDiagonal( ...
        logs.log_ekf_covariance, ...
        6);

    stateSigma = sqrt(max(pDiag,0));

    fig = figure('Visible',visibilityValue);
    tiledlayout(3,1,'TileSpacing','compact');

    nexttile;
    plot(tp,stateSigma(:,1:2),'LineWidth',1.1);
    grid on;
    xlabel('Time (s)');
    ylabel('\sigma_p (m)');
    legend('\sigma_{p_x}','\sigma_{p_y}','Location','best');
    title('EKF Position Standard Deviation');

    nexttile;
    plot(tp,stateSigma(:,3:4),'LineWidth',1.1);
    grid on;
    xlabel('Time (s)');
    ylabel('\sigma_v (m/s)');
    legend('\sigma_{v_x}','\sigma_{v_y}','Location','best');
    title('EKF Velocity Standard Deviation');

    nexttile;
    plot(tp,stateSigma(:,5:6),'LineWidth',1.1);
    grid on;
    xlabel('Time (s)');
    ylabel('\sigma_a (m/s^2)');
    legend('\sigma_{a_x}','\sigma_{a_y}','Location','best');
    title('EKF Acceleration Standard Deviation');

    saveTestFigure(fig,figureFolder,'07_ekf_state_standard_deviation');
    createdFigures{end+1} = ...
        '07_ekf_state_standard_deviation'; %#ok<SAGROW>

    if closeFiguresAfterSave
        close(fig);
    end
end

% ---------------------------------------------------------
% 图8：控制器X/Y分量
% ---------------------------------------------------------
controllerSignals = { ...
    'log_v_p', ...
    'log_v_ff', ...
    'log_v_adapt', ...
    'log_v_norm_limited', ...
    'log_v_issued'};

if all(isfield(logs,controllerSignals))
    [tp,vp] = signalToTimeData(logs.log_v_p);
    [tff,vff] = signalToTimeData(logs.log_v_ff);
    [ta,va] = signalToTimeData(logs.log_v_adapt);
    [tn,vn] = signalToTimeData(logs.log_v_norm_limited);
    [ti,vi] = signalToTimeData(logs.log_v_issued);

    fig = figure('Visible',visibilityValue);
    tiledlayout(2,1,'TileSpacing','compact');

    for axisIndex = 1:2
        nexttile;
        hold on;
        plot(tp,vp(:,axisIndex),'LineWidth',1.0);
        plot(tff,vff(:,axisIndex),'LineWidth',1.0);
        plot(ta,va(:,axisIndex),'LineWidth',1.0);
        plot(tn,vn(:,axisIndex),'LineWidth',1.1);
        plot(ti,vi(:,axisIndex),'LineWidth',1.3);
        yline(0,':');
        grid on;
        xlabel('Time (s)');
        ylabel('Velocity (m/s)');
        legend( ...
            'v_p', ...
            'v_{ff}', ...
            'v_{adapt}', ...
            'v_{norm-limited}', ...
            'v_{issued}', ...
            'Location','best');

        if axisIndex == 1
            title('X-axis Controller Components');
        else
            title('Y-axis Controller Components');
        end
    end

    saveTestFigure(fig,figureFolder,'08_controller_components_xy');
    createdFigures{end+1} = ...
        '08_controller_components_xy'; %#ok<SAGROW>

    if closeFiguresAfterSave
        close(fig);
    end
end

% ---------------------------------------------------------
% 图9：速度和加速度限幅
% ---------------------------------------------------------
if isfield(logs,'log_saturation') && ...
        isfield(logs,'log_acceleration_limit')

    [tsat,saturationData] = signalToTimeData(logs.log_saturation);
    [tacc,accelerationData] = signalToTimeData( ...
        logs.log_acceleration_limit);

    fig = figure('Visible',visibilityValue);
    stairs(tsat,saturationData(:,1),'LineWidth',1.1);
    hold on;
    stairs(tacc,accelerationData(:,1) + 1.2,'LineWidth',1.1);
    grid on;
    xlabel('Time (s)');
    ylabel('Active with vertical offset');
    legend( ...
        'Velocity saturation', ...
        'Acceleration limit + 1.2', ...
        'Location','best');
    title('Controller Limits');

    saveTestFigure(fig,figureFolder,'09_controller_limits');
    createdFigures{end+1} = ...
        '09_controller_limits'; %#ok<SAGROW>

    if closeFiguresAfterSave
        close(fig);
    end
end

% ---------------------------------------------------------
% 图10：静态视觉测量噪声
% 只有static试验才具有明确物理意义。
% ---------------------------------------------------------
if strcmp(testType,'static') && ...
        isfield(metrics,'staticMeasurement') && ...
        metrics.staticMeasurement.available

    measurement = metrics.staticMeasurement;

    fig = figure('Visible',visibilityValue);
    tiledlayout(1,2,'TileSpacing','compact');

    nexttile;
    scatter( ...
        1000*measurement.centeredPosition(:,1), ...
        1000*measurement.centeredPosition(:,2), ...
        12, ...
        'filled');
    axis equal;
    grid on;
    xlabel('\Delta X measurement (mm)');
    ylabel('\Delta Y measurement (mm)');
    title('Static Measurement Scatter');

    nexttile;
    histogram( ...
        1000*measurement.centeredPosition(:,1), ...
        'Normalization','pdf');
    hold on;
    histogram( ...
        1000*measurement.centeredPosition(:,2), ...
        'Normalization','pdf');
    grid on;
    xlabel('Centered measurement (mm)');
    ylabel('Probability density');
    legend('X','Y','Location','best');
    title('Static Measurement Noise Distribution');

    saveTestFigure(fig,figureFolder,'10_static_measurement_noise');
    createdFigures{end+1} = ...
        '10_static_measurement_noise'; %#ok<SAGROW>

    if closeFiguresAfterSave
        close(fig);
    end
end

% ---------------------------------------------------------
% 图11：EKF速度与离线粗略速度参考
%
% 参考速度由：
%   平滑后的z_meas差分 + 实测相机速度
% 得到。它只用于观察EKF滞后和尖峰，不是真值。
% ---------------------------------------------------------
if isfield(metrics,'velocityComparison') && ...
        metrics.velocityComparison.available

    comparison = metrics.velocityComparison;

    fig = figure('Visible',visibilityValue);
    tiledlayout(2,1,'TileSpacing','compact');

    for axisIndex = 1:2
        nexttile;
        plot( ...
            comparison.time, ...
            comparison.referenceVelocity(:,axisIndex), ...
            'LineWidth',1.0);
        hold on;
        plot( ...
            comparison.time, ...
            comparison.ekfVelocity(:,axisIndex), ...
            'LineWidth',1.2);
        yline(0,':');
        grid on;
        xlabel('Time (s)');
        ylabel('Velocity (m/s)');
        legend( ...
            'Offline rough reference', ...
            'EKF estimate', ...
            'Location','best');

        if axisIndex == 1
            title('Target Velocity Comparison - X');
        else
            title('Target Velocity Comparison - Y');
        end
    end

    saveTestFigure(fig,figureFolder,'11_velocity_reference_vs_ekf');
    createdFigures{end+1} = ...
        '11_velocity_reference_vs_ekf'; %#ok<SAGROW>

    if closeFiguresAfterSave
        close(fig);
    end
end

%% 10. 生成测试摘要

summaryFile = fullfile(runFolder,'summary.txt');

writeSummary( ...
    summaryFile, ...
    safeTestName, ...
    safeTestType, ...
    timeStamp, ...
    logs, ...
    parameters, ...
    metrics, ...
    missingLogs, ...
    createdFigures);

%% 11. 完成提示

fprintf('\n保存完成。\n');
fprintf('日志、参数与指标：%s\n', ...
    fullfile(runFolder,'selected_logs_parameters_metrics.mat'));

if saveCompleteSimulationOutput && ~isempty(simulationOutput)
    fprintf('完整仿真输出：%s\n', ...
        fullfile(runFolder,'complete_simulation_output.mat'));
end

fprintf('测试摘要：%s\n',summaryFile);
fprintf('图像目录：%s\n',figureFolder);
fprintf('============================================\n\n');

%% =========================================================
% 局部函数
% ==========================================================

function signal = getLoggedSignal(logName,simulationOutput)
% 优先从SimulationOutput中读取；找不到时读取基础工作区。

signal = [];

if ~isempty(simulationOutput)
    outputNames = simulationOutput.who;

    if any(strcmp(outputNames,logName))
        signal = simulationOutput.get(logName);
        return;
    end

    if any(strcmp(outputNames,'logsout'))
        logsout = simulationOutput.get('logsout');

        if isa(logsout,'Simulink.SimulationData.Dataset')
            try
                element = logsout.getElement(logName);

                if ~isempty(element)
                    signal = element.Values;
                    return;
                end
            catch
                % logsout中不存在该名称时继续搜索基础工作区。
            end
        end
    end
end

variableExists = evalin( ...
    'base', ...
    sprintf('exist(''%s'',''var'')',logName));

if variableExists
    signal = evalin('base',logName);
end

end


function [time,data] = signalToTimeData(signal)
% 将常见Simulink日志格式转换为：
% time：N×1
% data：N×通道数

if isa(signal,'Simulink.SimulationData.Signal')
    signal = signal.Values;
end

if isa(signal,'timeseries')
    time = double(signal.Time(:));
    data = signal.Data;

elseif isstruct(signal) && ...
        isfield(signal,'time') && ...
        isfield(signal,'signals')

    time = double(signal.time(:));
    data = signal.signals.values;

elseif istimetable(signal)
    time = seconds( ...
        signal.Properties.RowTimes - ...
        signal.Properties.RowTimes(1));

    data = signal.Variables;

else
    error( ...
        '不支持的日志格式：%s', ...
        class(signal));
end

data = squeeze(data);

if isvector(data)
    data = data(:);

elseif size(data,1) ~= numel(time) && ...
        size(data,2) == numel(time)

    data = data.';

elseif size(data,1) ~= numel(time)
    data = reshape(data,numel(time),[]);
end

data = double(data);

end


function [time,diagonalData] = covarianceToDiagonal(signal,stateCount)
% 将N个stateCount×stateCount协方差矩阵转换成N×stateCount对角线。

if isa(signal,'Simulink.SimulationData.Signal')
    signal = signal.Values;
end

if isa(signal,'timeseries')
    time = double(signal.Time(:));
    rawData = signal.Data;

elseif isstruct(signal) && ...
        isfield(signal,'time') && ...
        isfield(signal,'signals')

    time = double(signal.time(:));
    rawData = signal.signals.values;

elseif istimetable(signal)
    time = seconds( ...
        signal.Properties.RowTimes - ...
        signal.Properties.RowTimes(1));

    rawData = signal.Variables;

else
    error( ...
        '不支持的协方差日志格式：%s', ...
        class(signal));
end

sampleCount = numel(time);
diagonalData = nan(sampleCount,stateCount);

rawSize = size(rawData);

if ismatrix(rawData)
    matrixData = double(rawData);

    if size(matrixData,1) == sampleCount && ...
            size(matrixData,2) == stateCount*stateCount

        for sampleIndex = 1:sampleCount
            covarianceMatrix = reshape( ...
                matrixData(sampleIndex,:), ...
                stateCount, ...
                stateCount);

            diagonalData(sampleIndex,:) = diag(covarianceMatrix).';
        end

    elseif size(matrixData,2) == sampleCount && ...
            size(matrixData,1) == stateCount*stateCount

        matrixData = matrixData.';

        for sampleIndex = 1:sampleCount
            covarianceMatrix = reshape( ...
                matrixData(sampleIndex,:), ...
                stateCount, ...
                stateCount);

            diagonalData(sampleIndex,:) = diag(covarianceMatrix).';
        end
    else
        error( ...
            '无法识别协方差矩阵日志尺寸：%s', ...
            mat2str(rawSize));
    end

elseif ndims(rawData) == 3
    if rawSize(1) == stateCount && ...
            rawSize(2) == stateCount && ...
            rawSize(3) == sampleCount

        for sampleIndex = 1:sampleCount
            covarianceMatrix = double(rawData(:,:,sampleIndex));
            diagonalData(sampleIndex,:) = diag(covarianceMatrix).';
        end

    elseif rawSize(1) == sampleCount && ...
            rawSize(2) == stateCount && ...
            rawSize(3) == stateCount

        for sampleIndex = 1:sampleCount
            covarianceMatrix = double( ...
                squeeze(rawData(sampleIndex,:,:)));

            diagonalData(sampleIndex,:) = diag(covarianceMatrix).';
        end

    elseif rawSize(1) == stateCount && ...
            rawSize(2) == sampleCount && ...
            rawSize(3) == stateCount

        for sampleIndex = 1:sampleCount
            covarianceMatrix = double( ...
                squeeze(rawData(:,sampleIndex,:)));

            diagonalData(sampleIndex,:) = diag(covarianceMatrix).';
        end
    else
        error( ...
            '无法识别三维协方差日志尺寸：%s', ...
            mat2str(rawSize));
    end
else
    error( ...
        '无法识别协方差日志维度：%s', ...
        mat2str(rawSize));
end

end


function result = interpolateData( ...
    sourceTime, ...
    sourceData, ...
    queryTime, ...
    method)

sourceTime = double(sourceTime(:));
queryTime = double(queryTime(:));
sourceData = double(sourceData);

[sourceTime,uniqueIndex] = unique(sourceTime,'stable');
sourceData = sourceData(uniqueIndex,:);

if numel(sourceTime) < 2
    result = repmat(sourceData(1,:),numel(queryTime),1);
    return;
end

result = interp1( ...
    sourceTime, ...
    sourceData, ...
    queryTime, ...
    method, ...
    'extrap');

end


function logicalData = interpolateBooleanSignal(signal,queryTime)

[sourceTime,sourceData] = signalToTimeData(signal);

interpolatedData = interpolateData( ...
    sourceTime, ...
    sourceData(:,1), ...
    queryTime, ...
    'previous');

logicalData = interpolatedData > 0.5;

end


function metrics = calculateMetrics(logs,parameters,options)

metrics = struct();
metrics.testType = options.testType;
metrics.analysisStartSec = options.analysisStartSec;

% ---------------------------------------------------------
% 测试持续时间
% ---------------------------------------------------------
allLogNames = fieldnames(logs);
metrics.testDurationSec = NaN;

for index = 1:numel(allLogNames)
    try
        [time,~] = signalToTimeData(logs.(allLogNames{index}));

        if numel(time) >= 2
            metrics.testDurationSec = time(end) - time(1);
            break;
        end
    catch
        % 跳过无法转换的日志。
    end
end

% ---------------------------------------------------------
% 图像误差指标
% ---------------------------------------------------------
metrics.imageError = struct('available',false);

if isfield(logs,'log_error')
    [time,errorData] = signalToTimeData(logs.log_error);

    mask = time >= time(1) + options.analysisStartSec;
    mask = mask & all(isfinite(errorData),2);

    if any(mask)
        selectedError = errorData(mask,:);

        metrics.imageError.available = true;
        metrics.imageError.rmsNormalized = sqrt( ...
            mean(selectedError.^2,1,'omitnan'));

        metrics.imageError.maxAbsNormalized = max( ...
            abs(selectedError),[],1,'omitnan');

        pixelScale = ones(1,size(selectedError,2));

        if isfield(parameters,'fx') && numel(pixelScale) >= 1
            pixelScale(1) = parameters.fx;
        end

        if isfield(parameters,'fy') && numel(pixelScale) >= 2
            pixelScale(2) = parameters.fy;
        end

        pixelError = selectedError.*pixelScale;

        metrics.imageError.rmsPixels = sqrt( ...
            mean(pixelError.^2,1,'omitnan'));

        metrics.imageError.maxAbsPixels = max( ...
            abs(pixelError),[],1,'omitnan');
    end
end

% ---------------------------------------------------------
% 新帧数量、有效帧数量和正确的接受率
% ---------------------------------------------------------
metrics.measurement = struct('available',false);

if isfield(logs,'log_measurement_is_new')
    [time,newData] = signalToTimeData( ...
        logs.log_measurement_is_new);

    analysisMask = time >= time(1) + options.analysisStartSec;
    newMask = newData(:,1) > 0.5;

    safeMask = true(size(newMask));

    if isfield(logs,'log_safe_valid')
        safeMask = interpolateBooleanSignal( ...
            logs.log_safe_valid, ...
            time);
    end

    acceptedMask = false(size(newMask));

    if isfield(logs,'log_measurement_accepted')
        acceptedMask = interpolateBooleanSignal( ...
            logs.log_measurement_accepted, ...
            time);
    end

    validNewMask = analysisMask & newMask & safeMask;
    acceptedNewMask = validNewMask & acceptedMask;

    metrics.measurement.available = true;
    metrics.measurement.controlSampleCount = sum(analysisMask);
    metrics.measurement.newFrameCount = sum(analysisMask & newMask);
    metrics.measurement.validNewFrameCount = sum(validNewMask);
    metrics.measurement.acceptedNewFrameCount = sum(acceptedNewMask);

    if metrics.measurement.validNewFrameCount > 0
        metrics.measurement.acceptanceRatioPercent = ...
            100*metrics.measurement.acceptedNewFrameCount / ...
            metrics.measurement.validNewFrameCount;
    else
        metrics.measurement.acceptanceRatioPercent = NaN;
    end

    if metrics.testDurationSec > 0
        metrics.measurement.newFrameRateHz = ...
            metrics.measurement.newFrameCount / ...
            max(metrics.testDurationSec - options.analysisStartSec,eps);
    else
        metrics.measurement.newFrameRateHz = NaN;
    end
end

% ---------------------------------------------------------
% NIS统计：只统计有效新帧上的有限NIS
% ---------------------------------------------------------
metrics.nis = struct('available',false);

if isfield(logs,'log_innovation_nis')
    [time,nisData] = signalToTimeData(logs.log_innovation_nis);
    nisVector = nisData(:,1);

    mask = time >= time(1) + options.analysisStartSec;
    mask = mask & isfinite(nisVector);

    if isfield(logs,'log_measurement_is_new')
        mask = mask & interpolateBooleanSignal( ...
            logs.log_measurement_is_new, ...
            time);
    end

    if isfield(logs,'log_safe_valid')
        mask = mask & interpolateBooleanSignal( ...
            logs.log_safe_valid, ...
            time);
    end

    selectedNis = nisVector(mask);

    if ~isempty(selectedNis)
        metrics.nis.available = true;
        metrics.nis.sampleCount = numel(selectedNis);
        metrics.nis.mean = mean(selectedNis,'omitnan');
        metrics.nis.median = median(selectedNis,'omitnan');
        metrics.nis.percentile95 = prctile(selectedNis,95);
        metrics.nis.percentile99 = prctile(selectedNis,99);

        if isfield(parameters,'ekf_gate_threshold')
            metrics.nis.gate = parameters.ekf_gate_threshold;
            metrics.nis.gateExceedancePercent = ...
                100*mean( ...
                    selectedNis > parameters.ekf_gate_threshold, ...
                    'omitnan');
        else
            metrics.nis.gate = NaN;
            metrics.nis.gateExceedancePercent = NaN;
        end
    end
end

% ---------------------------------------------------------
% EKF速度静态噪声
% ---------------------------------------------------------
metrics.ekfVelocity = struct('available',false);

if isfield(logs,'log_ekf_velocity')
    [time,velocityData] = signalToTimeData(logs.log_ekf_velocity);

    mask = time >= time(1) + options.analysisStartSec;
    mask = mask & all(isfinite(velocityData),2);

    if any(mask)
        selectedVelocity = velocityData(mask,:);

        metrics.ekfVelocity.available = true;
        metrics.ekfVelocity.mean = mean( ...
            selectedVelocity,1,'omitnan');

        metrics.ekfVelocity.meanAbs = mean( ...
            abs(selectedVelocity),1,'omitnan');

        metrics.ekfVelocity.rms = sqrt( ...
            mean(selectedVelocity.^2,1,'omitnan'));

        metrics.ekfVelocity.percentile95Abs = prctile( ...
            abs(selectedVelocity),95,1);

        metrics.ekfVelocity.maxAbs = max( ...
            abs(selectedVelocity),[],1,'omitnan');
    end
end

% ---------------------------------------------------------
% 静态测量噪声与R估计
% ---------------------------------------------------------
metrics.staticMeasurement = struct('available',false);

if strcmp(options.testType,'static') && ...
        isfield(logs,'log_z_meas')

    [time,zData] = signalToTimeData(logs.log_z_meas);

    mask = time >= time(1) + options.analysisStartSec;
    mask = mask & all(isfinite(zData(:,1:2)),2);

    if isfield(logs,'log_measurement_is_new')
        mask = mask & interpolateBooleanSignal( ...
            logs.log_measurement_is_new, ...
            time);
    end

    if isfield(logs,'log_safe_valid')
        mask = mask & interpolateBooleanSignal( ...
            logs.log_safe_valid, ...
            time);
    end

    selectedPosition = zData(mask,1:2);

    if size(selectedPosition,1) >= 10
        meanPosition = mean(selectedPosition,1,'omitnan');
        centeredPosition = selectedPosition - meanPosition;

        measurementCovariance = cov( ...
            selectedPosition, ...
            'omitrows');

        sigmaMeters = std( ...
            selectedPosition, ...
            0, ...
            1, ...
            'omitnan');

        sigmaPixels = nan(1,2);

        if isfield(parameters,'Z_hat') && ...
                parameters.Z_hat > 0

            if isfield(parameters,'fx')
                sigmaPixels(1) = ...
                    sigmaMeters(1)*parameters.fx/parameters.Z_hat;
            end

            if isfield(parameters,'fy')
                sigmaPixels(2) = ...
                    sigmaMeters(2)*parameters.fy/parameters.Z_hat;
            end
        end

        metrics.staticMeasurement.available = true;
        metrics.staticMeasurement.sampleCount = ...
            size(selectedPosition,1);

        metrics.staticMeasurement.meanPosition = meanPosition;
        metrics.staticMeasurement.centeredPosition = centeredPosition;
        metrics.staticMeasurement.sigmaMeters = sigmaMeters;
        metrics.staticMeasurement.sigmaPixels = sigmaPixels;
        metrics.staticMeasurement.covariance = measurementCovariance;
        metrics.staticMeasurement.suggestedR = measurementCovariance;
    end
end

% ---------------------------------------------------------
% 相机速度反馈有效率
% ---------------------------------------------------------
metrics.cameraVelocity = struct('available',false);

if isfield(logs,'log_v_camera_measured')
    [time,cameraVelocity] = signalToTimeData( ...
        logs.log_v_camera_measured);

    mask = time >= time(1) + options.analysisStartSec;
    mask = mask & all(isfinite(cameraVelocity),2);

    if any(mask)
        selectedVelocity = cameraVelocity(mask,:);

        metrics.cameraVelocity.available = true;
        metrics.cameraVelocity.rms = sqrt( ...
            mean(selectedVelocity.^2,1,'omitnan'));

        metrics.cameraVelocity.maxAbs = max( ...
            abs(selectedVelocity),[],1,'omitnan');

        if isfield(logs,'log_v_camera_measured_valid')
            validMask = interpolateBooleanSignal( ...
                logs.log_v_camera_measured_valid, ...
                time(mask));

            metrics.cameraVelocity.validRatioPercent = ...
                100*mean(validMask,'omitnan');
        else
            metrics.cameraVelocity.validRatioPercent = NaN;
        end
    end
end

% ---------------------------------------------------------
% EKF协方差指标
% ---------------------------------------------------------
metrics.covariance = struct('available',false);

if isfield(logs,'log_ekf_covariance')
    [time,pDiag] = covarianceToDiagonal( ...
        logs.log_ekf_covariance, ...
        6);

    mask = time >= time(1) + options.analysisStartSec;
    mask = mask & all(isfinite(pDiag),2);

    if any(mask)
        selectedDiag = pDiag(mask,:);
        sigmaData = sqrt(max(selectedDiag,0));

        metrics.covariance.available = true;
        metrics.covariance.meanSigma = mean( ...
            sigmaData,1,'omitnan');

        metrics.covariance.finalSigma = sigmaData(end,:);
        metrics.covariance.maxSigma = max( ...
            sigmaData,[],1,'omitnan');
    end
end

% ---------------------------------------------------------
% 控制器指标
% ---------------------------------------------------------
metrics.controller = struct('available',false);

if isfield(logs,'log_v_issued')
    [time,issuedVelocity] = signalToTimeData(logs.log_v_issued);

    mask = time >= time(1) + options.analysisStartSec;
    mask = mask & all(isfinite(issuedVelocity),2);

    if any(mask)
        selectedIssued = issuedVelocity(mask,:);
        issuedSpeed = sqrt(sum(selectedIssued.^2,2));

        metrics.controller.available = true;
        metrics.controller.maximumIssuedSpeed = max( ...
            issuedSpeed,[],1,'omitnan');

        metrics.controller.rmsIssuedSpeed = sqrt( ...
            mean(issuedSpeed.^2,'omitnan'));
    end
end

if isfield(logs,'log_saturation')
    [time,saturationData] = signalToTimeData(logs.log_saturation);
    mask = time >= time(1) + options.analysisStartSec;

    metrics.controller.velocitySaturationPercent = ...
        100*mean(saturationData(mask,1) > 0.5,'omitnan');
end

if isfield(logs,'log_acceleration_limit')
    [time,accelerationData] = signalToTimeData( ...
        logs.log_acceleration_limit);

    mask = time >= time(1) + options.analysisStartSec;

    metrics.controller.accelerationLimitPercent = ...
        100*mean(accelerationData(mask,1) > 0.5,'omitnan');
end

% 前馈与比例项的RMS比值。
if isfield(logs,'log_v_p') && isfield(logs,'log_v_ff')
    [tp,vp] = signalToTimeData(logs.log_v_p);
    [tff,vff] = signalToTimeData(logs.log_v_ff);

    vffOnVpTime = interpolateData(tff,vff,tp,'linear');

    mask = tp >= tp(1) + options.analysisStartSec;
    mask = mask & all(isfinite(vp),2) & all(isfinite(vffOnVpTime),2);

    if any(mask)
        vpNorm = sqrt(sum(vp(mask,:).^2,2));
        vffNorm = sqrt(sum(vffOnVpTime(mask,:).^2,2));

        rmsVp = sqrt(mean(vpNorm.^2,'omitnan'));
        rmsVff = sqrt(mean(vffNorm.^2,'omitnan'));

        metrics.controller.rmsVp = rmsVp;
        metrics.controller.rmsVff = rmsVff;
        metrics.controller.feedforwardToProportionalRmsRatio = ...
            rmsVff/max(rmsVp,eps);
    end
end

% ---------------------------------------------------------
% 离线粗略速度参考
% ---------------------------------------------------------
metrics.velocityComparison = struct('available',false);

requiredLogs = { ...
    'log_z_meas', ...
    'log_ekf_velocity', ...
    'log_measurement_is_new'};

if all(isfield(logs,requiredLogs))
    [tz,zData] = signalToTimeData(logs.log_z_meas);

    mask = tz >= tz(1) + options.analysisStartSec;
    mask = mask & all(isfinite(zData(:,1:2)),2);
    mask = mask & interpolateBooleanSignal( ...
        logs.log_measurement_is_new, ...
        tz);

    if isfield(logs,'log_safe_valid')
        mask = mask & interpolateBooleanSignal( ...
            logs.log_safe_valid, ...
            tz);
    end

    selectedTime = tz(mask);
    selectedPosition = zData(mask,1:2);

    if numel(selectedTime) >= 10
        [selectedTime,uniqueIndex] = unique( ...
            selectedTime, ...
            'stable');

        selectedPosition = selectedPosition(uniqueIndex,:);

        medianDt = median(diff(selectedTime),'omitnan');

        if isfinite(medianDt) && medianDt > 0
            smoothingSamples = max( ...
                3, ...
                round( ...
                    options.offlineVelocitySmoothingSec / ...
                    medianDt));

            if mod(smoothingSamples,2) == 0
                smoothingSamples = smoothingSamples + 1;
            end

            smoothedPosition = movmean( ...
                selectedPosition, ...
                smoothingSamples, ...
                1, ...
                'omitnan');

            relativeVelocity = zeros(size(smoothedPosition));

            relativeVelocity(:,1) = gradient( ...
                smoothedPosition(:,1), ...
                selectedTime);

            relativeVelocity(:,2) = gradient( ...
                smoothedPosition(:,2), ...
                selectedTime);

            measuredCameraVelocity = zeros(size(relativeVelocity));

            if isfield(logs,'log_v_camera_measured')
                [tc,vc] = signalToTimeData( ...
                    logs.log_v_camera_measured);

                measuredCameraVelocity = interpolateData( ...
                    tc, ...
                    vc(:,1:2), ...
                    selectedTime, ...
                    'linear');
            end

            referenceVelocity = ...
                relativeVelocity + measuredCameraVelocity;

            [tv,ekfVelocity] = signalToTimeData( ...
                logs.log_ekf_velocity);

            ekfVelocity = interpolateData( ...
                tv, ...
                ekfVelocity(:,1:2), ...
                selectedTime, ...
                'linear');

            finiteMask = ...
                all(isfinite(referenceVelocity),2) & ...
                all(isfinite(ekfVelocity),2);

            if sum(finiteMask) >= 10
                selectedTime = selectedTime(finiteMask);
                referenceVelocity = referenceVelocity(finiteMask,:);
                ekfVelocity = ekfVelocity(finiteMask,:);

                metrics.velocityComparison.available = true;
                metrics.velocityComparison.time = selectedTime;
                metrics.velocityComparison.referenceVelocity = ...
                    referenceVelocity;

                metrics.velocityComparison.ekfVelocity = ...
                    ekfVelocity;

                metrics.velocityComparison.rmse = sqrt( ...
                    mean( ...
                        (ekfVelocity-referenceVelocity).^2, ...
                        1, ...
                        'omitnan'));

                metrics.velocityComparison.meanAbsoluteError = mean( ...
                    abs(ekfVelocity-referenceVelocity), ...
                    1, ...
                    'omitnan');

                metrics.velocityComparison.note = [ ...
                    'Reference is computed from smoothed z_meas derivative ' ...
                    'plus measured camera velocity; it is not ground truth.'];
            end
        end
    end
end

end


function labels = makeChannelLabels(prefix,channelCount)

if channelCount == 2
    labels = { ...
        sprintf('%s_x',prefix), ...
        sprintf('%s_y',prefix)};
else
    labels = cell(1,channelCount);

    for channelIndex = 1:channelCount
        labels{channelIndex} = sprintf( ...
            '%s_%d', ...
            prefix, ...
            channelIndex);
    end
end

end


function saveTestFigure(fig,figureFolder,fileBaseName)

exportgraphics( ...
    fig, ...
    fullfile(figureFolder,[fileBaseName,'.png']), ...
    'Resolution',200);

savefig( ...
    fig, ...
    fullfile(figureFolder,[fileBaseName,'.fig']));

end


function copyConfigurationFiles(runFolder)

initCandidates = dir( ...
    fullfile(pwd,'**','init_single_camera_xy_tracking.m'));

if ~isempty(initCandidates)
    initSource = fullfile( ...
        initCandidates(1).folder, ...
        initCandidates(1).name);

    copyfile( ...
        initSource, ...
        fullfile(runFolder, ...
            'init_single_camera_xy_tracking_snapshot.m'));
end

homeDirectory = getenv('HOME');

yamlSearchRoot = fullfile( ...
    homeDirectory, ...
    'franka_ros2_ws', ...
    'src');

if exist(yamlSearchRoot,'dir')
    yamlCandidates = dir( ...
        fullfile( ...
            yamlSearchRoot, ...
            '**', ...
            'velocity_servo_tag.yaml'));

    if ~isempty(yamlCandidates)
        yamlSource = fullfile( ...
            yamlCandidates(1).folder, ...
            yamlCandidates(1).name);

        copyfile( ...
            yamlSource, ...
            fullfile(runFolder, ...
                'velocity_servo_tag_snapshot.yaml'));
    end
end

end


function writeSummary( ...
    summaryFile, ...
    testName, ...
    testType, ...
    timeStamp, ...
    logs, ...
    parameters, ...
    metrics, ...
    missingLogs, ...
    createdFigures)

fileID = fopen(summaryFile,'w');

if fileID < 0
    warning('无法创建测试摘要：%s',summaryFile);
    return;
end

cleanupObject = onCleanup(@() fclose(fileID)); %#ok<NASGU>

fprintf(fileID,'Single-camera XY EKF tuning test summary\n');
fprintf(fileID,'========================================\n');
fprintf(fileID,'Test name: %s\n',testName);
fprintf(fileID,'Test type: %s\n',testType);
fprintf(fileID,'Timestamp: %s\n',timeStamp);
fprintf(fileID,'Analysis start offset: %.3f s\n\n', ...
    metrics.analysisStartSec);

fprintf(fileID,'Available logs: %s\n', ...
    strjoin(fieldnames(logs),', '));

if isempty(missingLogs)
    fprintf(fileID,'Missing logs: none\n\n');
else
    fprintf(fileID,'Missing logs: %s\n\n', ...
        strjoin(missingLogs,', '));
end

fprintf(fileID,'Controller and EKF parameters\n');
fprintf(fileID,'-----------------------------\n');

scalarParameterNames = { ...
    'Ts', ...
    'fx', ...
    'fy', ...
    'cx', ...
    'cy', ...
    'Z_hat', ...
    'Kpx', ...
    'Kpy', ...
    'k_ff', ...
    'gamma_adapt', ...
    'sigma_adapt', ...
    'adapt_max', ...
    'v_xy_max', ...
    'a_xy_max', ...
    'ekf_gate_threshold', ...
    'ekf_reset_timeout_sec', ...
    'target_timeout_sec', ...
    'joint_state_timeout_sec', ...
    'camera_velocity_feedback_timeout_sec'};

for parameterIndex = 1:numel(scalarParameterNames)
    parameterName = scalarParameterNames{parameterIndex};

    if isfield(parameters,parameterName)
        fprintf( ...
            fileID, ...
            '%s: %.12g\n', ...
            parameterName, ...
            double(parameters.(parameterName)));
    end
end

matrixParameterNames = {'P0','Q_ekf','R_ekf'};

for parameterIndex = 1:numel(matrixParameterNames)
    parameterName = matrixParameterNames{parameterIndex};

    if isfield(parameters,parameterName)
        fprintf(fileID,'%s:\n',parameterName);
        writeMatrix(fileID,double(parameters.(parameterName)));
    end
end

fprintf(fileID,'\nMeasured indicators\n');
fprintf(fileID,'-------------------\n');

if isfinite(metrics.testDurationSec)
    fprintf(fileID,'Test duration: %.6f s\n', ...
        metrics.testDurationSec);
end

if isfield(metrics,'measurement') && metrics.measurement.available
    fprintf(fileID,'New frame count: %d\n', ...
        metrics.measurement.newFrameCount);

    fprintf(fileID,'Valid new frame count: %d\n', ...
        metrics.measurement.validNewFrameCount);

    fprintf(fileID,'Accepted new frame count: %d\n', ...
        metrics.measurement.acceptedNewFrameCount);

    fprintf(fileID,'New frame rate: %.6f Hz\n', ...
        metrics.measurement.newFrameRateHz);

    fprintf(fileID, ...
        'Measurement acceptance ratio among valid new frames: %.6f %%\n', ...
        metrics.measurement.acceptanceRatioPercent);
end

if isfield(metrics,'nis') && metrics.nis.available
    fprintf(fileID,'NIS sample count: %d\n', ...
        metrics.nis.sampleCount);

    fprintf(fileID,'NIS mean: %.12g\n',metrics.nis.mean);
    fprintf(fileID,'NIS median: %.12g\n',metrics.nis.median);
    fprintf(fileID,'NIS 95th percentile: %.12g\n', ...
        metrics.nis.percentile95);

    fprintf(fileID,'NIS 99th percentile: %.12g\n', ...
        metrics.nis.percentile99);

    fprintf(fileID,'NIS gate: %.12g\n',metrics.nis.gate);

    fprintf(fileID,'NIS gate exceedance: %.6f %%\n', ...
        metrics.nis.gateExceedancePercent);
end

if isfield(metrics,'imageError') && metrics.imageError.available
    writeVector(fileID,'Image error RMS normalized', ...
        metrics.imageError.rmsNormalized);

    writeVector(fileID,'Image error max abs normalized', ...
        metrics.imageError.maxAbsNormalized);

    writeVector(fileID,'Image error RMS pixels', ...
        metrics.imageError.rmsPixels);

    writeVector(fileID,'Image error max abs pixels', ...
        metrics.imageError.maxAbsPixels);
end

if isfield(metrics,'ekfVelocity') && metrics.ekfVelocity.available
    writeVector(fileID,'EKF velocity mean', ...
        metrics.ekfVelocity.mean);

    writeVector(fileID,'EKF velocity RMS', ...
        metrics.ekfVelocity.rms);

    writeVector(fileID,'EKF velocity 95th percentile abs', ...
        metrics.ekfVelocity.percentile95Abs);

    writeVector(fileID,'EKF velocity max abs', ...
        metrics.ekfVelocity.maxAbs);
end

if isfield(metrics,'staticMeasurement') && ...
        metrics.staticMeasurement.available

    fprintf(fileID,'\nStatic measurement noise estimate\n');
    fprintf(fileID,'---------------------------------\n');

    fprintf(fileID,'Static valid new samples: %d\n', ...
        metrics.staticMeasurement.sampleCount);

    writeVector(fileID,'Static measurement sigma (m)', ...
        metrics.staticMeasurement.sigmaMeters);

    writeVector(fileID,'Static measurement sigma (pixels)', ...
        metrics.staticMeasurement.sigmaPixels);

    fprintf(fileID,'Measured static covariance / suggested initial R:\n');
    writeMatrix(fileID,metrics.staticMeasurement.suggestedR);

    fprintf(fileID, ...
        ['Note: use this R estimate only when the robot and target were ' ...
         'both stationary and the data contains no obvious outliers.\n']);
end

if isfield(metrics,'cameraVelocity') && ...
        metrics.cameraVelocity.available

    writeVector(fileID,'Measured camera velocity RMS (m/s)', ...
        metrics.cameraVelocity.rms);

    writeVector(fileID,'Measured camera velocity max abs (m/s)', ...
        metrics.cameraVelocity.maxAbs);

    fprintf(fileID,'Camera velocity feedback valid ratio: %.6f %%\n', ...
        metrics.cameraVelocity.validRatioPercent);
end

if isfield(metrics,'covariance') && metrics.covariance.available
    writeVector(fileID,'Mean EKF state sigma', ...
        metrics.covariance.meanSigma);

    writeVector(fileID,'Final EKF state sigma', ...
        metrics.covariance.finalSigma);

    writeVector(fileID,'Maximum EKF state sigma', ...
        metrics.covariance.maxSigma);
end

if isfield(metrics,'controller') && metrics.controller.available
    fprintf(fileID,'Maximum issued XY speed: %.12g m/s\n', ...
        metrics.controller.maximumIssuedSpeed);

    fprintf(fileID,'RMS issued XY speed: %.12g m/s\n', ...
        metrics.controller.rmsIssuedSpeed);

    if isfield(metrics.controller,'velocitySaturationPercent')
        fprintf(fileID,'Velocity saturation ratio: %.6f %%\n', ...
            metrics.controller.velocitySaturationPercent);
    end

    if isfield(metrics.controller,'accelerationLimitPercent')
        fprintf(fileID,'Acceleration limit ratio: %.6f %%\n', ...
            metrics.controller.accelerationLimitPercent);
    end

    if isfield( ...
            metrics.controller, ...
            'feedforwardToProportionalRmsRatio')

        fprintf(fileID,'RMS(v_ff)/RMS(v_p): %.12g\n', ...
            metrics.controller. ...
                feedforwardToProportionalRmsRatio);
    end
end

if isfield(metrics,'velocityComparison') && ...
        metrics.velocityComparison.available

    fprintf(fileID,'\nOffline velocity comparison\n');
    fprintf(fileID,'---------------------------\n');

    writeVector(fileID,'Velocity comparison RMSE (m/s)', ...
        metrics.velocityComparison.rmse);

    writeVector(fileID,'Velocity comparison MAE (m/s)', ...
        metrics.velocityComparison.meanAbsoluteError);

    fprintf(fileID,'%s\n',metrics.velocityComparison.note);
end

fprintf(fileID,'\nHow to interpret this test\n');
fprintf(fileID,'--------------------------\n');

switch testType
    case 'static'
        fprintf(fileID, ...
            ['1. Use static z_meas standard deviation/covariance to check R.\n' ...
             '2. v_target_hat should remain close to zero.\n' ...
             '3. NIS should mostly stay below the gate.\n' ...
             '4. Large NIS only during complete stillness usually means ' ...
             'R is too small, visual outliers exist, or the model/units are wrong.\n']);

    case 'moving_target'
        fprintf(fileID, ...
            ['1. Compare EKF velocity with the offline rough reference.\n' ...
             '2. Smooth but delayed velocity and high NIS during turns suggest Q is too small.\n' ...
             '3. Fast but noisy velocity suggests Q is too large or R is too small.\n']);

    case 'moving_camera'
        fprintf(fileID, ...
            ['1. The target is fixed, so v_target_hat should stay near zero.\n' ...
             '2. NIS increasing only while the robot moves points to camera-velocity ' ...
             'sign, frame, timing, or Jacobian problems before Q/R tuning.\n']);

    case 'closed_loop'
        fprintf(fileID, ...
            ['1. Compare repeated tests with EKF feedforward off/on.\n' ...
             '2. Check image-error RMS, maximum error, limit ratios and RMS(v_ff)/RMS(v_p).\n' ...
             '3. EKF feedforward is useful only when error improves without causing ' ...
             'large velocity oscillation or frequent acceleration limiting.\n']);

    otherwise
        fprintf(fileID, ...
            ['Use the NIS, velocity, covariance, camera feedback and controller plots ' ...
             'to identify whether the problem comes from measurement noise, motion model, ' ...
             'camera compensation or controller gains.\n']);
end

fprintf(fileID,'\nCreated figures\n');
fprintf(fileID,'---------------\n');

for figureIndex = 1:numel(createdFigures)
    fprintf(fileID,'%s\n',createdFigures{figureIndex});
end

end


function writeVector(fileID,label,value)

fprintf(fileID,'%s:',label);

for index = 1:numel(value)
    fprintf(fileID,' %.12g',value(index));
end

fprintf(fileID,'\n');

end


function writeMatrix(fileID,matrixValue)

for rowIndex = 1:size(matrixValue,1)
    for columnIndex = 1:size(matrixValue,2)
        fprintf(fileID,' %.12g',matrixValue(rowIndex,columnIndex));
    end
    fprintf(fileID,'\n');
end

end