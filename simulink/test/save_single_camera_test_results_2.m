%% save_single_camera_test_results.m
% 用法：
% 1. 先运行初始化脚本并完成一次 Simulink 测试；
% 2. 停止仿真后，在 MATLAB 命令窗口运行：
%
%       testName = 'ekf_static_test';
%       save_single_camera_test_results
%
% 脚本会自动：
% - 查找 out / simOut 中的日志，或基础工作区中的 log_* 变量；
% - 创建带时间戳的测试文件夹；
% - 保存完整仿真输出、关键日志和控制参数；
% - 绘制并保存 PNG 和 FIG；
% - 生成 summary.txt，汇总本次测试的主要指标。
%
% 注意：
% - 本脚本不会重新运行 Simulink，只处理刚刚结束的那次测试。
% - 建议每次测试前先设置不同的 testName。

%% 1. 用户设置

% 如果运行脚本前没有手动设置 testName，则使用默认名称。
if ~exist('testName','var') || isempty(testName)
    testName = 'single_camera_test';
end

showFigures = true;
closeFiguresAfterSave = false;
saveCompleteSimulationOutput = true;

% 所有测试结果保存到当前目录下的 test_results 文件夹。
resultsRoot = fullfile(pwd,'test_results');

%% 2. 创建本次测试文件夹

safeTestName = regexprep(char(testName),'[^a-zA-Z0-9_\-]','_');
timeStamp = char(datetime('now','Format','MMdd_HHmmss'));

runFolderName = sprintf('%s_%s',timeStamp,safeTestName);
runFolder = fullfile(resultsRoot,runFolderName);
figureFolder = fullfile(runFolder,'figures');

if ~exist(resultsRoot,'dir')
    mkdir(resultsRoot);
end

mkdir(runFolder);
mkdir(figureFolder);

fprintf('\n============================================\n');
fprintf('开始保存本次测试：%s\n',safeTestName);
fprintf('结果目录：%s\n',runFolder);
fprintf('============================================\n');

%% 3. 查找 Simulink SimulationOutput

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
    fprintf('未找到 SimulationOutput 对象，将尝试读取基础工作区中的 log_* 变量。\n');
else
    fprintf('找到 SimulationOutput：%s\n',simulationOutputName);
end

%% 4. 读取关键日志

requestedLogs = { ...
    'log_error', ...
    'log_ekf_velocity', ...
    'log_v_p', ...
    'log_v_ff', ...
    'log_v_adapt', ...
    'log_v_norm_limited', ...
    'log_v_issued', ...
    'log_camera_velocity', ...
    'log_innovation_nis', ...
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

availableLogNames = fieldnames(logs);

if isempty(availableLogNames)
    error([ ...
        '没有找到任何所需日志。请确认仿真已经结束，并检查 To Workspace 块、' ...
        'Signal Logging 或 SimulationOutput 中的变量名称。']);
end

fprintf('找到 %d 个关键日志。\n',numel(availableLogNames));

if ~isempty(missingLogs)
    fprintf('以下日志未找到，将跳过相关图形：\n');
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

%% 6. 保存 MAT 数据

save( ...
    fullfile(runFolder,'selected_logs_and_parameters.mat'), ...
    'logs', ...
    'parameters', ...
    'missingLogs', ...
    'safeTestName', ...
    'timeStamp', ...
    '-v7.3');

if saveCompleteSimulationOutput && ~isempty(simulationOutput)
    save( ...
        fullfile(runFolder,'complete_simulation_output.mat'), ...
        'simulationOutput', ...
        '-v7.3');
end

%% 7. 保存初始化文件和 YAML 配置副本

copyConfigurationFiles(runFolder);

%% 8. 绘制图像

visibilityValue = 'off';
if showFigures
    visibilityValue = 'on';
end

createdFigures = {};

% ---------------------------------------------------------
% 图1：归一化图像误差
% ---------------------------------------------------------
if isfield(logs,'log_error')
    [time,errorData] = signalToTimeData(logs.log_error);

    fig = figure('Visible',visibilityValue);
    plot(time,errorData,'LineWidth',1.2);
    grid on;
    xlabel('Time (s)');
    ylabel('Normalized error');
    legend(makeChannelLabels('e',size(errorData,2)),'Location','best');
    title('Normalized Image Error');

    saveTestFigure(fig,figureFolder,'01_normalized_image_error');
    createdFigures{end+1} = '01_normalized_image_error'; %#ok<SAGROW>

    if closeFiguresAfterSave
        close(fig);
    end
end

% ---------------------------------------------------------
% 图2：EKF目标速度
% ---------------------------------------------------------
if isfield(logs,'log_ekf_velocity')
    [time,velocityData] = signalToTimeData(logs.log_ekf_velocity);

    fig = figure('Visible',visibilityValue);
    plot(time,velocityData,'LineWidth',1.2);
    grid on;
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    legend(makeChannelLabels('V_{target}',size(velocityData,2)),'Location','best');
    title('EKF Target Velocity');

    saveTestFigure(fig,figureFolder,'02_ekf_target_velocity');
    createdFigures{end+1} = '02_ekf_target_velocity'; %#ok<SAGROW>

    if closeFiguresAfterSave
        close(fig);
    end
end

% ---------------------------------------------------------
% 图3：X方向控制器速度分量
% ---------------------------------------------------------
requiredXSignals = { ...
    'log_v_p', ...
    'log_v_ff', ...
    'log_v_adapt', ...
    'log_v_issued'};

if all(isfield(logs,requiredXSignals))
    [tp,vp] = signalToTimeData(logs.log_v_p);
    [tff,vff] = signalToTimeData(logs.log_v_ff);
    [ta,va] = signalToTimeData(logs.log_v_adapt);
    [ti,vi] = signalToTimeData(logs.log_v_issued);

    fig = figure('Visible',visibilityValue);
    hold on;
    plot(tp,vp(:,1),'LineWidth',1.1);
    plot(tff,vff(:,1),'LineWidth',1.1);
    plot(ta,va(:,1),'LineWidth',1.1);
    plot(ti,vi(:,1),'LineWidth',1.3);
    grid on;
    xlabel('Time (s)');
    ylabel('X velocity (m/s)');
    legend( ...
        'v_{p,x}', ...
        'v_{ff,x}', ...
        'v_{adapt,x}', ...
        'v_{issued,x}', ...
        'Location','best');
    title('X-axis Controller Components');

    saveTestFigure(fig,figureFolder,'03_controller_velocity_x');
    createdFigures{end+1} = '03_controller_velocity_x'; %#ok<SAGROW>

    if closeFiguresAfterSave
        close(fig);
    end
end

% ---------------------------------------------------------
% 图4：Y方向控制器速度分量
% ---------------------------------------------------------
if all(isfield(logs,requiredXSignals))
    [tp,vp] = signalToTimeData(logs.log_v_p);
    [tff,vff] = signalToTimeData(logs.log_v_ff);
    [ta,va] = signalToTimeData(logs.log_v_adapt);
    [ti,vi] = signalToTimeData(logs.log_v_issued);

    if size(vp,2) >= 2 && size(vff,2) >= 2 && ...
            size(va,2) >= 2 && size(vi,2) >= 2

        fig = figure('Visible',visibilityValue);
        hold on;
        plot(tp,vp(:,2),'LineWidth',1.1);
        plot(tff,vff(:,2),'LineWidth',1.1);
        plot(ta,va(:,2),'LineWidth',1.1);
        plot(ti,vi(:,2),'LineWidth',1.3);
        grid on;
        xlabel('Time (s)');
        ylabel('Y velocity (m/s)');
        legend( ...
            'v_{p,y}', ...
            'v_{ff,y}', ...
            'v_{adapt,y}', ...
            'v_{issued,y}', ...
            'Location','best');
        title('Y-axis Controller Components');

        saveTestFigure(fig,figureFolder,'04_controller_velocity_y');
        createdFigures{end+1} = '04_controller_velocity_y'; %#ok<SAGROW>

        if closeFiguresAfterSave
            close(fig);
        end
    end
end

% ---------------------------------------------------------
% 图5：NIS和测量接受状态
% ---------------------------------------------------------
if isfield(logs,'log_innovation_nis') && ...
        isfield(logs,'log_measurement_accepted')

    [tnis,nisData] = signalToTimeData(logs.log_innovation_nis);
    [taccept,acceptedData] = signalToTimeData(logs.log_measurement_accepted);

    fig = figure('Visible',visibilityValue);

    yyaxis left;
    plot(tnis,nisData(:,1),'LineWidth',1.1);
    ylabel('Innovation NIS');

    if isfield(parameters,'ekf_gate_threshold')
        yline(parameters.ekf_gate_threshold,'--','Gate');
    end

    yyaxis right;
    stairs(taccept,acceptedData(:,1),'LineWidth',1.0);
    ylabel('Measurement accepted');

    grid on;
    xlabel('Time (s)');
    title('EKF Innovation and Measurement Acceptance');

    saveTestFigure(fig,figureFolder,'05_ekf_nis_and_acceptance');
    createdFigures{end+1} = '05_ekf_nis_and_acceptance'; %#ok<SAGROW>

    if closeFiguresAfterSave
        close(fig);
    end
end

% ---------------------------------------------------------
% 图6：速度限幅和加速度限幅状态
% ---------------------------------------------------------
if isfield(logs,'log_saturation') && ...
        isfield(logs,'log_acceleration_limit')

    [tsat,saturationData] = signalToTimeData(logs.log_saturation);
    [tacc,accelerationData] = signalToTimeData(logs.log_acceleration_limit);

    fig = figure('Visible',visibilityValue);
    stairs(tsat,saturationData(:,1),'LineWidth',1.1);
    hold on;
    stairs(tacc,accelerationData(:,1),'LineWidth',1.1);
    grid on;
    xlabel('Time (s)');
    ylabel('Active');
    legend( ...
        'Velocity saturation', ...
        'Acceleration limit', ...
        'Location','best');
    title('Controller Limits');

    saveTestFigure(fig,figureFolder,'06_controller_limits');
    createdFigures{end+1} = '06_controller_limits'; %#ok<SAGROW>

    if closeFiguresAfterSave
        close(fig);
    end
end

% ---------------------------------------------------------
% 图7：有效性和控制器状态
% ---------------------------------------------------------
statusSignals = { ...
    'log_safe_valid', ...
    'log_controller_ok', ...
    'log_ekf_initialized'};

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
            statusData(:,1), ...
            'LineWidth',1.0);

        statusLabels{statusIndex} = strrep(signalName,'log_','');
    end

    grid on;
    xlabel('Time (s)');
    ylabel('State');
    legend(statusLabels,'Interpreter','none','Location','best');
    title('Validity and Controller Status');

    saveTestFigure(fig,figureFolder,'07_validity_and_controller_status');
    createdFigures{end+1} = '07_validity_and_controller_status'; %#ok<SAGROW>

    if closeFiguresAfterSave
        close(fig);
    end
end

%% 9. 生成测试摘要

summaryFile = fullfile(runFolder,'summary.txt');
writeSummary( ...
    summaryFile, ...
    safeTestName, ...
    timeStamp, ...
    logs, ...
    parameters, ...
    missingLogs, ...
    createdFigures);

%% 10. 完成提示

fprintf('\n保存完成。\n');
fprintf('原始日志：%s\n', ...
    fullfile(runFolder,'selected_logs_and_parameters.mat'));

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
% 优先从 SimulationOutput 中读取；找不到时读取基础工作区。

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
    time = seconds(signal.Properties.RowTimes - signal.Properties.RowTimes(1));
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


function labels = makeChannelLabels(prefix,channelCount)
% 生成通道图例。

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
% 同时保存PNG和MATLAB FIG。

exportgraphics( ...
    fig, ...
    fullfile(figureFolder,[fileBaseName,'.png']), ...
    'Resolution',200);

savefig( ...
    fig, ...
    fullfile(figureFolder,[fileBaseName,'.fig']));

end


function copyConfigurationFiles(runFolder)
% 尝试保存初始化脚本和ROS YAML配置副本。

% 1. 当前目录或子目录中的初始化脚本
initCandidates = dir( ...
    fullfile(pwd,'**','init_single_camera_xy_tracking.m'));

if ~isempty(initCandidates)
    initSource = fullfile( ...
        initCandidates(1).folder, ...
        initCandidates(1).name);

    copyfile( ...
        initSource, ...
        fullfile(runFolder,'init_single_camera_xy_tracking_snapshot.m'));
end

% 2. ROS工作空间中的统一YAML配置
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
            fullfile(runFolder,'velocity_servo_tag_snapshot.yaml'));
    end
end

end


function writeSummary( ...
    summaryFile, ...
    testName, ...
    timeStamp, ...
    logs, ...
    parameters, ...
    missingLogs, ...
    createdFigures)
% 生成便于比较不同测试的文本摘要。

fileID = fopen(summaryFile,'w');

if fileID < 0
    warning('无法创建测试摘要：%s',summaryFile);
    return;
end

cleanupObject = onCleanup(@() fclose(fileID)); %#ok<NASGU>

fprintf(fileID,'Single-camera XY tracking test summary\n');
fprintf(fileID,'======================================\n');
fprintf(fileID,'Test name: %s\n',testName);
fprintf(fileID,'Timestamp: %s\n\n',timeStamp);

fprintf(fileID,'Available logs: %s\n', ...
    strjoin(fieldnames(logs),', '));

if isempty(missingLogs)
    fprintf(fileID,'Missing logs: none\n\n');
else
    fprintf(fileID,'Missing logs: %s\n\n', ...
        strjoin(missingLogs,', '));
end

fprintf(fileID,'Controller parameters\n');
fprintf(fileID,'---------------------\n');

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
    'target_timeout_sec', ...
    'joint_state_timeout_sec'};

for parameterIndex = 1:numel(scalarParameterNames)
    parameterName = scalarParameterNames{parameterIndex};

    if isfield(parameters,parameterName)
        fprintf( ...
            fileID, ...
            '%s: %.9g\n', ...
            parameterName, ...
            double(parameters.(parameterName)));
    end
end

fprintf(fileID,'\nMeasured indicators\n');
fprintf(fileID,'-------------------\n');

if isfield(logs,'log_error')
    [time,errorData] = signalToTimeData(logs.log_error);

    fprintf(fileID,'Test duration: %.3f s\n',time(end)-time(1));

    for channelIndex = 1:size(errorData,2)
        fprintf( ...
            fileID, ...
            'Error channel %d RMS: %.9g\n', ...
            channelIndex, ...
            sqrt(mean(errorData(:,channelIndex).^2,'omitnan')));

        fprintf( ...
            fileID, ...
            'Error channel %d max abs: %.9g\n', ...
            channelIndex, ...
            max(abs(errorData(:,channelIndex)),[],'omitnan'));
    end
end

if isfield(logs,'log_ekf_velocity')
    [~,velocityData] = signalToTimeData(logs.log_ekf_velocity);

    for channelIndex = 1:size(velocityData,2)
        fprintf( ...
            fileID, ...
            'EKF velocity channel %d mean abs: %.9g m/s\n', ...
            channelIndex, ...
            mean(abs(velocityData(:,channelIndex)),'omitnan'));

        fprintf( ...
            fileID, ...
            'EKF velocity channel %d max abs: %.9g m/s\n', ...
            channelIndex, ...
            max(abs(velocityData(:,channelIndex)),[],'omitnan'));
    end
end

if isfield(logs,'log_v_issued')
    [~,issuedVelocity] = signalToTimeData(logs.log_v_issued);

    issuedSpeed = sqrt(sum(issuedVelocity.^2,2));

    fprintf( ...
        fileID, ...
        'Maximum issued XY speed: %.9g m/s\n', ...
        max(issuedSpeed,[],'omitnan'));
end

if isfield(logs,'log_measurement_accepted')
    [~,acceptedData] = signalToTimeData( ...
        logs.log_measurement_accepted);

    fprintf( ...
        fileID, ...
        'Measurement acceptance ratio: %.3f %%\n', ...
        100*mean(acceptedData(:,1) > 0.5,'omitnan'));
end

if isfield(logs,'log_saturation')
    [~,saturationData] = signalToTimeData(logs.log_saturation);

    fprintf( ...
        fileID, ...
        'Velocity saturation ratio: %.3f %%\n', ...
        100*mean(saturationData(:,1) > 0.5,'omitnan'));
end

if isfield(logs,'log_acceleration_limit')
    [~,accelerationData] = signalToTimeData( ...
        logs.log_acceleration_limit);

    fprintf( ...
        fileID, ...
        'Acceleration limit ratio: %.3f %%\n', ...
        100*mean(accelerationData(:,1) > 0.5,'omitnan'));
end

if isfield(logs,'log_innovation_nis')
    [~,nisData] = signalToTimeData(logs.log_innovation_nis);

    finiteNis = nisData(isfinite(nisData));

    if ~isempty(finiteNis)
        fprintf( ...
            fileID, ...
            'NIS median: %.9g\n', ...
            median(finiteNis,'omitnan'));

        fprintf( ...
            fileID, ...
            'NIS 95th percentile: %.9g\n', ...
            prctile(finiteNis,95));
    end
end

fprintf(fileID,'\nCreated figures\n');
fprintf(fileID,'---------------\n');

for figureIndex = 1:numel(createdFigures)
    fprintf(fileID,'%s\n',createdFigures{figureIndex});
end

end
