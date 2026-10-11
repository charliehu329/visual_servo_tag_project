%% save_single_camera_test_results.m
% 单目XY视觉伺服：Base坐标系KF调参数据保存与可视化脚本
%
% 当前模型约定：
% - z_meas = [X_C;Y_C]，属于相机坐标系；
% - ekf_state = [X_B;Y_B;Vx_B;Vy_B]，属于机器人Base坐标系；
% - v_p、v_ff、v_issued属于相机坐标系。
%
% 使用方法：
% 1. 运行初始化脚本并完成一次Simulink测试；
% 2. 停止仿真后，在MATLAB命令窗口设置：
%
%       testName = 'base_kf_static_01';
%       testType = 'static';
%       save_single_camera_base_kf_tuning_results
%
% testType可选：
%   'static'        机器人不动、目标不动；
%   'moving_target' 机器人不动、目标移动；
%   'moving_camera' 目标不动、机器人移动；
%   'closed_loop'   正常闭环跟踪；
%   'generic'       通用分析。
%
% 本脚本会：
% - 保存完整日志、参数和指标；
% - 单独导出ekf_state_timeseries.csv；
% - 绘制完整四状态[X_B,Y_B,Vx_B,Vy_B]；
% - 绘制状态不确定度、NIS、控制器分量和状态信号；
% - 生成summary.txt。

%% 1. 用户设置

if ~exist('testName','var') || isempty(testName)
    testName = 'single_camera_base_kf_test';
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
    error('testType必须是：%s',strjoin(validTestTypes,', '));
end

analysisStartSec = 1.0;
if ~exist('showFigures','var') || isempty(showFigures)
    showFigures = true;
end
if ~exist('closeFiguresAfterSave','var') || isempty(closeFiguresAfterSave)
    closeFiguresAfterSave = false;
end
if ~exist('saveCompleteSimulationOutput','var') || isempty(saveCompleteSimulationOutput)
    saveCompleteSimulationOutput = true;
end
if ~exist('resultsRoot','var') || isempty(resultsRoot)
    resultsRoot = fileparts(mfilename('fullpath'));

end

%% 2. 创建本次测试文件夹

safeTestName = regexprep(char(testName),'[^a-zA-Z0-9_\-]','_');
safeTestType = regexprep(char(testType),'[^a-zA-Z0-9_\-]','_');
timeStamp = char(datetime('now','Format','MMdd_HHmmss'));

runFolderName = sprintf('%s_%s_%s',timeStamp,safeTestType,safeTestName);
runFolder = fullfile(resultsRoot,runFolderName);
figureFolder = fullfile(runFolder,'figures');

if ~exist(resultsRoot,'dir')
    mkdir(resultsRoot);
end
mkdir(runFolder);
mkdir(figureFolder);

fprintf('\n============================================\n');
fprintf('开始保存Base坐标系KF测试：%s\n',safeTestName);
fprintf('测试类型：%s\n',safeTestType);
fprintf('结果目录：%s\n',runFolder);
fprintf('============================================\n');

%% 3. 查找Simulink SimulationOutput

simulationOutput = [];
simulationOutputName = '';
candidateOutputNames = {'out','simOut','ans'};

for candidateIndex = 1:numel(candidateOutputNames)
    candidateName = candidateOutputNames{candidateIndex};
    variableExists = evalin('base',sprintf('exist(''%s'',''var'')',candidateName));

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

%% 4. 读取日志
%
% 当前模型核心日志：
% log_z_meas：相机坐标系XY测量；
% log_ekf_state：Base坐标系四状态；
% log_ekf_velocity：Base坐标系目标XY速度；
% log_v_p、log_v_ff、log_v_issued：相机坐标系控制速度。

requestedLogs = { ...
    'log_z_meas', ...
    'log_error', ...
    'log_e', ...
    'log_ekf_state', ...
    'log_ekf_covariance', ...
    'log_ekf_velocity', ...
    'log_v_p', ...
    'log_v_ff', ...
    'log_v_issued', ...
    'log_v_task_raw', ...
    'log_v_norm_limited', ...
    'log_saturation_active', ...
    'log_acceleration_active', ...
    'log_camera_velocity', ...
    'log_innovation_nis', ...
    'log_measurement_is_new', ...
    'log_target_is_new', ...
    'log_measurement_accepted', ...
    'log_safe_valid', ...
    'log_controller_ok', ...
    'log_ekf_initialized', ...
    'log_v_camera_measured', ...
    'log_v_camera_measured_valid', ...
    'log_u', ...
    'log_v', ...
    'log_joint_position', ...
    'log_joint_velocity', ...
    'log_qdot_raw', ...
    'log_qdot_command', ...
    'log_velocity_scale', ...
    'log_acceleration_scale', ...
    'log_position_scale', ...
    'log_guard_valid', ...
    'log_target_age', ...
    'log_joint_age', ...
    'log_target_data', ...
    'log_sigma_min', ...
    'log_singularity_scale', ...
    'log_damping_used', ...
    'log_solver_valid', ...
    'log_control_active', ...
    'log_kinematics_valid', ...
    'log_target_visible'};

logs = struct();
missingLogs = {};

for logIndex = 1:numel(requestedLogs)
    logName = requestedLogs{logIndex};
    signal = getLoggedSignal(logName,simulationOutput);

    if isempty(signal)
        missingLogs{end+1} = logName; %#ok<SAGROW>
    else
        logs.(logName) = signal;
    end
end

[logs,missingLogs] = deriveCurrentModelLogs(logs,missingLogs);

% 兼容log_error和log_e。
if ~isfield(logs,'log_error') && isfield(logs,'log_e')
    logs.log_error = logs.log_e;
    missingLogs(strcmp(missingLogs,'log_error')) = [];
end

% 兼容新旧的新帧日志名称。
if ~isfield(logs,'log_measurement_is_new') && isfield(logs,'log_target_is_new')
    logs.log_measurement_is_new = logs.log_target_is_new;
    missingLogs(strcmp(missingLogs,'log_measurement_is_new')) = [];
end

missingLogs(strcmp(missingLogs,'log_target_is_new')) = [];

availableLogNames = fieldnames(logs);

if isempty(availableLogNames)
    error(['没有找到任何所需日志。请确认仿真已经结束，并检查' ...
        'To Workspace块、Signal Logging或SimulationOutput变量名。']);
end

fprintf('找到%d个日志。\n',numel(availableLogNames));

if ~isempty(missingLogs)
    fprintf('以下日志未找到，将跳过对应分析：\n');
    fprintf('  %s\n',strjoin(missingLogs,', '));
end

%% 5. 保存参数快照

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
    'task_velocity_filter_tau', ...
    'enable_proportional', ...
    'enable_ekf_feedforward', ...
    'controller_enable', ...
    'USE_ROS', ...
    'P0', ...
    'Q_ekf', ...
    'R_ekf', ...
    'ekf_gate_threshold', ...
    'required_valid_frames', ...
    'measurement_reset_timeout', ...
    'target_timeout_sec', ...
    'joint_state_timeout_sec', ...
    'camera_velocity_feedback_timeout_sec', ...
    'visual_damping_min', ...
    'visual_damping_max', ...
    'singularity_sigma_stop', ...
    'singularity_sigma_warn', ...
    'nullspace_posture_gain', ...
    'depth_safe_min', ...
    'depth_safe_max', ...
    'depth_safety_gain', ...
    'visual_min_depth', ...
    'joint_soft_margin', ...
    'joint_nominal_configuration', ...
    'joint_position_min', ...
    'joint_position_max', ...
    'joint_velocity_limit', ...
    'joint_acceleration_limit', ...
    'T_link8_camera'};

parameters = struct();

for parameterIndex = 1:numel(parameterNames)
    parameterName = parameterNames{parameterIndex};
    variableExists = evalin('base',sprintf('exist(''%s'',''var'')',parameterName));

    if variableExists
        parameters.(parameterName) = evalin('base',parameterName);
    end
end

%% 6. 计算指标

analysisOptions = struct();
analysisOptions.testType = testType;
analysisOptions.analysisStartSec = analysisStartSec;

metrics = calculateMetrics(logs,parameters,analysisOptions);

%% 7. 保存MAT和EKF状态CSV

save(fullfile(runFolder,'selected_logs_parameters_metrics.mat'), ...
    'logs','parameters','metrics','missingLogs','safeTestName', ...
    'safeTestType','timeStamp','analysisOptions','-v7.3');

if saveCompleteSimulationOutput && ~isempty(simulationOutput)
    save(fullfile(runFolder,'complete_simulation_output.mat'), ...
        'simulationOutput','-v7.3');
end

% 单独导出完整四状态，便于Origin、Excel或后续MATLAB分析。
if isfield(logs,'log_ekf_state')
    [stateTime,stateData] = signalToTimeData(logs.log_ekf_state);

    if size(stateData,2) >= 4
        stateTable = table( ...
            stateTime, ...
            stateData(:,1),stateData(:,2), ...
            stateData(:,3),stateData(:,4), ...
            'VariableNames',{ ...
                'time_s','X_B_m','Y_B_m', ...
                'Vx_B_mps','Vy_B_mps'});

        writetable(stateTable,fullfile(runFolder,'ekf_state_timeseries.csv'));
    end
end

% 单独导出奇异度时序，便于Origin、Excel或后续MATLAB分析。
if isfield(logs,'log_sigma_min')
    [singularityTime,sigmaData] = ...
        signalToTimeData(logs.log_sigma_min);

    singularityTable = table( ...
        singularityTime, ...
        sigmaData(:,1), ...
        'VariableNames',{'time_s','sigma_min'});

    if isfield(logs,'log_singularity_scale')
        [scaleTime,scaleData] = ...
            signalToTimeData(logs.log_singularity_scale);
        singularityTable.singularity_scale = ...
            interpolateData(scaleTime,scaleData(:,1), ...
            singularityTime,'linear');
    end
    if isfield(logs,'log_damping_used')
        [dampingTime,dampingData] = ...
            signalToTimeData(logs.log_damping_used);
        singularityTable.damping_used = ...
            interpolateData(dampingTime,dampingData(:,1), ...
            singularityTime,'linear');
    end

    writetable(singularityTable, ...
        fullfile(runFolder,'singularity_timeseries.csv'));
end

%% 8. 保存初始化文件和YAML配置副本

copyConfigurationFiles(runFolder);

%% 9. 绘图

visibilityValue = 'off';
if showFigures
    visibilityValue = 'on';
end
createdFigures = {};

% ---------------------------------------------------------
% 图1：归一化误差和像素误差
% ---------------------------------------------------------
if isfield(logs,'log_error')
    [time,errorData] = signalToTimeData(logs.log_error);

    fig = figure('Visible',visibilityValue,'Name','Image Error');
    tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

    nexttile;
    plot(time,errorData(:,1:min(2,size(errorData,2))),'LineWidth',1.1);
    yline(0,':');
    grid on;
    xlabel('Time (s)');
    ylabel('Normalized error');
    legend({'e_x','e_y'},'Location','best');
    title('Normalized Image Error');

    pixelError = errorData;
    if isfield(parameters,'fx') && size(pixelError,2) >= 1
        pixelError(:,1) = pixelError(:,1)*parameters.fx;
    end
    if isfield(parameters,'fy') && size(pixelError,2) >= 2
        pixelError(:,2) = pixelError(:,2)*parameters.fy;
    end

    nexttile;
    plot(time,pixelError(:,1:min(2,size(pixelError,2))),'LineWidth',1.1);
    yline(0,':');
    grid on;
    xlabel('Time (s)');
    ylabel('Pixel error (px)');
    legend({'e_{u}','e_{v}'},'Location','best');
    title('Image Error in Pixels');

    saveTestFigure(fig,figureFolder,'01_image_error');
    createdFigures{end+1} = '01_image_error'; %#ok<SAGROW>
    if closeFiguresAfterSave, close(fig); end
end

% ---------------------------------------------------------
% 图2：相机坐标系视觉XY测量
% 注意：不能与Base坐标系EKF位置直接叠加比较。
% ---------------------------------------------------------
if isfield(logs,'log_z_meas')
    [time,zData] = signalToTimeData(logs.log_z_meas);

    fig = figure('Visible',visibilityValue,'Name','Camera-frame Measurement');
    tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

    nexttile;
    plot(time,zData(:,1),'LineWidth',1.1);
    yline(0,':');
    grid on;
    xlabel('Time (s)');
    ylabel('X_C (m)');
    title('Visual Measurement in Camera Frame - X');

    nexttile;
    if size(zData,2) >= 2
        plot(time,zData(:,2),'LineWidth',1.1);
    end
    yline(0,':');
    grid on;
    xlabel('Time (s)');
    ylabel('Y_C (m)');
    title('Visual Measurement in Camera Frame - Y');

    saveTestFigure(fig,figureFolder,'02_camera_frame_measurement');
    createdFigures{end+1} = '02_camera_frame_measurement'; %#ok<SAGROW>
    if closeFiguresAfterSave, close(fig); end
end

% ---------------------------------------------------------
% 图3：完整KF四状态输出（Base坐标系）
% ---------------------------------------------------------
if isfield(logs,'log_ekf_state')
    [time,stateData] = signalToTimeData(logs.log_ekf_state);

    if size(stateData,2) >= 4
        fig = figure('Visible',visibilityValue,'Name','Base-frame KF State');
        tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

        nexttile;
        plot(time,stateData(:,1:2),'LineWidth',1.15);
        grid on;
        xlabel('Time (s)');
        ylabel('Position (m)');
        legend({'\hat X_B','\hat Y_B'},'Location','best');
        title('KF Position State in Base Frame');

        nexttile;
        plot(time,stateData(:,3:4),'LineWidth',1.15);
        yline(0,':');
        grid on;
        xlabel('Time (s)');
        ylabel('Velocity (m/s)');
        legend({'\hat V_{x,B}','\hat V_{y,B}'},'Location','best');
        title('KF Velocity State in Base Frame');

        saveTestFigure(fig,figureFolder,'03_ekf_state_base_frame');
        createdFigures{end+1} = '03_ekf_state_base_frame'; %#ok<SAGROW>
        if closeFiguresAfterSave, close(fig); end
    end
end

% ---------------------------------------------------------
% 图4：KF状态标准差
% ---------------------------------------------------------
if isfield(logs,'log_ekf_covariance')
    [time,pDiag] = covarianceToDiagonal(logs.log_ekf_covariance,4);
    stateSigma = sqrt(max(pDiag,0));

    fig = figure('Visible',visibilityValue,'Name','KF State Uncertainty');
    tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

    nexttile;
    plot(time,stateSigma(:,1:2),'LineWidth',1.1);
    grid on;
    xlabel('Time (s)');
    ylabel('\sigma_p (m)');
    legend({'\sigma_{X_B}','\sigma_{Y_B}'},'Location','best');
    title('Position Standard Deviation');

    nexttile;
    plot(time,stateSigma(:,3:4),'LineWidth',1.1);
    grid on;
    xlabel('Time (s)');
    ylabel('\sigma_v (m/s)');
    legend({'\sigma_{Vx_B}','\sigma_{Vy_B}'},'Location','best');
    title('Velocity Standard Deviation');

    saveTestFigure(fig,figureFolder,'04_ekf_state_uncertainty');
    createdFigures{end+1} = '04_ekf_state_uncertainty'; %#ok<SAGROW>
    if closeFiguresAfterSave, close(fig); end
end

% ---------------------------------------------------------
% 图5：控制器P、FF和最终命令（相机坐标系）
% ---------------------------------------------------------
controllerSignals = {'log_v_p','log_v_ff','log_v_issued'};
if all(isfield(logs,controllerSignals))
    [tp,vp] = signalToTimeData(logs.log_v_p);
    [tff,vff] = signalToTimeData(logs.log_v_ff);
    [ti,vi] = signalToTimeData(logs.log_v_issued);

    vff = interpolateData(tff,vff,tp,'linear');
    vi = interpolateData(ti,vi,tp,'linear');

    fig = figure('Visible',visibilityValue,'Name','Controller Components');
    tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

    for axisIndex = 1:2
        nexttile;
        plot(tp,vp(:,axisIndex),'LineWidth',1.0);
        hold on;
        plot(tp,vff(:,axisIndex),'LineWidth',1.0);
        plot(tp,vi(:,axisIndex),'LineWidth',1.25);
        yline(0,':');
        grid on;
        xlabel('Time (s)');
        ylabel('Velocity (m/s)');
        legend({'v_p','v_{ff}','v_{issued}'},'Location','best');

        if axisIndex == 1
            title('Camera-frame X Velocity Command');
        else
            title('Camera-frame Y Velocity Command');
        end
    end

    saveTestFigure(fig,figureFolder,'05_controller_components_xy');
    createdFigures{end+1} = '05_controller_components_xy'; %#ok<SAGROW>
    if closeFiguresAfterSave, close(fig); end
end

% ---------------------------------------------------------
% 图5B：限幅前后速度与限幅状态
%
% 上两幅图：
%   v_norm_limited与v_issued在相机坐标系X/Y方向上的对比。
%
% 下幅图：
%   saturation_active：速度模长限幅是否激活；
%   acceleration_active：加速度限制是否激活。
% ---------------------------------------------------------
limitDiagnosticSignals = { ...
    'log_v_norm_limited', ...
    'log_v_issued', ...
    'log_saturation_active', ...
    'log_acceleration_active'};

if all(isfield(logs,limitDiagnosticSignals))
    [tn,vNormLimited] = ...
        signalToTimeData(logs.log_v_norm_limited);

    [ti,vIssued] = ...
        signalToTimeData(logs.log_v_issued);

    [ts,saturationActive] = ...
        signalToTimeData(logs.log_saturation_active);

    [ta,accelerationActive] = ...
        signalToTimeData(logs.log_acceleration_active);

    if size(vNormLimited,2) < 2
        error('log_v_norm_limited必须至少包含X、Y两列。');
    end

    if size(vIssued,2) < 2
        error('log_v_issued必须至少包含X、Y两列。');
    end

    % 统一到v_norm_limited的时间轴，便于逐点比较。
    vIssuedOnLimitedTime = interpolateData( ...
        ti, ...
        vIssued(:,1:2), ...
        tn, ...
        'linear');

    saturationOnLimitedTime = interpolateData( ...
        ts, ...
        double(saturationActive(:,1) > 0.5), ...
        tn, ...
        'previous');

    accelerationOnLimitedTime = interpolateData( ...
        ta, ...
        double(accelerationActive(:,1) > 0.5), ...
        tn, ...
        'previous');

    saturationOnLimitedTime = ...
        saturationOnLimitedTime > 0.5;

    accelerationOnLimitedTime = ...
        accelerationOnLimitedTime > 0.5;

    saturationRatioPercent = ...
        100*mean(saturationOnLimitedTime);

    accelerationRatioPercent = ...
        100*mean(accelerationOnLimitedTime);

    velocityDifference = ...
        vIssuedOnLimitedTime - vNormLimited(:,1:2);

    maximumDifference = max( ...
        abs(velocityDifference), ...
        [], ...
        1, ...
        'omitnan');

    fig = figure( ...
        'Visible',visibilityValue, ...
        'Name','Limited and Issued Velocity');

    tiledlayout( ...
        3, ...
        1, ...
        'TileSpacing','compact', ...
        'Padding','compact');

    nexttile;
    plot( ...
        tn, ...
        vNormLimited(:,1), ...
        'LineWidth',1.1);
    hold on;
    plot( ...
        tn, ...
        vIssuedOnLimitedTime(:,1), ...
        'LineWidth',1.25);
    yline(0,':');
    grid on;
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    legend( ...
        {'v_{norm-limited,x}','v_{issued,x}'}, ...
        'Location','best');
    title(sprintf( ...
        'Camera-frame X | max difference = %.6g m/s', ...
        maximumDifference(1)));

    nexttile;
    plot( ...
        tn, ...
        vNormLimited(:,2), ...
        'LineWidth',1.1);
    hold on;
    plot( ...
        tn, ...
        vIssuedOnLimitedTime(:,2), ...
        'LineWidth',1.25);
    yline(0,':');
    grid on;
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    legend( ...
        {'v_{norm-limited,y}','v_{issued,y}'}, ...
        'Location','best');
    title(sprintf( ...
        'Camera-frame Y | max difference = %.6g m/s', ...
        maximumDifference(2)));

    nexttile;
    stairs( ...
        tn, ...
        double(saturationOnLimitedTime), ...
        'LineWidth',1.1);
    hold on;
    stairs( ...
        tn, ...
        double(accelerationOnLimitedTime) + 1.25, ...
        'LineWidth',1.1);
    ylim([-0.15 2.40]);
    yticks([0 1 1.25 2.25]);
    yticklabels({ ...
        'Sat off', ...
        'Sat on', ...
        'Accel off', ...
        'Accel on'});
    grid on;
    xlabel('Time (s)');
    ylabel('Limit state');
    legend( ...
        { ...
            sprintf( ...
                'saturation active (%.2f%%)', ...
                saturationRatioPercent), ...
            sprintf( ...
                'acceleration active + 1.25 (%.2f%%)', ...
                accelerationRatioPercent) ...
        }, ...
        'Location','best');
    title('Controller Velocity and Acceleration Limit Activity');

    saveTestFigure( ...
        fig, ...
        figureFolder, ...
        '05b_limited_vs_issued_and_limit_states');

    createdFigures{end+1} = ...
        '05b_limited_vs_issued_and_limit_states'; %#ok<SAGROW>

    if closeFiguresAfterSave
        close(fig);
    end
end

% ---------------------------------------------------------
% 图6：NIS时序和分布
% ---------------------------------------------------------
if isfield(logs,'log_innovation_nis')
    [time,nisData] = signalToTimeData(logs.log_innovation_nis);
    nisVector = nisData(:,1);

    validNisMask = isfinite(nisVector);
    if isfield(logs,'log_measurement_is_new')
        validNisMask = validNisMask & ...
            interpolateBooleanSignal(logs.log_measurement_is_new,time);
    end
    if isfield(logs,'log_safe_valid')
        validNisMask = validNisMask & ...
            interpolateBooleanSignal(logs.log_safe_valid,time);
    end
    finiteNis = nisVector(validNisMask);

    fig = figure('Visible',visibilityValue,'Name','NIS');
    tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

    nexttile;
    plot(time,nisVector,'LineWidth',1.0);
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
        histogram(finiteNis,'Normalization','pdf', ...
            'NumBins',max(10,min(50,round(sqrt(numel(finiteNis))))));
        hold on;
        xMax = max(finiteNis);
        if isfield(parameters,'ekf_gate_threshold')
            xMax = max(xMax,1.5*parameters.ekf_gate_threshold);
            xline(parameters.ekf_gate_threshold,'--','NIS gate');
        end
        xGrid = linspace(0,max(xMax,1),300);
        plot(xGrid,0.5*exp(-xGrid/2),'LineWidth',1.2);
        legend({'Measured NIS','NIS gate','\chi^2(2) reference'}, ...
            'Location','best');
    else
        text(0.1,0.5,'No finite NIS samples');
    end
    grid on;
    xlabel('NIS');
    ylabel('Probability density');
    title('NIS Distribution');

    saveTestFigure(fig,figureFolder,'06_nis_time_and_distribution');
    createdFigures{end+1} = '06_nis_time_and_distribution'; %#ok<SAGROW>
    if closeFiguresAfterSave, close(fig); end
end

% ---------------------------------------------------------
% 图7：新帧、接受、初始化、安全和控制状态
% ---------------------------------------------------------
statusSignals = { ...
    'log_measurement_is_new', ...
    'log_measurement_accepted', ...
    'log_ekf_initialized', ...
    'log_safe_valid', ...
    'log_controller_ok'};

availableStatusSignals = statusSignals(isfield(logs,statusSignals));

if ~isempty(availableStatusSignals)
    fig = figure('Visible',visibilityValue,'Name','Status Signals');
    hold on;
    statusLabels = cell(1,numel(availableStatusSignals));

    for statusIndex = 1:numel(availableStatusSignals)
        signalName = availableStatusSignals{statusIndex};
        [statusTime,statusData] = signalToTimeData(logs.(signalName));
        stairs(statusTime,statusData(:,1) + 1.25*(statusIndex-1), ...
            'LineWidth',1.0);
        statusLabels{statusIndex} = strrep(signalName,'log_','');
    end

    grid on;
    xlabel('Time (s)');
    ylabel('Boolean state with offsets');
    legend(statusLabels,'Interpreter','none','Location','best');
    title('Measurement and Controller Status');

    saveTestFigure(fig,figureFolder,'07_measurement_and_status_states');
    createdFigures{end+1} = '07_measurement_and_status_states'; %#ok<SAGROW>
    if closeFiguresAfterSave, close(fig); end
end

% ---------------------------------------------------------
% 图8：相机速度反馈，仅作为机器人执行诊断
% 当前Base坐标系KF不再使用该速度做位置积分。
% ---------------------------------------------------------
if isfield(logs,'log_v_camera_measured')
    [time,cameraVelocity] = signalToTimeData(logs.log_v_camera_measured);

    fig = figure('Visible',visibilityValue,'Name','Camera Velocity Feedback');
    tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

    nexttile;
    plot(time,cameraVelocity(:,1:min(2,size(cameraVelocity,2))), ...
        'LineWidth',1.1);
    yline(0,':');
    grid on;
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    legend({'v_{cam,x_C}','v_{cam,y_C}'},'Location','best');
    title('Measured Camera XY Velocity in Camera Frame');

    nexttile;
    if isfield(logs,'log_v_camera_measured_valid')
        [validTime,validData] = ...
            signalToTimeData(logs.log_v_camera_measured_valid);
        stairs(validTime,validData(:,1),'LineWidth',1.1);
    else
        text(0.1,0.5,'Validity log unavailable');
    end
    ylim([-0.1 1.1]);
    grid on;
    xlabel('Time (s)');
    ylabel('Valid');
    title('Camera Velocity Feedback Validity');

    saveTestFigure(fig,figureFolder,'08_camera_velocity_feedback');
    createdFigures{end+1} = '08_camera_velocity_feedback'; %#ok<SAGROW>
    if closeFiguresAfterSave, close(fig); end
end

% ---------------------------------------------------------
% 图9：静态相机系测量噪声
% ---------------------------------------------------------
if strcmp(testType,'static') && ...
        isfield(metrics,'staticMeasurement') && ...
        metrics.staticMeasurement.available

    measurement = metrics.staticMeasurement;

    fig = figure('Visible',visibilityValue,'Name','Static Measurement Noise');
    tiledlayout(1,2,'TileSpacing','compact','Padding','compact');

    nexttile;
    scatter(1000*measurement.centeredPosition(:,1), ...
        1000*measurement.centeredPosition(:,2),12,'filled');
    axis equal;
    grid on;
    xlabel('\Delta X_C (mm)');
    ylabel('\Delta Y_C (mm)');
    title('Static Camera-frame Measurement Scatter');

    nexttile;
    histogram(1000*measurement.centeredPosition(:,1), ...
        'Normalization','pdf');
    hold on;
    histogram(1000*measurement.centeredPosition(:,2), ...
        'Normalization','pdf');
    grid on;
    xlabel('Centered measurement (mm)');
    ylabel('Probability density');
    legend({'X_C','Y_C'},'Location','best');
    title('Static Measurement Noise Distribution');

    saveTestFigure(fig,figureFolder,'09_static_measurement_noise');
    createdFigures{end+1} = '09_static_measurement_noise'; %#ok<SAGROW>
    if closeFiguresAfterSave, close(fig); end
end

% ---------------------------------------------------------
% 图10：奇异度诊断
%
% sigma_min：6x7 相机雅可比（[v;omega]，相机坐标系表达）的最小奇异值，
%   由 Redundant Visual Velocity Solver 每个控制周期计算。
% singularity_scale：奇异值缩放。
%   sigma_min <= sigma_stop               -> 0，任务速度完全归零；
%   sigma_stop < sigma_min < sigma_warn   -> smoothstep 平滑降速；
%   sigma_min >= sigma_warn               -> 1，正常速度。
% damping_used：接近奇异时使用的阻尼系数。
% 第三幅图对比 |qdot_raw|（求解器输出）与 |qdot_command|（终极保护后），
% 奇异值缩放与加速度保护的实际效果都在这里。
% 注意：sigma_min<=0 表示关节反馈超时导致 J_camera 被置零，不是真实奇异，
% 绘图中已置为 NaN 跳过。
% ---------------------------------------------------------
if isfield(logs,'log_sigma_min')
    [sigmaTime,sigmaData] = signalToTimeData(logs.log_sigma_min);
    sigmaPlot = sigmaData(:,1);
    sigmaPlot(sigmaPlot <= 0) = NaN;

    fig = figure( ...
        'Visible',visibilityValue, ...
        'Name','Singularity Diagnostics');

    tiledlayout( ...
        3, ...
        1, ...
        'TileSpacing','compact', ...
        'Padding','compact');

    nexttile;
    semilogy(sigmaTime,sigmaPlot,'LineWidth',1.15);
    hold on;
    if isfield(parameters,'singularity_sigma_warn')
        yline( ...
            parameters.singularity_sigma_warn, ...
            '--', ...
            sprintf( ...
                '\\sigma_{warn} = %.4g', ...
                parameters.singularity_sigma_warn));
    end
    if isfield(parameters,'singularity_sigma_stop')
        yline( ...
            parameters.singularity_sigma_stop, ...
            '--', ...
            sprintf( ...
                '\\sigma_{stop} = %.4g', ...
                parameters.singularity_sigma_stop));
    end
    grid on;
    xlabel('Time (s)');
    ylabel('\sigma_{min}');
    title('Minimum Singular Value of J_{camera} (6x7, log scale)');

    nexttile;
    legendEntries = {};
    hold on;
    if isfield(logs,'log_singularity_scale')
        [scaleTime,scaleData] = ...
            signalToTimeData(logs.log_singularity_scale);
        plot(scaleTime,scaleData(:,1),'LineWidth',1.15);
        legendEntries{end+1} = 'singularity_{scale}';
    end
    if isfield(logs,'log_damping_used')
        [dampingTime,dampingData] = ...
            signalToTimeData(logs.log_damping_used);
        plot(dampingTime,dampingData(:,1),'LineWidth',1.0);
        legendEntries{end+1} = 'damping_{used}';
    end
    yline(1,':');
    ylim([-0.05 1.10]);
    grid on;
    xlabel('Time (s)');
    ylabel('Scale');
    if ~isempty(legendEntries)
        legend(legendEntries,'Location','best');
    end
    title('Singularity Scale and Damping');

    nexttile;
    if isfield(logs,'log_qdot_raw') && ...
            isfield(logs,'log_qdot_command')
        [rawTime,rawData] = ...
            signalToTimeData(logs.log_qdot_raw);
        [commandTime,commandData] = ...
            signalToTimeData(logs.log_qdot_command);
        commandOnRawTime = interpolateData( ...
            commandTime, ...
            commandData, ...
            rawTime, ...
            'linear');

        plot(rawTime,sqrt(sum(rawData.^2,2)),'LineWidth',1.0);
        hold on;
        plot( ...
            rawTime, ...
            sqrt(sum(commandOnRawTime.^2,2)), ...
            'LineWidth',1.15);
        legend( ...
            {'|qdot_{raw}|','|qdot_{command}|'}, ...
            'Location','best');
    else
        text(0.1,0.5,'qdot logs unavailable');
    end
    grid on;
    xlabel('Time (s)');
    ylabel('Joint speed (rad/s)');
    title('Solver Output vs Guarded Joint Command');

    saveTestFigure( ...
        fig, ...
        figureFolder, ...
        '10_singularity_diagnostics');

    createdFigures{end+1} = ...
        '10_singularity_diagnostics'; %#ok<SAGROW>

    if closeFiguresAfterSave
        close(fig);
    end
end

%% 10. 生成摘要

summaryFile = fullfile(runFolder,'summary.txt');
writeSummary(summaryFile,safeTestName,safeTestType,timeStamp, ...
    logs,parameters,metrics,missingLogs,createdFigures);

%% 11. 完成提示

fprintf('\n保存完成。\n');
fprintf('日志、参数与指标：%s\n', ...
    fullfile(runFolder,'selected_logs_parameters_metrics.mat'));

if isfield(logs,'log_ekf_state')
    fprintf('KF四状态CSV：%s\n', ...
        fullfile(runFolder,'ekf_state_timeseries.csv'));
end

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

signal = [];
sourceNames = currentModelSignalNames(logName);

if ~isempty(simulationOutput)
    outputNames = simulationOutput.who;

    for sourceIndex = 1:numel(sourceNames)
        sourceName = sourceNames{sourceIndex};
        if any(strcmp(outputNames,sourceName))
            signal = simulationOutput.get(sourceName);
            return;
        end
    end

    if any(strcmp(outputNames,'logsout'))
        logsout = simulationOutput.get('logsout');
        if isa(logsout,'Simulink.SimulationData.Dataset')
            for sourceIndex = 1:numel(sourceNames)
                try
                    element = logsout.getElement(sourceNames{sourceIndex});
                    if ~isempty(element)
                        signal = element.Values;
                        return;
                    end
                catch
                end
            end
        end
    end
end

for sourceIndex = 1:numel(sourceNames)
    sourceName = sourceNames{sourceIndex};
    variableExists = evalin('base',sprintf('exist(''%s'',''var'')',sourceName));
    if variableExists
        signal = evalin('base',sourceName);
        return;
    end
end

end


function sourceNames = currentModelSignalNames(logName)

switch logName
    case {'log_error','log_e'}
        sourceNames = {'image_error','error','e'};
    case 'log_ekf_state'
        sourceNames = {'ekf_state'};
    case 'log_ekf_covariance'
        sourceNames = {'ekf_covariance'};
    case 'log_ekf_velocity'
        sourceNames = {'v_target_hat','ekf_velocity'};
    case {'log_camera_velocity','log_v_camera_measured'}
        sourceNames = {'camera_twist_measured','camera_velocity'};
    case {'log_measurement_is_new','log_target_is_new'}
        sourceNames = {'target_is_new','measurement_is_new'};
    case 'log_safe_valid'
        sourceNames = {'safe_valid'};
    case 'log_controller_ok'
        sourceNames = {'controller_ok'};
    case 'log_ekf_initialized'
        sourceNames = {'ekf_initialized'};
    case 'log_measurement_accepted'
        sourceNames = {'measurement_accepted'};
    case 'log_joint_position'
        sourceNames = {'joint_position'};
    case 'log_joint_velocity'
        sourceNames = {'joint_velocity'};
    case 'log_v_norm_limited'
        sourceNames = {'v_norm_limited'};
    case 'log_saturation_active'
        sourceNames = {'saturation_active','saturation'};
    case 'log_acceleration_active'
        sourceNames = {'acceleration_active','acceleration_limit'};
    case 'log_target_data'
        sourceNames = {'target_data'};
    case 'log_u'
        sourceNames = {'u'};
    case 'log_v'
        sourceNames = {'v'};
    case 'log_target_depth'
        sourceNames = {'target_depth','z'};
    otherwise
        sourceNames = {regexprep(logName,'^log_','')};
end

end


function [logs,missingLogs] = deriveCurrentModelLogs(logs,missingLogs)

if isfield(logs,'log_target_data')
    [targetTime,targetData] = signalToTimeData(logs.log_target_data);
    if size(targetData,2) >= 4
        logs.log_u = timeseries(targetData(:,2),targetTime);
        logs.log_v = timeseries(targetData(:,3),targetTime);
        logs.log_target_depth = timeseries(targetData(:,4),targetTime);
        missingLogs = setdiff(missingLogs, ...
            {'log_u','log_v','log_target_depth'},'stable');

        fx = getBaseScalar('fx',NaN);
        fy = getBaseScalar('fy',NaN);
        cx = getBaseScalar('cx',NaN);
        cy = getBaseScalar('cy',NaN);
        if all(isfinite([fx fy cx cy])) && fx > 0 && fy > 0
            zMeas = [ ...
                (targetData(:,2)-cx).*targetData(:,4)./fx, ...
                (targetData(:,3)-cy).*targetData(:,4)./fy];
            logs.log_z_meas = timeseries(zMeas,targetTime);
            missingLogs(strcmp(missingLogs,'log_z_meas')) = [];
        end
    end
end

if ~isfield(logs,'log_saturation_active') && ...
        isfield(logs,'log_velocity_scale')
    [time,scale] = signalToTimeData(logs.log_velocity_scale);
    logs.log_saturation_active = timeseries(double(scale(:,1)<1-1e-9),time);
    missingLogs(strcmp(missingLogs,'log_saturation_active')) = [];
end

if ~isfield(logs,'log_acceleration_active') && ...
        isfield(logs,'log_acceleration_scale')
    [time,scale] = signalToTimeData(logs.log_acceleration_scale);
    logs.log_acceleration_active = timeseries(double(scale(:,1)<1-1e-9),time);
    missingLogs(strcmp(missingLogs,'log_acceleration_active')) = [];
end

end


function value = getBaseScalar(variableName,defaultValue)

value = defaultValue;
variableExists = evalin('base',sprintf('exist(''%s'',''var'')',variableName));
if variableExists
    candidate = evalin('base',variableName);
    if isnumeric(candidate) && isscalar(candidate)
        value = double(candidate);
    end
end

end


function [time,data] = signalToTimeData(signal)

if isa(signal,'Simulink.SimulationData.Signal')
    signal = signal.Values;
end

if isa(signal,'timeseries')
    time = double(signal.Time(:));
    data = signal.Data;
elseif isstruct(signal) && isfield(signal,'time') && isfield(signal,'signals')
    time = double(signal.time(:));
    data = signal.signals.values;
elseif istimetable(signal)
    time = seconds(signal.Properties.RowTimes - signal.Properties.RowTimes(1));
    data = signal.Variables;
else
    error('不支持的日志格式：%s',class(signal));
end

data = squeeze(data);

if isvector(data)
    data = data(:);
elseif size(data,1) ~= numel(time) && size(data,2) == numel(time)
    data = data.';
elseif size(data,1) ~= numel(time)
    data = reshape(data,numel(time),[]);
end

data = double(data);

end


function [time,diagonalData] = covarianceToDiagonal(signal,stateCount)

if isa(signal,'Simulink.SimulationData.Signal')
    signal = signal.Values;
end

if isa(signal,'timeseries')
    time = double(signal.Time(:));
    rawData = signal.Data;
elseif isstruct(signal) && isfield(signal,'time') && isfield(signal,'signals')
    time = double(signal.time(:));
    rawData = signal.signals.values;
elseif istimetable(signal)
    time = seconds(signal.Properties.RowTimes - signal.Properties.RowTimes(1));
    rawData = signal.Variables;
else
    error('不支持的协方差日志格式：%s',class(signal));
end

sampleCount = numel(time);
diagonalData = nan(sampleCount,stateCount);
rawSize = size(rawData);

if ismatrix(rawData)
    matrixData = double(rawData);

    if size(matrixData,1) == sampleCount && ...
            size(matrixData,2) == stateCount*stateCount
        for sampleIndex = 1:sampleCount
            covarianceMatrix = reshape(matrixData(sampleIndex,:), ...
                stateCount,stateCount);
            diagonalData(sampleIndex,:) = diag(covarianceMatrix).';
        end
    elseif size(matrixData,2) == sampleCount && ...
            size(matrixData,1) == stateCount*stateCount
        matrixData = matrixData.';
        for sampleIndex = 1:sampleCount
            covarianceMatrix = reshape(matrixData(sampleIndex,:), ...
                stateCount,stateCount);
            diagonalData(sampleIndex,:) = diag(covarianceMatrix).';
        end
    else
        error('无法识别协方差矩阵日志尺寸：%s',mat2str(rawSize));
    end
elseif ndims(rawData) == 3
    if rawSize(1) == stateCount && rawSize(2) == stateCount && ...
            rawSize(3) == sampleCount
        for sampleIndex = 1:sampleCount
            covarianceMatrix = double(rawData(:,:,sampleIndex));
            diagonalData(sampleIndex,:) = diag(covarianceMatrix).';
        end
    elseif rawSize(1) == sampleCount && rawSize(2) == stateCount && ...
            rawSize(3) == stateCount
        for sampleIndex = 1:sampleCount
            covarianceMatrix = double(squeeze(rawData(sampleIndex,:,:)));
            diagonalData(sampleIndex,:) = diag(covarianceMatrix).';
        end
    elseif rawSize(1) == stateCount && rawSize(2) == sampleCount && ...
            rawSize(3) == stateCount
        for sampleIndex = 1:sampleCount
            covarianceMatrix = double(squeeze(rawData(:,sampleIndex,:)));
            diagonalData(sampleIndex,:) = diag(covarianceMatrix).';
        end
    else
        error('无法识别三维协方差日志尺寸：%s',mat2str(rawSize));
    end
else
    error('无法识别协方差日志维度：%s',mat2str(rawSize));
end

end


function result = interpolateData(sourceTime,sourceData,queryTime,method)

sourceTime = double(sourceTime(:));
queryTime = double(queryTime(:));
sourceData = double(sourceData);

[sourceTime,uniqueIndex] = unique(sourceTime,'stable');
sourceData = sourceData(uniqueIndex,:);

if isempty(sourceTime)
    result = nan(numel(queryTime),size(sourceData,2));
    return;
end

if numel(sourceTime) < 2
    result = repmat(sourceData(1,:),numel(queryTime),1);
    return;
end

result = interp1(sourceTime,sourceData,queryTime,method,'extrap');

end


function logicalData = interpolateBooleanSignal(signal,queryTime)

[sourceTime,sourceData] = signalToTimeData(signal);
interpolatedData = interpolateData(sourceTime,sourceData(:,1), ...
    queryTime,'previous');
logicalData = interpolatedData > 0.5;

end


function metrics = calculateMetrics(logs,parameters,options)

metrics = struct();
metrics.testType = options.testType;
metrics.analysisStartSec = options.analysisStartSec;
metrics.testDurationSec = NaN;

allLogNames = fieldnames(logs);
for index = 1:numel(allLogNames)
    try
        [time,~] = signalToTimeData(logs.(allLogNames{index}));
        if numel(time) >= 2
            metrics.testDurationSec = time(end)-time(1);
            break;
        end
    catch
    end
end

% Initialize vector-interface diagnostics as unavailable; the current q_dot
% model wires parameters as individual Constant blocks, not a parameter vector.
metrics.parameterVector = struct('available',false);

% 图像误差。
metrics.imageError = struct('available',false);
if isfield(logs,'log_error')
    [time,errorData] = signalToTimeData(logs.log_error);
    mask = time >= time(1)+options.analysisStartSec & ...
        all(isfinite(errorData),2);

    if any(mask)
        selected = errorData(mask,:);
        metrics.imageError.available = true;
        metrics.imageError.rmsNormalized = sqrt(mean(selected.^2,1,'omitnan'));
        metrics.imageError.maxAbsNormalized = max(abs(selected),[],1,'omitnan');

        scale = ones(1,size(selected,2));
        if isfield(parameters,'fx') && numel(scale)>=1, scale(1)=parameters.fx; end
        if isfield(parameters,'fy') && numel(scale)>=2, scale(2)=parameters.fy; end
        pixelError = selected.*scale;
        metrics.imageError.rmsPixels = sqrt(mean(pixelError.^2,1,'omitnan'));
        metrics.imageError.maxAbsPixels = max(abs(pixelError),[],1,'omitnan');
    end
end

% 新帧和接受率。
metrics.measurement = struct('available',false);
if isfield(logs,'log_measurement_is_new')
    [time,newData] = signalToTimeData(logs.log_measurement_is_new);
    analysisMask = time >= time(1)+options.analysisStartSec;
    newMask = newData(:,1)>0.5;
    safeMask = true(size(newMask));
    acceptedMask = false(size(newMask));

    if isfield(logs,'log_safe_valid')
        safeMask = interpolateBooleanSignal(logs.log_safe_valid,time);
    end
    if isfield(logs,'log_measurement_accepted')
        acceptedMask = interpolateBooleanSignal(logs.log_measurement_accepted,time);
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

    effectiveDuration = metrics.testDurationSec-options.analysisStartSec;
    if isfinite(effectiveDuration) && effectiveDuration>0
        metrics.measurement.newFrameRateHz = ...
            metrics.measurement.newFrameCount/effectiveDuration;
    else
        metrics.measurement.newFrameRateHz = NaN;
    end
end

% NIS。
metrics.nis = struct('available',false);
if isfield(logs,'log_innovation_nis')
    [time,nisData] = signalToTimeData(logs.log_innovation_nis);
    nisVector = nisData(:,1);
    mask = time >= time(1)+options.analysisStartSec & isfinite(nisVector);

    if isfield(logs,'log_measurement_is_new')
        mask = mask & interpolateBooleanSignal(logs.log_measurement_is_new,time);
    end
    if isfield(logs,'log_safe_valid')
        mask = mask & interpolateBooleanSignal(logs.log_safe_valid,time);
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
                100*mean(selectedNis>parameters.ekf_gate_threshold,'omitnan');
        else
            metrics.nis.gate = NaN;
            metrics.nis.gateExceedancePercent = NaN;
        end
    end
end

% 完整四状态指标。
metrics.ekfState = struct('available',false);
if isfield(logs,'log_ekf_state')
    [time,stateData] = signalToTimeData(logs.log_ekf_state);

    if size(stateData,2)>=4
        mask = time >= time(1)+options.analysisStartSec & ...
            all(isfinite(stateData(:,1:4)),2);

        if any(mask)
            selected = stateData(mask,1:4);
            metrics.ekfState.available = true;
            metrics.ekfState.mean = mean(selected,1,'omitnan');
            metrics.ekfState.rms = sqrt(mean(selected.^2,1,'omitnan'));
            metrics.ekfState.maxAbs = max(abs(selected),[],1,'omitnan');
            metrics.ekfState.final = selected(end,:);
            metrics.ekfState.positionRms = ...
                sqrt(mean(selected(:,1:2).^2,1,'omitnan'));
            metrics.ekfState.velocityRms = ...
                sqrt(mean(selected(:,3:4).^2,1,'omitnan'));
        end
    end
end

% 检查v_target_hat是否与ekf_state(3:4)一致。
metrics.stateVelocityConsistency = struct('available',false);
if isfield(logs,'log_ekf_state') && isfield(logs,'log_ekf_velocity')
    [ts,stateData] = signalToTimeData(logs.log_ekf_state);
    [tv,velocityData] = signalToTimeData(logs.log_ekf_velocity);

    if size(stateData,2)>=4 && size(velocityData,2)>=2
        velocityOnStateTime = interpolateData(tv,velocityData(:,1:2),ts,'linear');
        mask = ts >= ts(1)+options.analysisStartSec & ...
            all(isfinite(stateData(:,3:4)),2) & ...
            all(isfinite(velocityOnStateTime),2);

        if any(mask)
            difference = velocityOnStateTime(mask,:) - stateData(mask,3:4);
            metrics.stateVelocityConsistency.available = true;
            metrics.stateVelocityConsistency.maxAbsDifference = ...
                max(abs(difference),[],1,'omitnan');
            metrics.stateVelocityConsistency.rmsDifference = ...
                sqrt(mean(difference.^2,1,'omitnan'));
        end
    end
end

% 静态相机系测量噪声，用于R_ekf初值。
metrics.staticMeasurement = struct('available',false);
if strcmp(options.testType,'static') && isfield(logs,'log_z_meas')
    [time,zData] = signalToTimeData(logs.log_z_meas);
    mask = time >= time(1)+options.analysisStartSec & ...
        all(isfinite(zData(:,1:2)),2);

    if isfield(logs,'log_measurement_is_new')
        mask = mask & interpolateBooleanSignal(logs.log_measurement_is_new,time);
    end
    if isfield(logs,'log_safe_valid')
        mask = mask & interpolateBooleanSignal(logs.log_safe_valid,time);
    end

    selectedPosition = zData(mask,1:2);
    if size(selectedPosition,1)>=10
        meanPosition = mean(selectedPosition,1,'omitnan');
        centeredPosition = selectedPosition-meanPosition;
        measurementCovariance = cov(selectedPosition,'omitrows');
        sigmaMeters = std(selectedPosition,0,1,'omitnan');
        sigmaPixels = nan(1,2);

        if isfield(parameters,'Z_hat') && parameters.Z_hat>0
            if isfield(parameters,'fx')
                sigmaPixels(1)=sigmaMeters(1)*parameters.fx/parameters.Z_hat;
            end
            if isfield(parameters,'fy')
                sigmaPixels(2)=sigmaMeters(2)*parameters.fy/parameters.Z_hat;
            end
        end

        metrics.staticMeasurement.available = true;
        metrics.staticMeasurement.sampleCount = size(selectedPosition,1);
        metrics.staticMeasurement.meanPosition = meanPosition;
        metrics.staticMeasurement.centeredPosition = centeredPosition;
        metrics.staticMeasurement.sigmaMeters = sigmaMeters;
        metrics.staticMeasurement.sigmaPixels = sigmaPixels;
        metrics.staticMeasurement.covariance = measurementCovariance;
        metrics.staticMeasurement.suggestedR = measurementCovariance;
    end
end

% 协方差指标。
metrics.covariance = struct('available',false);
if isfield(logs,'log_ekf_covariance')
    [time,pDiag] = covarianceToDiagonal(logs.log_ekf_covariance,4);
    mask = time >= time(1)+options.analysisStartSec & all(isfinite(pDiag),2);

    if any(mask)
        sigmaData = sqrt(max(pDiag(mask,:),0));
        metrics.covariance.available = true;
        metrics.covariance.meanSigma = mean(sigmaData,1,'omitnan');
        metrics.covariance.finalSigma = sigmaData(end,:);
        metrics.covariance.maxSigma = max(sigmaData,[],1,'omitnan');
    end
end

% 相机速度反馈，仅用于机器人执行诊断。
metrics.cameraVelocity = struct('available',false);
if isfield(logs,'log_v_camera_measured')
    [time,cameraVelocity] = signalToTimeData(logs.log_v_camera_measured);
    mask = time >= time(1)+options.analysisStartSec & ...
        all(isfinite(cameraVelocity),2);

    if any(mask)
        selected = cameraVelocity(mask,:);
        metrics.cameraVelocity.available = true;
        metrics.cameraVelocity.rms = sqrt(mean(selected.^2,1,'omitnan'));
        metrics.cameraVelocity.maxAbs = max(abs(selected),[],1,'omitnan');

        if isfield(logs,'log_v_camera_measured_valid')
            validMask = interpolateBooleanSignal( ...
                logs.log_v_camera_measured_valid,time(mask));
            metrics.cameraVelocity.validRatioPercent = ...
                100*mean(validMask,'omitnan');
        else
            metrics.cameraVelocity.validRatioPercent = NaN;
        end
    end
end

% 控制器指标。
metrics.controller = struct('available',false);
if isfield(logs,'log_v_issued')
    [time,issuedVelocity] = signalToTimeData(logs.log_v_issued);
    mask = time >= time(1)+options.analysisStartSec & ...
        all(isfinite(issuedVelocity),2);

    if any(mask)
        selected = issuedVelocity(mask,:);
        speed = sqrt(sum(selected.^2,2));
        metrics.controller.available = true;
        metrics.controller.maximumIssuedSpeed = max(speed,[],'omitnan');
        metrics.controller.rmsIssuedSpeed = sqrt(mean(speed.^2,'omitnan'));
    end
end

if isfield(logs,'log_v_p') && isfield(logs,'log_v_ff')
    [tp,vp] = signalToTimeData(logs.log_v_p);
    [tff,vff] = signalToTimeData(logs.log_v_ff);
    vff = interpolateData(tff,vff,tp,'linear');
    mask = tp >= tp(1)+options.analysisStartSec & ...
        all(isfinite(vp),2) & all(isfinite(vff),2);

    if any(mask)
        vpNorm = sqrt(sum(vp(mask,:).^2,2));
        vffNorm = sqrt(sum(vff(mask,:).^2,2));
        rmsVp = sqrt(mean(vpNorm.^2,'omitnan'));
        rmsVff = sqrt(mean(vffNorm.^2,'omitnan'));
        metrics.controller.rmsVp = rmsVp;
        metrics.controller.rmsVff = rmsVff;
        metrics.controller.feedforwardToProportionalRmsRatio = ...
            rmsVff/max(rmsVp,eps);
    end
end

% 奇异度指标。
metrics.singularity = struct('available',false);

if isfield(logs,'log_sigma_min')
    [time,sigmaData] = signalToTimeData(logs.log_sigma_min);
    sigmaVector = sigmaData(:,1);
    mask = time >= time(1)+options.analysisStartSec & ...
        isfinite(sigmaVector);

    if any(mask)
        selectedSigma = sigmaVector(mask);
        selectedTime = time(mask);
        [minimumSigma,minimumIndex] = min(selectedSigma);

        metrics.singularity.available = true;
        metrics.singularity.minimum = minimumSigma;
        metrics.singularity.median = median(selectedSigma);
        metrics.singularity.maximum = max(selectedSigma);
        metrics.singularity.timeOfMinimum = ...
            selectedTime(minimumIndex);

        if isfield(parameters,'singularity_sigma_stop') && ...
                isfield(parameters,'singularity_sigma_warn')
            metrics.singularity.stopThreshold = ...
                parameters.singularity_sigma_stop;
            metrics.singularity.warnThreshold = ...
                parameters.singularity_sigma_warn;
            metrics.singularity.belowWarnPercent = ...
                100*mean(selectedSigma < ...
                parameters.singularity_sigma_warn);
            metrics.singularity.belowStopPercent = ...
                100*mean(selectedSigma < ...
                parameters.singularity_sigma_stop);
        end
    end
end

if isfield(logs,'log_singularity_scale')
    [time,scaleData] = ...
        signalToTimeData(logs.log_singularity_scale);
    scaleVector = scaleData(:,1);
    mask = time >= time(1)+options.analysisStartSec & ...
        isfinite(scaleVector);

    if any(mask)
        metrics.singularity.scaleAvailable = true;
        metrics.singularity.scaleMinimum = min(scaleVector(mask));
        metrics.singularity.scaleMedian = median(scaleVector(mask));

        if isfield(logs,'log_control_active')
            activeMask = mask & interpolateBooleanSignal( ...
                logs.log_control_active,time);

            if any(activeMask)
                metrics.singularity.scaleMedianWhileActive = ...
                    median(scaleVector(activeMask));
                metrics.singularity.scaleMeanWhileActive = ...
                    mean(scaleVector(activeMask));
                metrics.singularity.activeSamplePercent = ...
                    100*mean(activeMask);
            end
        end
    end
end

end


function saveTestFigure(fig,figureFolder,fileBaseName)

exportgraphics(fig,fullfile(figureFolder,[fileBaseName,'.png']), ...
    'Resolution',200);
savefig(fig,fullfile(figureFolder,[fileBaseName,'.fig']));

end


function copyConfigurationFiles(runFolder)

scriptDirectory = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDirectory);
initSource = fullfile(projectRoot,'config', ...
    'init_single_camera_q_dot_tracking.m');

if isfile(initSource)
    copyfile(initSource,fullfile(runFolder, ...
        'init_single_camera_q_dot_tracking_snapshot.m'));
end

homeDirectory = getenv('HOME');
yamlSearchRoot = fullfile(homeDirectory,'franka_ros2_ws','src');

if exist(yamlSearchRoot,'dir')
    yamlCandidates = dir(fullfile(yamlSearchRoot,'**','velocity_servo_tag.yaml'));
    if ~isempty(yamlCandidates)
        yamlSource = fullfile(yamlCandidates(1).folder,yamlCandidates(1).name);
        copyfile(yamlSource,fullfile(runFolder, ...
            'velocity_servo_tag_snapshot.yaml'));
    end
end

end


function writeSummary(summaryFile,testName,testType,timeStamp, ...
    logs,parameters,metrics,missingLogs,createdFigures)

fileID = fopen(summaryFile,'w');
if fileID<0
    warning('无法创建测试摘要：%s',summaryFile);
    return;
end
cleanupObject = onCleanup(@() fclose(fileID)); %#ok<NASGU>

fprintf(fileID,'Single-camera Base-frame KF tuning test summary\n');
fprintf(fileID,'================================================\n');
fprintf(fileID,'Test name: %s\n',testName);
fprintf(fileID,'Test type: %s\n',testType);
fprintf(fileID,'Timestamp: %s\n',timeStamp);
fprintf(fileID,'Analysis start offset: %.3f s\n\n',metrics.analysisStartSec);

fprintf(fileID,'Coordinate conventions\n');
fprintf(fileID,'----------------------\n');
fprintf(fileID,'z_meas: [X_C,Y_C], camera frame.\n');
fprintf(fileID,'ekf_state: [X_B,Y_B,Vx_B,Vy_B], Base frame.\n');
fprintf(fileID,'v_p, v_ff, v_issued: camera frame.\n\n');

fprintf(fileID,'Available logs: %s\n',strjoin(fieldnames(logs),', '));
if isempty(missingLogs)
    fprintf(fileID,'Missing logs: none\n\n');
else
    fprintf(fileID,'Missing logs: %s\n\n',strjoin(missingLogs,', '));
end

fprintf(fileID,'Controller and KF parameters\n');
fprintf(fileID,'----------------------------\n');

scalarNames = {'Ts','fx','fy','cx','cy','Z_hat','Kpx','Kpy','k_ff', ...
    'ekf_gate_threshold','required_valid_frames', ...
    'measurement_reset_timeout','task_velocity_filter_tau', ...
    'target_timeout_sec', ...
    'joint_state_timeout_sec','camera_velocity_feedback_timeout_sec'};

for index = 1:numel(scalarNames)
    name = scalarNames{index};
    if isfield(parameters,name)
        fprintf(fileID,'%s: %.12g\n',name,double(parameters.(name)));
    end
end

for nameCell = {'P0','Q_ekf','R_ekf'}
    name = nameCell{1};
    if isfield(parameters,name)
        fprintf(fileID,'%s:\n',name);
        writeMatrix(fileID,double(parameters.(name)));
    end
end

if metrics.parameterVector.available
    fprintf(fileID,'controller_parameters length: %d\n', ...
        metrics.parameterVector.length);
    fprintf(fileID,'Expected current length: %d\n', ...
        metrics.parameterVector.expectedLength);
    if ~metrics.parameterVector.lengthCorrect
        fprintf(fileID,['WARNING: controller_parameters must contain 12 entries: ' ...
            '[Ts,Z,Kpx,Kpy,k_ff,gate,enable_p,enable_ff,controller_enable,reserved,v_xy_max,a_xy_max].\n']);
    end
end

fprintf(fileID,'\nMeasured indicators\n');
fprintf(fileID,'-------------------\n');

if isfinite(metrics.testDurationSec)
    fprintf(fileID,'Test duration: %.6f s\n',metrics.testDurationSec);
end

if metrics.measurement.available
    fprintf(fileID,'New frame count: %d\n',metrics.measurement.newFrameCount);
    fprintf(fileID,'Valid new frame count: %d\n', ...
        metrics.measurement.validNewFrameCount);
    fprintf(fileID,'Accepted new frame count: %d\n', ...
        metrics.measurement.acceptedNewFrameCount);
    fprintf(fileID,'New frame rate: %.6f Hz\n', ...
        metrics.measurement.newFrameRateHz);
    fprintf(fileID,'Measurement acceptance ratio: %.6f %%\n', ...
        metrics.measurement.acceptanceRatioPercent);
end

if metrics.nis.available
    fprintf(fileID,'NIS sample count: %d\n',metrics.nis.sampleCount);
    fprintf(fileID,'NIS mean: %.12g\n',metrics.nis.mean);
    fprintf(fileID,'NIS median: %.12g\n',metrics.nis.median);
    fprintf(fileID,'NIS 95th percentile: %.12g\n',metrics.nis.percentile95);
    fprintf(fileID,'NIS 99th percentile: %.12g\n',metrics.nis.percentile99);
    fprintf(fileID,'NIS gate: %.12g\n',metrics.nis.gate);
    fprintf(fileID,'NIS gate exceedance: %.6f %%\n', ...
        metrics.nis.gateExceedancePercent);
end

if metrics.imageError.available
    writeVector(fileID,'Image error RMS normalized', ...
        metrics.imageError.rmsNormalized);
    writeVector(fileID,'Image error max abs normalized', ...
        metrics.imageError.maxAbsNormalized);
    writeVector(fileID,'Image error RMS pixels', ...
        metrics.imageError.rmsPixels);
    writeVector(fileID,'Image error max abs pixels', ...
        metrics.imageError.maxAbsPixels);
end

if metrics.ekfState.available
    fprintf(fileID,'\nBase-frame KF state [X_B Y_B Vx_B Vy_B]\n');
    fprintf(fileID,'-----------------------------------------\n');
    writeVector(fileID,'State mean',metrics.ekfState.mean);
    writeVector(fileID,'State RMS',metrics.ekfState.rms);
    writeVector(fileID,'State max abs',metrics.ekfState.maxAbs);
    writeVector(fileID,'Final state',metrics.ekfState.final);
    writeVector(fileID,'Position RMS (m)',metrics.ekfState.positionRms);
    writeVector(fileID,'Velocity RMS (m/s)',metrics.ekfState.velocityRms);
end

if metrics.stateVelocityConsistency.available
    writeVector(fileID,'Max abs difference: log_ekf_velocity - state(3:4)', ...
        metrics.stateVelocityConsistency.maxAbsDifference);
    writeVector(fileID,'RMS difference: log_ekf_velocity - state(3:4)', ...
        metrics.stateVelocityConsistency.rmsDifference);
end

if metrics.staticMeasurement.available
    fprintf(fileID,'\nStatic camera-frame measurement noise\n');
    fprintf(fileID,'-------------------------------------\n');
    fprintf(fileID,'Valid new samples: %d\n', ...
        metrics.staticMeasurement.sampleCount);
    writeVector(fileID,'Measurement sigma (m)', ...
        metrics.staticMeasurement.sigmaMeters);
    writeVector(fileID,'Measurement sigma (pixels)', ...
        metrics.staticMeasurement.sigmaPixels);
    fprintf(fileID,'Suggested camera-frame R_ekf:\n');
    writeMatrix(fileID,metrics.staticMeasurement.suggestedR);
end

if metrics.covariance.available
    writeVector(fileID,'Mean state sigma',metrics.covariance.meanSigma);
    writeVector(fileID,'Final state sigma',metrics.covariance.finalSigma);
    writeVector(fileID,'Maximum state sigma',metrics.covariance.maxSigma);
end

if metrics.cameraVelocity.available
    writeVector(fileID,'Measured camera velocity RMS (m/s)', ...
        metrics.cameraVelocity.rms);
    writeVector(fileID,'Measured camera velocity max abs (m/s)', ...
        metrics.cameraVelocity.maxAbs);
    fprintf(fileID,'Camera velocity feedback valid ratio: %.6f %%\n', ...
        metrics.cameraVelocity.validRatioPercent);
end

if metrics.controller.available
    fprintf(fileID,'Maximum issued XY speed: %.12g m/s\n', ...
        metrics.controller.maximumIssuedSpeed);
    fprintf(fileID,'RMS issued XY speed: %.12g m/s\n', ...
        metrics.controller.rmsIssuedSpeed);

    if isfield(metrics.controller,'feedforwardToProportionalRmsRatio')
        fprintf(fileID,'RMS(v_ff)/RMS(v_p): %.12g\n', ...
            metrics.controller.feedforwardToProportionalRmsRatio);
    end
end

if metrics.singularity.available
    fprintf(fileID,'\nSingularity diagnostics (6x7 camera Jacobian)\n');
    fprintf(fileID,'----------------------------------------------\n');
    fprintf(fileID,'sigma_min minimum: %.12g at %.6f s\n', ...
        metrics.singularity.minimum, ...
        metrics.singularity.timeOfMinimum);
    fprintf(fileID,'sigma_min median: %.12g\n', ...
        metrics.singularity.median);
    fprintf(fileID,'sigma_min maximum: %.12g\n', ...
        metrics.singularity.maximum);

    if isfield(metrics.singularity,'warnThreshold')
        fprintf(fileID, ...
            'sigma_warn: %.12g | samples below: %.6f %%\n', ...
            metrics.singularity.warnThreshold, ...
            metrics.singularity.belowWarnPercent);
        fprintf(fileID, ...
            'sigma_stop: %.12g | samples below: %.6f %%\n', ...
            metrics.singularity.stopThreshold, ...
            metrics.singularity.belowStopPercent);
    end
end

if isfield(metrics.singularity,'scaleAvailable')
    fprintf(fileID,'singularity_scale minimum: %.12g\n', ...
        metrics.singularity.scaleMinimum);
    fprintf(fileID,'singularity_scale median: %.12g\n', ...
        metrics.singularity.scaleMedian);

    if isfield(metrics.singularity,'scaleMedianWhileActive')
        fprintf(fileID, ...
            'singularity_scale median while control_active: %.12g\n', ...
            metrics.singularity.scaleMedianWhileActive);
        fprintf(fileID, ...
            'control_active sample ratio: %.6f %%\n', ...
            metrics.singularity.activeSamplePercent);
    end
end

if metrics.singularity.available || ...
        isfield(metrics.singularity,'scaleAvailable')
    fprintf(fileID,['Interpretation: sigma_min is the smallest singular value of the ' ...
        'camera-frame 6x7 Jacobian. The solver scales qdot_raw by ' ...
        'singularity_scale = smoothstep(sigma_min) between sigma_stop and ' ...
        'sigma_warn, and returns zero below sigma_stop, so a small sigma_min ' ...
        'slows or freezes the task no matter how large the image error is.\n' ...
        'sigma_min <= 0 samples mean the joint feedback timed out and ' ...
        'J_camera was zeroed, not a real singularity.\n']);
end

fprintf(fileID,'\nHow to interpret this test\n');
fprintf(fileID,'--------------------------\n');

switch testType
    case 'static'
        fprintf(fileID,['1. Robot and target are stationary: Vx_B,Vy_B should approach zero.\n' ...
            '2. Use static z_meas covariance as the camera-frame R_ekf initial estimate.\n' ...
            '3. NIS should mostly stay below the gate.\n']);
    case 'moving_target'
        fprintf(fileID,['1. Observe whether Base-frame velocity follows direction changes without excessive spikes.\n' ...
            '2. Smooth but delayed velocity suggests Q is too small.\n' ...
            '3. Noisy velocity suggests Q is too large or R is too small.\n']);
    case 'moving_camera'
        fprintf(fileID,['1. With a fixed target, Base-frame Vx_B,Vy_B should remain near zero while the robot moves.\n' ...
            '2. Motion-correlated drift points first to T_camera2base direction, timing, Z_hat, or hand-eye calibration.\n' ...
            '3. Do not compare camera-frame z_meas directly with Base-frame X_B,Y_B.\n']);
    case 'closed_loop'
        fprintf(fileID,['1. Compare feedforward off/on tests using image-error RMS and issued-speed RMS.\n' ...
            '2. v_ff is useful only when tracking error improves without state spikes.\n']);
    otherwise
        fprintf(fileID,['Use the four-state, covariance, NIS, status and controller plots to separate ' ...
            'coordinate-transform, measurement-noise, model and controller problems.\n']);
end

fprintf(fileID,'\nCreated figures\n');
fprintf(fileID,'---------------\n');
for index = 1:numel(createdFigures)
    fprintf(fileID,'%s\n',createdFigures{index});
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
