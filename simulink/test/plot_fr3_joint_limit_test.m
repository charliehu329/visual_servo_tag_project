%% plot_fr3_joint_and_command_test.m
% FR3关节位置、命令速度、命令加速度综合测试图
%
% 功能：
% 1. 自动读取：
%       log_joint_position
%       log_command_velocity
%       log_command_acceleration
% 2. 绘制3个Figure：
%       - 关节位置
%       - 命令速度
%       - 命令加速度
% 3. 在图中标出：
%       - 位置硬限位
%       - 中间60%回中边界
%       - 速度限幅
%       - 加速度限幅
% 4. 自动保存PNG、FIG、CSV。
%
% 使用：
%   完成一次Simulink测试后直接运行：
%
%       plot_fr3_joint_and_command_test
%
% 日志要求：
%   log_joint_position         : 7维关节位置，单位rad
%   log_command_velocity       : 7维关节命令速度，单位rad/s
%   log_command_acceleration   : 7维关节命令加速度，单位rad/s^2

%% =========================================================
%% 1. 用户可调参数
%% =========================================================

% 与Mapper一致：中间60%不干预，越过该边界后逐渐回中。
centerFreeRatio = 0.60;

% 与velocity_command_node一致：
% 最终关节速度限幅 = FR3官方限速 * maxVelocityScale
maxVelocityScale = 0.80;

% 与velocity_command_node一致：单位 rad/s^2
maxJointAccelerations = [ ...
    0.50;
    0.50;
    0.50;
    0.50;
    0.50;
    0.50;
    0.50];

% FR3官方关节速度上限（与你Python代码一致）
fr3MaxJointVelocitiesOfficial = [ ...
    2.62;
    2.62;
    2.62;
    2.62;
    5.26;
    4.18;
    5.26];

% 位置限位（来自fr3.urdf）
qMin = [ ...
    -2.9007;
    -1.8361;
    -2.9007;
    -3.0770;
    -2.8763;
     0.4398;
    -3.0508];

qMax = [ ...
     2.9007;
     1.8361;
     2.9007;
    -0.1169;
     2.8763;
     4.6216;
     3.0508];

%% =========================================================
%% 2. 派生量
%% =========================================================

qMid = 0.5*(qMin + qMax);
qHalfRange = 0.5*(qMax - qMin);

qCenterLower = qMid - centerFreeRatio*qHalfRange;
qCenterUpper = qMid + centerFreeRatio*qHalfRange;

commandVelocityLimit = ...
    fr3MaxJointVelocitiesOfficial * maxVelocityScale;

commandAccelerationLimit = maxJointAccelerations(:);

%% =========================================================
%% 3. 自动读取日志
%% =========================================================

jointSignal = findLoggedSignal('log_joint_position');
velocitySignal = findLoggedSignal('log_command_velocity');
accelerationSignal = findLoggedSignal('log_command_acceleration');

if isempty(jointSignal)
    error(['没有找到log_joint_position。请确认模型中已经记录' ...
        '7维关节位置日志。']);
end

if isempty(velocitySignal)
    error(['没有找到log_command_velocity。请确认模型中已经记录' ...
        '7维命令速度日志。']);
end

if isempty(accelerationSignal)
    error(['没有找到log_command_acceleration。请确认模型中已经记录' ...
        '7维命令加速度日志。']);
end

[timeQ,qData] = signalToTimeData(jointSignal);
[timeV,vData] = signalToTimeData(velocitySignal);
[timeA,aData] = signalToTimeData(accelerationSignal);

if size(qData,2) ~= 7
    error('log_joint_position必须包含7列，当前尺寸为%s。', ...
        mat2str(size(qData)));
end

if size(vData,2) ~= 7
    error('log_command_velocity必须包含7列，当前尺寸为%s。', ...
        mat2str(size(vData)));
end

if size(aData,2) ~= 7
    error('log_command_acceleration必须包含7列，当前尺寸为%s。', ...
        mat2str(size(aData)));
end

%% =========================================================
%% 4. 关节位置指标
%% =========================================================

lowerMargin = qData - qMin.';
upperMargin = qMax.' - qData;

% 每个时刻距离最近硬限位的距离
nearestMargin = min(lowerMargin,upperMargin);

% 0%=关节中点，100%=硬限位
normalizedPosition = ...
    (qData - qMid.') ./ qHalfRange.';

limitUsagePercent = 100*abs(normalizedPosition);

minimumMargin = min(nearestMargin,[],1,'omitnan');
maximumUsagePercent = max(limitUsagePercent,[],1,'omitnan');
finalPosition = qData(end,:);

%% =========================================================
%% 5. 命令速度指标
%% =========================================================

velocityUsagePercent = ...
    100 * abs(vData) ./ commandVelocityLimit.';

maximumVelocityUsagePercent = ...
    max(velocityUsagePercent,[],1,'omitnan');

maximumVelocityAbs = max(abs(vData),[],1,'omitnan');
finalVelocity = vData(end,:);

%% =========================================================
%% 6. 命令加速度指标
%% =========================================================

accelerationUsagePercent = ...
    100 * abs(aData) ./ commandAccelerationLimit.';

maximumAccelerationUsagePercent = ...
    max(accelerationUsagePercent,[],1,'omitnan');

maximumAccelerationAbs = max(abs(aData),[],1,'omitnan');
finalAcceleration = aData(end,:);

%% =========================================================
%% 7. Figure 1：关节位置
%% =========================================================

fig1 = figure( ...
    'Name','FR3 Joint Position Monitor', ...
    'Position',[80 60 1500 900]);

layout1 = tiledlayout(4,2, ...
    'TileSpacing','compact', ...
    'Padding','compact');

title(layout1, ...
    'FR3 Joint Positions, Hard Limits and 60% Center-Free Region');

for jointIndex = 1:7
    ax = nexttile(layout1,jointIndex);

    plot(timeQ,qData(:,jointIndex), ...
        'LineWidth',1.25);
    hold on;

    yline(qMin(jointIndex),'r--', ...
        'Lower hard limit', ...
        'LabelHorizontalAlignment','left');

    yline(qMax(jointIndex),'r--', ...
        'Upper hard limit', ...
        'LabelHorizontalAlignment','left');

    yline(qCenterLower(jointIndex),'k:', ...
        '60% lower boundary', ...
        'LabelHorizontalAlignment','left');

    yline(qCenterUpper(jointIndex),'k:', ...
        '60% upper boundary', ...
        'LabelHorizontalAlignment','left');

    yline(qMid(jointIndex),'b-.','Center', ...
        'LabelHorizontalAlignment','left');

    grid on;
    xlabel('Time (s)');
    ylabel(sprintf('q_%d (rad)',jointIndex));

    ylimPadding = 0.08*(qMax(jointIndex)-qMin(jointIndex));
    ylim([ ...
        qMin(jointIndex)-ylimPadding, ...
        qMax(jointIndex)+ylimPadding]);

    if maximumUsagePercent(jointIndex) >= 100
        titleColor = [0.85 0.1 0.1];
        stateText = 'HARD LIMIT EXCEEDED';
    elseif maximumUsagePercent(jointIndex) >= 60
        titleColor = [0.85 0.45 0.05];
        stateText = 'CENTERING ACTIVE';
    else
        titleColor = [0.1 0.5 0.1];
        stateText = 'CENTER REGION';
    end

    title(ax,sprintf( ...
        ['Joint %d | %s | min margin %.4f rad | ' ...
         'max usage %.1f%%'], ...
        jointIndex, ...
        stateText, ...
        minimumMargin(jointIndex), ...
        maximumUsagePercent(jointIndex)), ...
        'Color',titleColor);

    if jointIndex == 1
        legend( ...
            {'q','Lower hard limit','Upper hard limit', ...
             '60%% lower','60%% upper','Center'}, ...
            'Location','best');
    end
end

axSummary1 = nexttile(layout1,8);
axis(axSummary1,'off');

summaryLines1 = strings(9,1);
summaryLines1(1) = "Joint position summary";
summaryLines1(2) = "-------------------------------";

for jointIndex = 1:7
    summaryLines1(jointIndex+2) = sprintf( ...
        ['J%d: final=% .4f rad | min margin=%.4f rad | ' ...
         'max usage=%.1f%%'], ...
        jointIndex, ...
        finalPosition(jointIndex), ...
        minimumMargin(jointIndex), ...
        maximumUsagePercent(jointIndex));
end

text(axSummary1,0.02,0.98,summaryLines1, ...
    'Units','normalized', ...
    'VerticalAlignment','top', ...
    'FontName','Consolas', ...
    'FontSize',10);

%% =========================================================
%% 8. Figure 2：命令速度
%% =========================================================

fig2 = figure( ...
    'Name','FR3 Command Velocity Monitor', ...
    'Position',[100 80 1500 900]);

layout2 = tiledlayout(4,2, ...
    'TileSpacing','compact', ...
    'Padding','compact');

title(layout2, ...
    sprintf(['FR3 Command Velocity | Limit = Official Limit ' ...
    '\\times %.2f'],maxVelocityScale));

for jointIndex = 1:7
    ax = nexttile(layout2,jointIndex);

    plot(timeV,vData(:,jointIndex), ...
        'LineWidth',1.25);
    hold on;

    yline(commandVelocityLimit(jointIndex),'r--', ...
        'Upper velocity limit', ...
        'LabelHorizontalAlignment','left');

    yline(-commandVelocityLimit(jointIndex),'r--', ...
        'Lower velocity limit', ...
        'LabelHorizontalAlignment','left');

    yline(0,'k:');

    grid on;
    xlabel('Time (s)');
    ylabel(sprintf('dq_%d (rad/s)',jointIndex));

    ylimPadding = 0.20*commandVelocityLimit(jointIndex);
    ylim([ ...
        -commandVelocityLimit(jointIndex)-ylimPadding, ...
         commandVelocityLimit(jointIndex)+ylimPadding]);

    if maximumVelocityUsagePercent(jointIndex) > 100
        titleColor = [0.85 0.1 0.1];
        stateText = 'VELOCITY LIMIT EXCEEDED';
    else
        titleColor = [0.1 0.5 0.1];
        stateText = 'WITHIN LIMIT';
    end

    title(ax,sprintf( ...
        ['Joint %d | %s | peak |dq| %.4f rad/s | ' ...
         'max usage %.1f%%'], ...
        jointIndex, ...
        stateText, ...
        maximumVelocityAbs(jointIndex), ...
        maximumVelocityUsagePercent(jointIndex)), ...
        'Color',titleColor);

    if jointIndex == 1
        legend( ...
            {'dq command','Upper limit','Lower limit','Zero'}, ...
            'Location','best');
    end
end

axSummary2 = nexttile(layout2,8);
axis(axSummary2,'off');

summaryLines2 = strings(10,1);
summaryLines2(1) = "Command velocity summary";
summaryLines2(2) = "---------------------------------------------";
summaryLines2(3) = sprintf("maxVelocityScale = %.3f",maxVelocityScale);

for jointIndex = 1:7
    summaryLines2(jointIndex+3) = sprintf( ...
        ['J%d: final=% .4f rad/s | peak=%.4f rad/s | ' ...
         'max usage=%.1f%%'], ...
        jointIndex, ...
        finalVelocity(jointIndex), ...
        maximumVelocityAbs(jointIndex), ...
        maximumVelocityUsagePercent(jointIndex));
end

text(axSummary2,0.02,0.98,summaryLines2, ...
    'Units','normalized', ...
    'VerticalAlignment','top', ...
    'FontName','Consolas', ...
    'FontSize',10);

%% =========================================================
%% 9. Figure 3：命令加速度
%% =========================================================

fig3 = figure( ...
    'Name','FR3 Command Acceleration Monitor', ...
    'Position',[120 100 1500 900]);

layout3 = tiledlayout(4,2, ...
    'TileSpacing','compact', ...
    'Padding','compact');

title(layout3, ...
    'FR3 Command Acceleration and Per-Joint Acceleration Limits');

for jointIndex = 1:7
    ax = nexttile(layout3,jointIndex);

    plot(timeA,aData(:,jointIndex), ...
        'LineWidth',1.25);
    hold on;

    yline(commandAccelerationLimit(jointIndex),'r--', ...
        'Upper acceleration limit', ...
        'LabelHorizontalAlignment','left');

    yline(-commandAccelerationLimit(jointIndex),'r--', ...
        'Lower acceleration limit', ...
        'LabelHorizontalAlignment','left');

    yline(0,'k:');

    grid on;
    xlabel('Time (s)');
    ylabel(sprintf('ddq_%d (rad/s^2)',jointIndex));

    ylimPadding = 0.25*commandAccelerationLimit(jointIndex);
    ylim([ ...
        -commandAccelerationLimit(jointIndex)-ylimPadding, ...
         commandAccelerationLimit(jointIndex)+ylimPadding]);

    if maximumAccelerationUsagePercent(jointIndex) > 100
        titleColor = [0.85 0.1 0.1];
        stateText = 'ACCEL LIMIT EXCEEDED';
    else
        titleColor = [0.1 0.5 0.1];
        stateText = 'WITHIN LIMIT';
    end

    title(ax,sprintf( ...
        ['Joint %d | %s | peak |ddq| %.4f rad/s^2 | ' ...
         'max usage %.1f%%'], ...
        jointIndex, ...
        stateText, ...
        maximumAccelerationAbs(jointIndex), ...
        maximumAccelerationUsagePercent(jointIndex)), ...
        'Color',titleColor);

    if jointIndex == 1
        legend( ...
            {'ddq command','Upper limit','Lower limit','Zero'}, ...
            'Location','best');
    end
end

axSummary3 = nexttile(layout3,8);
axis(axSummary3,'off');

summaryLines3 = strings(10,1);
summaryLines3(1) = "Command acceleration summary";
summaryLines3(2) = "---------------------------------------------";
summaryLines3(3) = "Per-joint acceleration limits (rad/s^2):";

for jointIndex = 1:7
    summaryLines3(jointIndex+3) = sprintf( ...
        ['J%d: limit=%.3f | final=% .4f rad/s^2 | peak=%.4f | ' ...
         'max usage=%.1f%%'], ...
        jointIndex, ...
        commandAccelerationLimit(jointIndex), ...
        finalAcceleration(jointIndex), ...
        maximumAccelerationAbs(jointIndex), ...
        maximumAccelerationUsagePercent(jointIndex));
end

text(axSummary3,0.02,0.98,summaryLines3, ...
    'Units','normalized', ...
    'VerticalAlignment','top', ...
    'FontName','Consolas', ...
    'FontSize',10);

%% =========================================================
%% 10. 保存图像和指标
%% =========================================================

timeStamp = char(datetime('now','Format','yyyyMMdd_HHmmss'));
outputFolder = fullfile(pwd,'test_results','joint_command_monitor');

if ~exist(outputFolder,'dir')
    mkdir(outputFolder);
end

fileBase = fullfile( ...
    outputFolder, ...
    ['fr3_joint_command_monitor_',timeStamp]);

exportgraphics(fig1,[fileBase,'_joint_position.png'],'Resolution',200);
savefig(fig1,[fileBase,'_joint_position.fig']);

exportgraphics(fig2,[fileBase,'_command_velocity.png'],'Resolution',200);
savefig(fig2,[fileBase,'_command_velocity.fig']);

exportgraphics(fig3,[fileBase,'_command_acceleration.png'],'Resolution',200);
savefig(fig3,[fileBase,'_command_acceleration.fig']);

jointPositionMetrics = table( ...
    (1:7).', ...
    qMin, ...
    qMax, ...
    qCenterLower, ...
    qCenterUpper, ...
    finalPosition.', ...
    minimumMargin.', ...
    maximumUsagePercent.', ...
    'VariableNames',{ ...
        'Joint', ...
        'LowerLimitRad', ...
        'UpperLimitRad', ...
        'Center60LowerRad', ...
        'Center60UpperRad', ...
        'FinalPositionRad', ...
        'MinimumMarginRad', ...
        'MaximumLimitUsagePercent'});

commandVelocityMetrics = table( ...
    (1:7).', ...
    fr3MaxJointVelocitiesOfficial, ...
    repmat(maxVelocityScale,7,1), ...
    commandVelocityLimit, ...
    finalVelocity.', ...
    maximumVelocityAbs.', ...
    maximumVelocityUsagePercent.', ...
    'VariableNames',{ ...
        'Joint', ...
        'OfficialVelocityLimitRadPerSec', ...
        'VelocityScale', ...
        'AppliedVelocityLimitRadPerSec', ...
        'FinalVelocityRadPerSec', ...
        'PeakAbsVelocityRadPerSec', ...
        'MaximumVelocityUsagePercent'});

commandAccelerationMetrics = table( ...
    (1:7).', ...
    commandAccelerationLimit, ...
    finalAcceleration.', ...
    maximumAccelerationAbs.', ...
    maximumAccelerationUsagePercent.', ...
    'VariableNames',{ ...
        'Joint', ...
        'AccelerationLimitRadPerSec2', ...
        'FinalAccelerationRadPerSec2', ...
        'PeakAbsAccelerationRadPerSec2', ...
        'MaximumAccelerationUsagePercent'});

writetable( ...
    jointPositionMetrics, ...
    [fileBase,'_joint_position_metrics.csv']);

writetable( ...
    commandVelocityMetrics, ...
    [fileBase,'_command_velocity_metrics.csv']);

writetable( ...
    commandAccelerationMetrics, ...
    [fileBase,'_command_acceleration_metrics.csv']);

fprintf('\n测试图已保存：\n');
fprintf('%s_joint_position.png\n',fileBase);
fprintf('%s_command_velocity.png\n',fileBase);
fprintf('%s_command_acceleration.png\n',fileBase);

fprintf('\n指标CSV已保存：\n');
fprintf('%s_joint_position_metrics.csv\n',fileBase);
fprintf('%s_command_velocity_metrics.csv\n',fileBase);
fprintf('%s_command_acceleration_metrics.csv\n\n',fileBase);

%% =========================================================
%% 局部函数
%% =========================================================

function signal = findLoggedSignal(signalName)

signal = [];

candidateOutputNames = {'out','simOut'};

for outputIndex = 1:numel(candidateOutputNames)
    outputName = candidateOutputNames{outputIndex};

    if evalin('base',sprintf('exist(''%s'',''var'')',outputName))
        simulationOutput = evalin('base',outputName);

        if isa(simulationOutput,'Simulink.SimulationOutput')
            outputNames = simulationOutput.who;

            if any(strcmp(outputNames,signalName))
                signal = simulationOutput.get(signalName);
                return
            end

            if any(strcmp(outputNames,'logsout'))
                logsout = simulationOutput.get('logsout');

                if isa(logsout,'Simulink.SimulationData.Dataset')
                    try
                        element = logsout.getElement(signalName);
                        signal = element.Values;
                        return
                    catch
                    end
                end
            end
        end
    end
end

if evalin('base',sprintf('exist(''%s'',''var'')',signalName))
    signal = evalin('base',signalName);
end

end

function [time,data] = signalToTimeData(signal)

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
        signal.Properties.RowTimes ...
        - signal.Properties.RowTimes(1));
    data = signal.Variables;

else
    error('不支持的日志格式：%s',class(signal));
end

data = squeeze(data);

if isvector(data)
    data = data(:);

elseif size(data,1) == 7 && ...
        size(data,2) == numel(time)
    data = data.';

elseif size(data,1) ~= numel(time) && ...
        size(data,2) == numel(time)
    data = data.';

elseif size(data,1) ~= numel(time)
    data = reshape(data,numel(time),[]);
end

data = double(data);

end