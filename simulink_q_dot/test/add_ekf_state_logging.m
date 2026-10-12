%% add_ekf_state_logging.m
% 给模型补上EKF估计的日志（ekf_state、ekf_covariance）。
%
% 背景：
% 模型通过根层Goto → Diagnostics Outputs里的To Workspace记录日志，
% 目前只记录了v_target_hat（KF速度）、image_error、target_data等，
% 没有记录EKF的状态本身，所以save_single_camera_test_results.m
% 的图3（四状态）、图3B（目标位移）、图4（标准差）都会缺失。
%
% 用法（模型已打开或当前目录在模型所在文件夹）：
%   add_ekf_state_logging
%   add_ekf_state_logging('single_camera_q_dot')   % 指定模型名
%
% 脚本做的事：
% 1. 在根层为KF子系统的ekf_state、ekf_covariance输出各加一个Goto；
% 2. 在Diagnostics Outputs里加对应的From + To Workspace，
%    变量名分别为ekf_state、ekf_covariance（与测试脚本的读取名一致）；
% 3. 保存模型。
%
% 重复运行是安全的：已存在的块会跳过。

function add_ekf_state_logging(modelName)

if nargin < 1 || isempty(modelName)
    modelName = 'single_camera_q_dot';
end

alreadyLoaded = bdIsLoaded(modelName);

if ~alreadyLoaded
    load_system(modelName);
    cleanupObject = onCleanup(@() close_system(modelName,0)); %#ok<NASGU>
end

kfBlock = [modelName '/Planar Target KF and Task Command'];
dbgBlock = [modelName '/Diagnostics Outputs'];

assert(blockExists(kfBlock), ...
    '找不到KF子系统：%s', kfBlock);
assert(blockExists(dbgBlock), ...
    '找不到日志子系统：%s', dbgBlock);

% 每行：变量名 | Goto标签 | 根层Goto块名 | From块名 | KF子系统输出端口号
signals = { ...
    'ekf_state',      'DBG_QDOT_ekf_state', ...
    'DBG_Goto_ekf_state',      'DBG_From_ekf_state',      1; ...
    'ekf_covariance', 'DBG_QDOT_ekf_covariance', ...
    'DBG_Goto_ekf_covariance', 'DBG_From_ekf_covariance', 2};

for signalIndex = 1:size(signals,1)
    variableName = signals{signalIndex,1};
    gotoTag = signals{signalIndex,2};
    gotoName = signals{signalIndex,3};
    fromName = signals{signalIndex,4};
    portIndex = signals{signalIndex,5};

    % --- 根层：KF输出 → Goto ---
    if isempty(find_system(modelName,'SearchDepth',1, ...
            'BlockType','Goto','GotoTag',gotoTag))
        gotoPath = [modelName '/' gotoName];
        add_block('simulink/Signal Routing/Goto',gotoPath, ...
            'GotoTag',gotoTag, ...
            'TagVisibility','global', ...
            'ShowName','off', ...
            'Position',[1085, 1090+40*(signalIndex-1), ...
            1295, 1106+40*(signalIndex-1)]);

        kfPorts = get_param(kfBlock,'PortHandles');
        gotoPorts = get_param(gotoPath,'PortHandles');
        add_line(modelName,kfPorts.Outport(portIndex), ...
            gotoPorts.Inport(1),'autorouting','on');

        fprintf('已添加根层Goto：%s（tag %s）\n',gotoName,gotoTag);
    else
        fprintf('已存在，跳过Goto：%s\n',gotoName);
    end

    % --- Diagnostics Outputs：From → To Workspace ---
    existingBlocks = find_system(dbgBlock,'SearchDepth',1, ...
        'BlockType','ToWorkspace','VariableName',variableName);

    if isempty(existingBlocks)
        toWorkspacePath = [dbgBlock '/' variableName];
        fromPath = [dbgBlock '/' fromName];

        add_block('simulink/Signal Routing/From',fromPath, ...
            'GotoTag',gotoTag, ...
            'ShowName','off', ...
            'Position',[1140, 940+40*(signalIndex-1), ...
            1240, 964+40*(signalIndex-1)]);

        add_block('simulink/Sinks/To Workspace',toWorkspacePath, ...
            'VariableName',variableName, ...
            'SaveFormat','Timeseries', ...
            'MaxDataPoints','inf', ...
            'SampleTime','-1', ...
            'Position',[1340, 940+40*(signalIndex-1), ...
            1505, 962+40*(signalIndex-1)]);

        add_line(dbgBlock,[fromName '/1'],[variableName '/1'], ...
            'autorouting','on');

        fprintf('已添加To Workspace：%s（变量名 %s）\n', ...
            toWorkspacePath,variableName);
    else
        fprintf('已存在，跳过To Workspace：%s\n',variableName);
    end
end

save_system(modelName);

fprintf('模型已保存：%s\n',get_param(modelName,'FileName'));
fprintf(['现在重跑一次仿真，再运行save_single_camera_test_results，\n' ...
    '就会多出图3（Base系四状态）、图3B（目标位移）和图4（标准差），\n' ...
    '以及ekf_state_timeseries.csv。\n']);

end


function tf = blockExists(blockPath)

tf = true;

try
    get_param(blockPath,'Handle');
catch
    tf = false;
end

end
