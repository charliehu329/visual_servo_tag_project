function build_single_camera_xy_tracking_isnew()
% build_single_camera_xy_tracking_isnew
%
% 功能：
% 1. 复制现有 single_camera_xy_tracking*.slx，生成：
%       single_camera_xy_tracking_isnew.slx
% 2. 将 Target ROS2 Subscriber 的 Subscribe/IsNew 引出为第4个输出；
% 3. ROS模式使用真实 IsNew，离线模式每个采样周期视为新测量；
% 4. 给 Planar Target EKF 增加 measurement_is_new 输入；
% 5. EKF每个Ts执行预测，但仅在 measurement_is_new=true 时更新；
% 6. 没有新帧时继续使用预测状态和最近一次接受的图像误差控制，
%    不再把速度历史清零；
% 7. 增加 log_measurement_is_new，便于验证新帧脉冲。
%
% 使用：
%   1. 保存并关闭原模型；
%   2. 把本文件放到 single_camera_xy_tracking*.slx 同一目录；
%   3. 在MATLAB命令窗口运行：
%          build_single_camera_xy_tracking_isnew
%
% 输出：
%   single_camera_xy_tracking_isnew.slx
%
% 适配模型：
%   用户上传的 single_camera_xy_tracking(2).slx 结构。

clc;

%% 1. 找到源模型并创建副本

outputModelName = 'single_camera_xy_tracking_isnew';
outputModelFile = fullfile(pwd,[outputModelName,'.slx']);

sourceModelFile = locateSourceModel(outputModelFile);

fprintf('源模型：%s\n',sourceModelFile);
fprintf('输出模型：%s\n',outputModelFile);

saveAndCloseLoadedFile(sourceModelFile);

if bdIsLoaded(outputModelName)
    close_system(outputModelName,0);
end

copyfile(sourceModelFile,outputModelFile,'f');
load_system(outputModelName);

model = outputModelName;

try
    %% 2. Target ROS2 Subscriber：引出真实 IsNew

    targetSubscriber = [model '/Target ROS2 Subscriber'];
    subscribeBlock = [targetSubscriber '/Subscribe'];
    targetIsNewOut = [targetSubscriber '/target_is_new'];

    assertBlockExists(targetSubscriber);
    assertBlockExists(subscribeBlock);

    if ~blockExists(targetIsNewOut)
        add_block( ...
            'simulink/Ports & Subsystems/Out1', ...
            targetIsNewOut, ...
            'Port','4', ...
            'Position',[500 195 530 215]);
    else
        set_param(targetIsNewOut,'Port','4');
    end

    addLineIfMissing( ...
        targetSubscriber, ...
        'Subscribe/1', ...
        'target_is_new/1');

    %% 3. 顶层：ROS IsNew / 离线 IsNew 选择

    offlineIsNew = [model '/Offline Target IsNew'];
    isNewSwitch = [model '/Target IsNew Source'];

    if ~blockExists(offlineIsNew)
        add_block( ...
            'simulink/Sources/Constant', ...
            offlineIsNew, ...
            'Value','1', ...
            'OutDataTypeStr','boolean', ...
            'Position',[165 1080 215 1110]);
    end

    if ~blockExists(isNewSwitch)
        add_block( ...
            'simulink/Signal Routing/Switch', ...
            isNewSwitch, ...
            'Criteria','u2 ~= 0', ...
            'Threshold','0.5', ...
            'Position',[275 1075 320 1115]);
    end

    % u1：ROS真实IsNew；u2：USE_ROS；u3：离线恒true。
    addLineIfMissing( ...
        model, ...
        'Target ROS2 Subscriber/4', ...
        'Target IsNew Source/1');

    addLineIfMissing( ...
        model, ...
        'USE_ROS/1', ...
        'Target IsNew Source/2');

    addLineIfMissing( ...
        model, ...
        'Offline Target IsNew/1', ...
        'Target IsNew Source/3');

    %% 4. Planar Target EKF：增加第4个输入

    ekfSubsystem = [model '/Planar Target EKF'];
    coreBlock = [ekfSubsystem '/EKF and Controller Core'];
    isNewInport = [ekfSubsystem '/measurement_is_new'];

    assertBlockExists(ekfSubsystem);
    assertBlockExists(coreBlock);

    if ~blockExists(isNewInport)
        add_block( ...
            'simulink/Ports & Subsystems/In1', ...
            isNewInport, ...
            'Port','4', ...
            'Position',[20 200 50 220]);
    else
        set_param(isNewInport,'Port','4');
    end

    % 修改MATLAB Function签名前，先删除参数常量到旧4~7端口的连线。
    deleteLineIfPresent( ...
        ekfSubsystem, ...
        'Controller Parameters/1', ...
        'EKF and Controller Core/4');

    deleteLineIfPresent( ...
        ekfSubsystem, ...
        'P0/1', ...
        'EKF and Controller Core/5');

    deleteLineIfPresent( ...
        ekfSubsystem, ...
        'Q EKF/1', ...
        'EKF and Controller Core/6');

    deleteLineIfPresent( ...
        ekfSubsystem, ...
        'R EKF/1', ...
        'EKF and Controller Core/7');

    %% 5. 更新EKF MATLAB Function代码

    chart = findEmbeddedMatlabChart(coreBlock);
    chart.Script = updatedCoreCode();

    % 强制Stateflow刷新MATLAB Function端口。
    drawnow;

    % 新端口顺序：
    % 1 z_meas
    % 2 e
    % 3 safe_valid
    % 4 measurement_is_new
    % 5 parameters
    % 6 P0
    % 7 Q_ekf
    % 8 R_ekf
    addLineIfMissing( ...
        ekfSubsystem, ...
        'measurement_is_new/1', ...
        'EKF and Controller Core/4');

    addLineIfMissing( ...
        ekfSubsystem, ...
        'Controller Parameters/1', ...
        'EKF and Controller Core/5');

    addLineIfMissing( ...
        ekfSubsystem, ...
        'P0/1', ...
        'EKF and Controller Core/6');

    addLineIfMissing( ...
        ekfSubsystem, ...
        'Q EKF/1', ...
        'EKF and Controller Core/7');

    addLineIfMissing( ...
        ekfSubsystem, ...
        'R EKF/1', ...
        'EKF and Controller Core/8');

    %% 6. 顶层：IsNew接入EKF第4输入

    addLineIfMissing( ...
        model, ...
        'Target IsNew Source/1', ...
        'Planar Target EKF/4');

    %% 7. Logging：增加 measurement_is_new 日志

    loggingSubsystem = [model '/Logging'];
    logIsNewInport = [loggingSubsystem '/measurement_is_new'];
    logIsNewBlock = [loggingSubsystem '/log_measurement_is_new'];

    assertBlockExists(loggingSubsystem);

    if ~blockExists(logIsNewInport)
        add_block( ...
            'simulink/Ports & Subsystems/In1', ...
            logIsNewInport, ...
            'Port','22', ...
            'Position',[20 616 50 634]);
    else
        set_param(logIsNewInport,'Port','22');
    end

    if ~blockExists(logIsNewBlock)
        add_block( ...
            'simulink/Sinks/To Workspace', ...
            logIsNewBlock, ...
            'VariableName','log_measurement_is_new', ...
            'SaveFormat','Structure With Time', ...
            'MaxDataPoints','inf', ...
            'Position',[110 613 270 637]);
    end

    addLineIfMissing( ...
        loggingSubsystem, ...
        'measurement_is_new/1', ...
        'log_measurement_is_new/1');

    addLineIfMissing( ...
        model, ...
        'Target IsNew Source/1', ...
        'Logging/22');

    %% 8. 更新模型并保存

    set_param(model,'SimulationCommand','update');
    save_system(model,outputModelFile);

    fprintf('\n构建完成：%s\n',outputModelFile);
    fprintf('请打开并检查：\n');
    fprintf('  1. Target ROS2 Subscriber 第4输出 target_is_new\n');
    fprintf('  2. Planar Target EKF 第4输入 measurement_is_new\n');
    fprintf('  3. Workspace日志 log_measurement_is_new\n');
    fprintf('\n正常现象：\n');
    fprintf('  safe_valid 在目标持续有效时保持1；\n');
    fprintf('  measurement_is_new 只在ROS收到真实新消息时产生脉冲；\n');
    fprintf('  measurement_accepted 只在新帧通过EKF门限时产生脉冲；\n');
    fprintf('  controller_ok 在两帧之间仍应保持1。\n');

    open_system(model);

catch ME
    fprintf(2,'\n构建失败：%s\n',ME.message);

    if bdIsLoaded(model)
        close_system(model,0);
    end

    rethrow(ME);
end

end


%% ============================================================
%  局部函数
% =============================================================

function sourceModelFile = locateSourceModel(outputModelFile)

preferredNames = { ...
    'single_camera_xy_tracking.slx', ...
    'single_camera_xy_tracking(2).slx', ...
    'single_camera_xy_tracking(1).slx'};

sourceModelFile = '';

for index = 1:numel(preferredNames)
    candidate = fullfile(pwd,preferredNames{index});

    if exist(candidate,'file') == 2 && ...
            ~strcmp(candidate,outputModelFile)
        sourceModelFile = candidate;
        return;
    end
end

files = dir(fullfile(pwd,'single_camera_xy_tracking*.slx'));

if isempty(files)
    error( ...
        ['当前目录没有找到 single_camera_xy_tracking*.slx。' ...
         '请把build文件放到模型同一目录。']);
end

fullPaths = arrayfun( ...
    @(item) fullfile(item.folder,item.name), ...
    files, ...
    'UniformOutput',false);

keep = ~strcmp(fullPaths,outputModelFile);
files = files(keep);

if isempty(files)
    error('只找到了输出模型，没有找到可用于复制的源模型。');
end

[~,latestIndex] = max([files.datenum]);
sourceModelFile = fullfile( ...
    files(latestIndex).folder, ...
    files(latestIndex).name);

end


function saveAndCloseLoadedFile(fileName)

loadedModels = find_system( ...
    'SearchDepth',0, ...
    'Type','block_diagram');

canonicalInput = canonicalPath(fileName);

for index = 1:numel(loadedModels)
    model = loadedModels{index};

    try
        loadedFile = get_param(model,'FileName');
    catch
        continue;
    end

    if isempty(loadedFile)
        continue;
    end

    if strcmp(canonicalPath(loadedFile),canonicalInput)
        if strcmp(get_param(model,'Dirty'),'on')
            save_system(model);
        end

        close_system(model,0);
        return;
    end
end

end


function pathOut = canonicalPath(pathIn)

pathOut = char(java.io.File(pathIn).getCanonicalPath());

end


function tf = blockExists(blockPath)

tf = getSimulinkBlockHandle(blockPath) ~= -1;

end


function assertBlockExists(blockPath)

if ~blockExists(blockPath)
    error('模型中没有找到模块：%s',blockPath);
end

end


function addLineIfMissing(systemPath,sourcePort,destinationPort)

sourceBlockName = extractBefore(sourcePort,'/');
destinationBlockName = extractBefore(destinationPort,'/');

sourceBlock = [systemPath '/' char(sourceBlockName)];
destinationBlock = [systemPath '/' char(destinationBlockName)];

assertBlockExists(sourceBlock);
assertBlockExists(destinationBlock);

destinationHandles = get_param(destinationBlock,'PortHandles');
destinationIndex = str2double(extractAfter(destinationPort,'/'));

if destinationIndex > numel(destinationHandles.Inport)
    error( ...
        '目标端口不存在：%s/%s', ...
        systemPath, ...
        destinationPort);
end

destinationHandle = destinationHandles.Inport(destinationIndex);
existingLine = get_param(destinationHandle,'Line');

if existingLine ~= -1
    sourceHandle = get_param(existingLine,'SrcPortHandle');
    expectedSourceHandles = get_param(sourceBlock,'PortHandles');
    sourceIndex = str2double(extractAfter(sourcePort,'/'));
    expectedSourceHandle = expectedSourceHandles.Outport(sourceIndex);

    if sourceHandle == expectedSourceHandle
        return;
    end

    delete_line(existingLine);
end

add_line( ...
    systemPath, ...
    sourcePort, ...
    destinationPort, ...
    'autorouting','on');

end


function deleteLineIfPresent(systemPath,sourcePort,destinationPort)

try
    delete_line(systemPath,sourcePort,destinationPort);
catch
    % 允许脚本重复运行，目标连线不存在时不报错。
end

end


function chart = findEmbeddedMatlabChart(blockPath)

root = sfroot;
charts = root.find('-isa','Stateflow.EMChart');

chart = [];

for index = 1:numel(charts)
    if strcmp(charts(index).Path,blockPath)
        chart = charts(index);
        break;
    end
end

if isempty(chart)
    error('没有找到MATLAB Function对应的Stateflow.EMChart：%s',blockPath);
end

end


function code = updatedCoreCode()

lines = [
"function [ekf_state,ekf_covariance,v_target_hat,v_p,v_ff,v_adapt,v_norm_limited,v_issued,camera_velocity,saturation_active,acceleration_active,ekf_initialized,innovation_nis,measurement_accepted,controller_ok] = fcn(z_meas,e,safe_valid,measurement_is_new,parameters,P0,Q_ekf,R_ekf)"
"%#codegen"
"% 单目XY目标EKF与控制器。"
"% 每个Ts执行预测，仅在measurement_is_new=true时执行测量更新。"
"% 两张图像之间继续使用预测状态和最近一次接受的图像误差控制。"
""
"persistent x_hat P d_hat v_previous p_camera_hat invalid_time initialized e_hold"
""
"if isempty(initialized)"
"    x_hat=zeros(6,1);"
"    P=P0;"
"    d_hat=zeros(2,1);"
"    v_previous=zeros(2,1);"
"    p_camera_hat=zeros(2,1);"
"    invalid_time=0.0;"
"    initialized=false;"
"    e_hold=zeros(2,1);"
"end"
""
"Ts=parameters(1);"
"Z_hat=parameters(2);"
"Kpx=parameters(3);"
"Kpy=parameters(4);"
"k_ff=parameters(5);"
"gamma=parameters(6);"
"sigma=parameters(7);"
"adapt_max=parameters(8);"
"v_max=parameters(9);"
"a_max=parameters(10);"
"gate=parameters(11);"
"enable_p=parameters(12)>0.5;"
"enable_ff=parameters(13)>0.5;"
"enable_adapt=parameters(14)>0.5;"
"controller_enable=parameters(15)>0.5;"
"reset_timeout=parameters(16);"
""
"measurement_is_new=measurement_is_new>0.5;"
""
"ekf_state=zeros(6,1);"
"ekf_covariance=P0;"
"v_target_hat=zeros(2,1);"
"v_p=zeros(2,1);"
"v_ff=zeros(2,1);"
"v_adapt=zeros(2,1);"
"v_norm_limited=zeros(2,1);"
"v_issued=zeros(2,1);"
"camera_velocity=zeros(6,1);"
"saturation_active=false;"
"acceleration_active=false;"
"ekf_initialized=initialized;"
"innovation_nis=0.0;"
"measurement_accepted=false;"
"controller_ok=false;"
""
"F=[1 0 Ts 0 0.5*Ts*Ts 0;"
"   0 1 0 Ts 0 0.5*Ts*Ts;"
"   0 0 1 0 Ts 0;"
"   0 0 0 1 0 Ts;"
"   0 0 0 0 1 0;"
"   0 0 0 0 0 1];"
""
"H=[1 0 0 0 0 0;"
"   0 1 0 0 0 0];"
""
"I6=eye(6);"
""
"% 使用上一周期已签发速度估计相机累计XY位移。"
"p_camera_hat=p_camera_hat+Ts*v_previous;"
""
"parameter_ok=all(isfinite(parameters(:))) && ..."
"    Ts>0 && Z_hat>0 && v_max>0 && a_max>0 && ..."
"    reset_timeout>=Ts;"
""
"input_ok=safe_valid && controller_enable && parameter_ok && ..."
"    all(isfinite(z_meas(:))) && all(isfinite(e(:)));"
""
"if ~input_ok"
"    % 输入无效时仍做预测，但输出速度为零。"
"    if initialized"
"        x_hat=F*x_hat;"
"        P=F*P*F'+Q_ekf;"
"        P=0.5*(P+P');"
"    end"
""
"    invalid_time=invalid_time+Ts;"
"    d_hat=zeros(2,1);"
"    v_previous=zeros(2,1);"
"    e_hold=zeros(2,1);"
""
"    state_ok=all(isfinite(x_hat(:))) && ..."
"        all(isfinite(P(:))) && all(diag(P)>=0);"
""
"    if invalid_time>=reset_timeout || ~state_ok"
"        x_hat=zeros(6,1);"
"        P=P0;"
"        p_camera_hat=zeros(2,1);"
"        initialized=false;"
"    end"
""
"    ekf_state=x_hat;"
"    ekf_covariance=P;"
"    v_target_hat=x_hat(3:4);"
"    ekf_initialized=initialized;"
"    return"
"end"
""
"invalid_time=0.0;"
""
"if ~initialized"
"    % 只有真实新帧才能初始化。"
"    if ~measurement_is_new"
"        ekf_state=x_hat;"
"        ekf_covariance=P;"
"        ekf_initialized=false;"
"        return"
"    end"
""
"    z_ekf=z_meas+p_camera_hat;"
"    x_hat=[z_ekf(1);z_ekf(2);0;0;0;0];"
"    P=P0;"
"    initialized=true;"
"    measurement_accepted=true;"
"    e_hold=e;"
"else"
"    % 每个控制周期都预测。"
"    x_pred=F*x_hat;"
"    P_pred=F*P*F'+Q_ekf;"
"    P_pred=0.5*(P_pred+P_pred');"
"    x_hat=x_pred;"
"    P=P_pred;"
""
"    % 只有真实新视觉帧才更新。"
"    if measurement_is_new"
"        z_ekf=z_meas+p_camera_hat;"
"        innovation=z_ekf-H*x_pred;"
"        S=H*P_pred*H'+R_ekf;"
""
"        if all(isfinite(S(:))) && rcond(S)>1e-12"
"            innovation_nis=innovation'*(S\innovation);"
"        else"
"            innovation_nis=inf;"
"        end"
""
"        if isfinite(innovation_nis) && innovation_nis<=gate"
"            K=(P_pred*H')/S;"
"            x_hat=x_pred+K*innovation;"
"            A=I6-K*H;"
"            P=A*P_pred*A'+K*R_ekf*K';"
"            P=0.5*(P+P');"
"            measurement_accepted=true;"
"            e_hold=e;"
"        end"
"    end"
"end"
""
"ekf_ok=all(isfinite(x_hat(:))) && ..."
"    all(isfinite(P(:))) && all(diag(P)>=0) && ..."
"    norm(P-P','fro')<1e-6;"
""
"if ~ekf_ok"
"    x_hat=zeros(6,1);"
"    P=P0;"
"    d_hat=zeros(2,1);"
"    v_previous=zeros(2,1);"
"    p_camera_hat=zeros(2,1);"
"    e_hold=zeros(2,1);"
"    initialized=false;"
"    return"
"end"
""
"ekf_state=x_hat;"
"ekf_covariance=P;"
"v_target_hat=x_hat(3:4);"
"ekf_initialized=initialized;"
""
"% 注意：没有新帧不等于控制无效。"
"% 使用最近一次通过门限的图像误差继续闭环控制。"
"e_control=e_hold;"
""
"if enable_p"
"    v_p=Z_hat*[Kpx*e_control(1);Kpy*e_control(2)];"
"end"
""
"if enable_ff"
"    v_ff=k_ff*v_target_hat;"
"end"
""
"if enable_adapt"
"    leak=max(0.0,1.0-sigma*Ts);"
"    d_hat=leak*d_hat+gamma*Z_hat*e_control*Ts;"
"    d_hat(1)=min(max(d_hat(1),-adapt_max),adapt_max);"
"    d_hat(2)=min(max(d_hat(2),-adapt_max),adapt_max);"
"else"
"    d_hat=zeros(2,1);"
"end"
""
"v_adapt=d_hat;"
"raw=v_p+v_ff+v_adapt;"
"raw_norm=norm(raw);"
"saturation_active=raw_norm>v_max;"
""
"if saturation_active"
"    v_norm_limited=raw*(v_max/raw_norm);"
"else"
"    v_norm_limited=raw;"
"end"
""
"dv=v_norm_limited-v_previous;"
"dv_norm=norm(dv);"
"dv_max=a_max*Ts;"
"acceleration_active=dv_norm>dv_max;"
""
"if acceleration_active"
"    v_issued=v_previous+dv*(dv_max/dv_norm);"
"else"
"    v_issued=v_norm_limited;"
"end"
""
"controller_ok=all(isfinite(v_p(:))) && ..."
"    all(isfinite(v_ff(:))) && ..."
"    all(isfinite(d_hat(:))) && ..."
"    all(isfinite(v_issued(:)));"
""
"if controller_ok"
"    v_previous=v_issued;"
"    camera_velocity=[v_issued;0;0;0;0];"
"else"
"    x_hat=zeros(6,1);"
"    P=P0;"
"    d_hat=zeros(2,1);"
"    v_previous=zeros(2,1);"
"    p_camera_hat=zeros(2,1);"
"    e_hold=zeros(2,1);"
"    initialized=false;"
"    ekf_state=x_hat;"
"    ekf_covariance=P;"
"    v_target_hat=zeros(2,1);"
"    v_adapt=zeros(2,1);"
"    v_norm_limited=zeros(2,1);"
"    v_issued=zeros(2,1);"
"end"
""
"ekf_initialized=initialized;"
"end"
];

code = char(join(lines,newline));

end
