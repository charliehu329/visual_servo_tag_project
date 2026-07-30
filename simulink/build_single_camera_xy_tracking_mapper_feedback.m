function build_single_camera_xy_tracking_mapper_feedback()
% build_single_camera_xy_tracking_mapper_feedback
%
% 作用：
% 1. 复制已经加入 measurement_is_new 的单目XY模型；
% 2. 订阅：
%      /velocity_mapper_node/target_joints_velocities
%    消息类型：
%      std_msgs/Float64MultiArray
% 3. 使用当前FR3关节角 q 和 mapper输出 qDotCommand 计算：
%      cameraTwist = J_camera(q) * qDotCommand
% 4. 将相机坐标系XY线速度接入EKF，用于替换原来的：
%      p_camera_hat = p_camera_hat + Ts*v_previous
% 5. 当反馈消息无效或超时时，自动回退到原来的 v_previous。
%
% 重要：
% /velocity_mapper_node/target_joints_velocities 是经过mapper限制后的
% "关节速度命令"，不是机器人传感器测得的真实关节速度。
% 它比Simulink上层的 v_issued 更接近执行值，但velocity_command_node
% 以及机器人底层仍可能继续限幅。
%
% 前置条件：
% - 源模型已经包含 measurement_is_new；
% - 当前目录位于工程 simulink 目录；
% - 工程中存在 velocity_servo_tag/config/urdf/fr3.urdf；
% - velocity_servo_tag.yaml 中包含 T_end_effector_camera。
%
% 使用：
%   build_single_camera_xy_tracking_mapper_feedback
%
% 输出：
%   single_camera_xy_tracking_mapper_feedback.slx
%
% 构建后新增日志：
%   log_mapper_joint_velocity
%   log_camera_velocity_feedback
%   log_camera_velocity_feedback_valid

clc;

%% 用户设置

mapperJointVelocityTopic = ...
    '/velocity_mapper_node/target_joints_velocities';

outputModelName = ...
    'single_camera_xy_tracking_mapper_feedback';

outputModelFile = ...
    fullfile(pwd,[outputModelName,'.slx']);

%% 找到源模型

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
    %% 检查源模型是否已经加入IsNew

    ekfSubsystem = [model '/Planar Target EKF'];
    coreBlock = [ekfSubsystem '/EKF and Controller Core'];
    isNewInport = [ekfSubsystem '/measurement_is_new'];

    assertBlockExists(ekfSubsystem);
    assertBlockExists(coreBlock);

    if ~blockExists(isNewInport)
        error([ ...
            '源模型没有 measurement_is_new。' ...
            '请先对 single_camera_xy_tracking_isnew.slx 运行本build。']);
    end

    %% 查找URDF和YAML，并读取运动学参数

    urdfPath = locateProjectFile('fr3.urdf');
    yamlPath = locateProjectFile('velocity_servo_tag.yaml');

    fprintf('URDF：%s\n',urdfPath);
    fprintf('YAML：%s\n',yamlPath);

    [ ...
        fr3OriginXYZ, ...
        fr3OriginRPY, ...
        fr3Axis, ...
        fr3Joint8OriginXYZ, ...
        fr3Joint8OriginRPY] = ...
        readFr3ChainFromUrdf(urdfPath);

    TLink8Camera = ...
        readHandEyeFromYaml(yamlPath);

    fprintf('已读取FR3关节链与手眼标定矩阵。\n');

    %% 创建mapper关节速度订阅子系统

    sourceSubscriber = ...
        [model '/Target ROS2 Subscriber'];

    mapperSubscriber = ...
        [model '/Mapper Joint Velocity ROS2 Subscriber'];

    assertBlockExists(sourceSubscriber);

    if blockExists(mapperSubscriber)
        delete_block(mapperSubscriber);
    end

    add_block( ...
        sourceSubscriber, ...
        mapperSubscriber, ...
        'Position',[25 1125 285 1255]);

    subscribeBlock = ...
        [mapperSubscriber '/Subscribe'];

    set_param( ...
        subscribeBlock, ...
        'topic',mapperJointVelocityTopic, ...
        'messageType','std_msgs/Float64MultiArray', ...
        'sampleTime','Ts');

    unpackBlock = ...
        [mapperSubscriber '/Unpack'];

    unpackChart = ...
        findEmbeddedMatlabChart(unpackBlock);

    unpackChart.Script = mapperJointVelocityUnpackCode();

    drawnow;

    renameBlockIfPresent( ...
        [mapperSubscriber '/target_data'], ...
        'qdot_command');

    renameBlockIfPresent( ...
        [mapperSubscriber '/target_count'], ...
        'qdot_count');

    renameBlockIfPresent( ...
        [mapperSubscriber '/target_age'], ...
        'qdot_age');

    set_param( ...
        [mapperSubscriber '/qdot_command'], ...
        'Port','1');

    set_param( ...
        [mapperSubscriber '/qdot_count'], ...
        'Port','2');

    set_param( ...
        [mapperSubscriber '/qdot_age'], ...
        'Port','3');

    %% 创建相机速度反馈估计子系统

    estimatorSubsystem = ...
        [model '/Mapper Camera Velocity Feedback'];

    if blockExists(estimatorSubsystem)
        delete_block(estimatorSubsystem);
    end

    add_block( ...
        'simulink/Ports & Subsystems/Subsystem', ...
        estimatorSubsystem, ...
        'Position',[390 1110 730 1305], ...
        'BackgroundColor','white');

    deleteBlockIfPresent([estimatorSubsystem '/In1']);
    deleteBlockIfPresent([estimatorSubsystem '/Out1']);

    % 输入
    addInport( ...
        estimatorSubsystem, ...
        'q', ...
        1, ...
        '[7 1]', ...
        [25 35 55 55]);

    addInport( ...
        estimatorSubsystem, ...
        'q_count', ...
        2, ...
        '1', ...
        [25 70 55 90]);

    addInport( ...
        estimatorSubsystem, ...
        'q_age', ...
        3, ...
        '1', ...
        [25 105 55 125]);

    addInport( ...
        estimatorSubsystem, ...
        'qdot_command', ...
        4, ...
        '[7 1]', ...
        [25 145 55 165]);

    addInport( ...
        estimatorSubsystem, ...
        'qdot_count', ...
        5, ...
        '1', ...
        [25 180 55 200]);

    addInport( ...
        estimatorSubsystem, ...
        'qdot_age', ...
        6, ...
        '1', ...
        [25 215 55 235]);

    % 固定运动学常量直接嵌入模型。
    constantDefinitions = {
        'FR3 Origin XYZ', ...
            matrixExpression(fr3OriginXYZ), ...
            [85 275 205 305]

        'FR3 Origin RPY', ...
            matrixExpression(fr3OriginRPY), ...
            [85 320 205 350]

        'FR3 Joint Axis', ...
            matrixExpression(fr3Axis), ...
            [85 365 205 395]

        'FR3 Joint8 Origin XYZ', ...
            matrixExpression(fr3Joint8OriginXYZ), ...
            [85 410 205 440]

        'FR3 Joint8 Origin RPY', ...
            matrixExpression(fr3Joint8OriginRPY), ...
            [85 455 205 485]

        'Link8 To Camera', ...
            matrixExpression(TLink8Camera), ...
            [85 500 205 530]

        'Feedback Timeout', ...
            'joint_state_timeout_sec', ...
            [85 545 205 575]
    };

    for constantIndex = 1:size(constantDefinitions,1)
        add_block( ...
            'simulink/Sources/Constant', ...
            [estimatorSubsystem '/' ...
                constantDefinitions{constantIndex,1}], ...
            'Value', ...
                constantDefinitions{constantIndex,2}, ...
            'OutDataTypeStr','double', ...
            'SampleTime','-1', ...
            'Position', ...
                constantDefinitions{constantIndex,3});
    end

    estimatorFunction = ...
        [estimatorSubsystem '/Camera Velocity From Joint Command'];

    add_block( ...
        'simulink/User-Defined Functions/MATLAB Function', ...
        estimatorFunction, ...
        'Position',[265 30 565 575]);

    estimatorChart = ...
        findEmbeddedMatlabChart(estimatorFunction);

    estimatorChart.Script = ...
        cameraVelocityEstimatorCode();

    drawnow;

    % 输出
    addOutport( ...
        estimatorSubsystem, ...
        'camera_velocity_feedback_xy', ...
        1, ...
        '[2 1]', ...
        [650 105 680 125]);

    addOutport( ...
        estimatorSubsystem, ...
        'feedback_valid', ...
        2, ...
        '1', ...
        [650 165 680 185]);

    % 输入连线
    estimatorInputNames = { ...
        'q', ...
        'q_count', ...
        'q_age', ...
        'qdot_command', ...
        'qdot_count', ...
        'qdot_age'};

    for inputIndex = 1:numel(estimatorInputNames)
        addLineIfMissing( ...
            estimatorSubsystem, ...
            [estimatorInputNames{inputIndex} '/1'], ...
            sprintf( ...
                'Camera Velocity From Joint Command/%d', ...
                inputIndex));
    end

    % 常量连线从第7个函数输入开始。
    for constantIndex = 1:size(constantDefinitions,1)
        addLineIfMissing( ...
            estimatorSubsystem, ...
            [constantDefinitions{constantIndex,1} '/1'], ...
            sprintf( ...
                'Camera Velocity From Joint Command/%d', ...
                constantIndex + 6));
    end

    addLineIfMissing( ...
        estimatorSubsystem, ...
        'Camera Velocity From Joint Command/1', ...
        'camera_velocity_feedback_xy/1');

    addLineIfMissing( ...
        estimatorSubsystem, ...
        'Camera Velocity From Joint Command/2', ...
        'feedback_valid/1');

    %% 顶层连接订阅数据和当前关节状态

    addLineIfMissing( ...
        model, ...
        'Joint Position Source/1', ...
        'Mapper Camera Velocity Feedback/1');

    addLineIfMissing( ...
        model, ...
        'Joint Count Source/1', ...
        'Mapper Camera Velocity Feedback/2');

    addLineIfMissing( ...
        model, ...
        'Joint Age Source/1', ...
        'Mapper Camera Velocity Feedback/3');

    addLineIfMissing( ...
        model, ...
        'Mapper Joint Velocity ROS2 Subscriber/1', ...
        'Mapper Camera Velocity Feedback/4');

    addLineIfMissing( ...
        model, ...
        'Mapper Joint Velocity ROS2 Subscriber/2', ...
        'Mapper Camera Velocity Feedback/5');

    addLineIfMissing( ...
        model, ...
        'Mapper Joint Velocity ROS2 Subscriber/3', ...
        'Mapper Camera Velocity Feedback/6');

    %% ROS/离线反馈源切换

    velocitySourceSwitch = ...
        [model '/Camera Velocity Feedback Source'];

    feedbackValidSwitch = ...
        [model '/Camera Velocity Feedback Valid Source'];

    offlineValid = ...
        [model '/Offline Camera Velocity Feedback Valid'];

    if ~blockExists(velocitySourceSwitch)
        add_block( ...
            'simulink/Signal Routing/Switch', ...
            velocitySourceSwitch, ...
            'Criteria','u2 ~= 0', ...
            'Threshold','0.5', ...
            'Position',[785 1135 835 1175]);
    end

    if ~blockExists(feedbackValidSwitch)
        add_block( ...
            'simulink/Signal Routing/Switch', ...
            feedbackValidSwitch, ...
            'Criteria','u2 ~= 0', ...
            'Threshold','0.5', ...
            'Position',[785 1200 835 1240]);
    end

    if ~blockExists(offlineValid)
        add_block( ...
            'simulink/Sources/Constant', ...
            offlineValid, ...
            'Value','true', ...
            'OutDataTypeStr','boolean', ...
            'Position',[640 1260 700 1290]);
    end

    % ROS模式：使用mapper反馈。
    addLineIfMissing( ...
        model, ...
        'Mapper Camera Velocity Feedback/1', ...
        'Camera Velocity Feedback Source/1');

    addLineIfMissing( ...
        model, ...
        'Mapper Camera Velocity Feedback/2', ...
        'Camera Velocity Feedback Valid Source/1');

    % u2选择信号。
    addLineIfMissing( ...
        model, ...
        'USE_ROS/1', ...
        'Camera Velocity Feedback Source/2');

    addLineIfMissing( ...
        model, ...
        'USE_ROS/1', ...
        'Camera Velocity Feedback Valid Source/2');

    % 离线模式：使用离线plant实际采用的XY速度。
    addLineIfMissing( ...
        model, ...
        'Plant Camera Velocity Delay/1', ...
        'Camera Velocity Feedback Source/3');

    addLineIfMissing( ...
        model, ...
        'Offline Camera Velocity Feedback Valid/1', ...
        'Camera Velocity Feedback Valid Source/3');

    %% 给Planar Target EKF增加相机速度反馈输入

    velocityFeedbackInport = ...
        [ekfSubsystem '/camera_velocity_feedback_xy'];

    feedbackValidInport = ...
        [ekfSubsystem '/camera_velocity_feedback_valid'];

    if ~blockExists(velocityFeedbackInport)
        add_block( ...
            'simulink/Ports & Subsystems/In1', ...
            velocityFeedbackInport, ...
            'Port','5', ...
            'PortDimensions','[2 1]', ...
            'OutDataTypeStr','double', ...
            'Position',[20 245 50 265]);
    else
        set_param(velocityFeedbackInport,'Port','5');
    end

    if ~blockExists(feedbackValidInport)
        add_block( ...
            'simulink/Ports & Subsystems/In1', ...
            feedbackValidInport, ...
            'Port','6', ...
            'PortDimensions','1', ...
            'OutDataTypeStr','boolean', ...
            'Position',[20 285 50 305]);
    else
        set_param(feedbackValidInport,'Port','6');
    end

    %% 修改EKF MATLAB Function输入和补偿逻辑

    coreChart = ...
        findEmbeddedMatlabChart(coreBlock);

    updatedScript = ...
        patchEkfCoreScript(coreChart.Script);

    coreChart.Script = updatedScript;

    drawnow;

    % 新输入追加在R_ekf之后，所以是函数输入9和10。
    addLineIfMissing( ...
        ekfSubsystem, ...
        'camera_velocity_feedback_xy/1', ...
        'EKF and Controller Core/9');

    addLineIfMissing( ...
        ekfSubsystem, ...
        'camera_velocity_feedback_valid/1', ...
        'EKF and Controller Core/10');

    %% 顶层连接到EKF

    addLineIfMissing( ...
        model, ...
        'Camera Velocity Feedback Source/1', ...
        'Planar Target EKF/5');

    addLineIfMissing( ...
        model, ...
        'Camera Velocity Feedback Valid Source/1', ...
        'Planar Target EKF/6');

    %% 添加调试日志

    addRootWorkspaceLog( ...
        model, ...
        'log_mapper_joint_velocity', ...
        [905 1125 1065 1155]);

    addRootWorkspaceLog( ...
        model, ...
        'log_camera_velocity_feedback', ...
        [905 1175 1065 1205]);

    addRootWorkspaceLog( ...
        model, ...
        'log_camera_velocity_feedback_valid', ...
        [905 1225 1065 1255]);

    addLineIfMissing( ...
        model, ...
        'Mapper Joint Velocity ROS2 Subscriber/1', ...
        'log_mapper_joint_velocity/1');

    addLineIfMissing( ...
        model, ...
        'Camera Velocity Feedback Source/1', ...
        'log_camera_velocity_feedback/1');

    addLineIfMissing( ...
        model, ...
        'Camera Velocity Feedback Valid Source/1', ...
        'log_camera_velocity_feedback_valid/1');

    %% 更新并保存

    set_param(model,'SimulationCommand','update');
    save_system(model,outputModelFile);

    fprintf('\n构建完成：%s\n',outputModelFile);
    fprintf('\n新增链路：\n');
    fprintf('  mapper qDot topic\n');
    fprintf('      -> FR3 camera Jacobian\n');
    fprintf('      -> camera velocity feedback XY\n');
    fprintf('      -> EKF camera-motion compensation\n');

    fprintf('\nEKF补偿规则：\n');
    fprintf('  feedback_valid=true：使用J(q)*qDotCommand\n');
    fprintf('  feedback_valid=false：回退到原来的v_previous\n');

    fprintf('\n新增日志：\n');
    fprintf('  log_mapper_joint_velocity\n');
    fprintf('  log_camera_velocity_feedback\n');
    fprintf('  log_camera_velocity_feedback_valid\n');

    fprintf('\n注意：该topic仍是命令值，不是传感器实测值。\n');

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
% 源模型定位
% =============================================================

function sourceModelFile = locateSourceModel(outputModelFile)

preferredNames = { ...
    'single_camera_xy_tracking_isnew.slx', ...
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

files = dir( ...
    fullfile(pwd,'single_camera_xy_tracking*.slx'));

if isempty(files)
    error([ ...
        '当前目录没有找到single_camera_xy_tracking*.slx。' ...
        '请把build放到模型所在目录。']);
end

fullPaths = arrayfun( ...
    @(item) fullfile(item.folder,item.name), ...
    files, ...
    'UniformOutput',false);

keep = ~strcmp(fullPaths,outputModelFile);
files = files(keep);

if isempty(files)
    error('没有找到可用于复制的源模型。');
end

[~,latestIndex] = max([files.datenum]);

sourceModelFile = fullfile( ...
    files(latestIndex).folder, ...
    files(latestIndex).name);

end


%% ============================================================
% 工程文件定位
% =============================================================

function filePath = locateProjectFile(fileName)

searchRoots = { ...
    pwd, ...
    fileparts(pwd), ...
    fileparts(fileparts(pwd)), ...
    fullfile(getenv('HOME'), ...
        'franka_ros2_ws','src')};

filePath = '';

for rootIndex = 1:numel(searchRoots)
    rootPath = searchRoots{rootIndex};

    if exist(rootPath,'dir') ~= 7
        continue;
    end

    directCandidate = ...
        fullfile(rootPath,fileName);

    if exist(directCandidate,'file') == 2
        filePath = directCandidate;
        return;
    end

    candidates = dir( ...
        fullfile(rootPath,'**',fileName));

    if ~isempty(candidates)
        % 优先选择velocity_servo_tag路径。
        candidatePaths = arrayfun( ...
            @(item) fullfile(item.folder,item.name), ...
            candidates, ...
            'UniformOutput',false);

        preferred = contains( ...
            candidatePaths, ...
            'velocity_servo_tag');

        preferredIndex = find(preferred,1,'first');

        if isempty(preferredIndex)
            preferredIndex = 1;
        end

        filePath = ...
            candidatePaths{preferredIndex};

        return;
    end
end

error('找不到工程文件：%s',fileName);

end


%% ============================================================
% URDF解析
% =============================================================

function [ ...
    originXYZ, ...
    originRPY, ...
    jointAxis, ...
    joint8XYZ, ...
    joint8RPY] = ...
    readFr3ChainFromUrdf(urdfPath)

document = xmlread(urdfPath);
jointNodes = document.getElementsByTagName('joint');

originXYZ = zeros(3,7);
originRPY = zeros(3,7);
jointAxis = zeros(3,7);

joint8XYZ = zeros(3,1);
joint8RPY = zeros(3,1);

found = false(8,1);

for nodeIndex = 0:jointNodes.getLength-1
    jointNode = jointNodes.item(nodeIndex);

    if ~jointNode.hasAttribute('name')
        continue;
    end

    jointName = char( ...
        jointNode.getAttribute('name'));

    for jointIndex = 1:8
        expectedName = ...
            sprintf('fr3_joint%d',jointIndex);

        if ~strcmp(jointName,expectedName)
            continue;
        end

        [xyz,rpy] = ...
            readJointOrigin(jointNode);

        if jointIndex <= 7
            originXYZ(:,jointIndex) = xyz;
            originRPY(:,jointIndex) = rpy;
            jointAxis(:,jointIndex) = ...
                readJointAxis(jointNode);
        else
            joint8XYZ = xyz;
            joint8RPY = rpy;
        end

        found(jointIndex) = true;
    end
end

if ~all(found)
    missingIndices = find(~found);

    error( ...
        'URDF缺少FR3关节：%s', ...
        mat2str(missingIndices(:)'));
end

end


function [xyz,rpy] = readJointOrigin(jointNode)

xyz = zeros(3,1);
rpy = zeros(3,1);

originNodes = ...
    jointNode.getElementsByTagName('origin');

if originNodes.getLength < 1
    return;
end

originNode = originNodes.item(0);

if originNode.hasAttribute('xyz')
    xyz = parseVectorAttribute( ...
        char(originNode.getAttribute('xyz')), ...
        3);
end

if originNode.hasAttribute('rpy')
    rpy = parseVectorAttribute( ...
        char(originNode.getAttribute('rpy')), ...
        3);
end

end


function axis = readJointAxis(jointNode)

axis = [0;0;1];

axisNodes = ...
    jointNode.getElementsByTagName('axis');

if axisNodes.getLength < 1
    return;
end

axisNode = axisNodes.item(0);

if axisNode.hasAttribute('xyz')
    axis = parseVectorAttribute( ...
        char(axisNode.getAttribute('xyz')), ...
        3);
end

axisNorm = norm(axis);

if ~isfinite(axisNorm) || axisNorm <= 1e-12
    error('URDF中存在无效关节轴。');
end

axis = axis/axisNorm;

end


function vector = parseVectorAttribute(textValue,count)

values = sscanf(textValue,'%f');

if numel(values) ~= count
    error( ...
        '无法解析向量属性：%s', ...
        textValue);
end

vector = reshape(double(values),count,1);

end


%% ============================================================
% YAML手眼矩阵读取
% =============================================================

function transform = readHandEyeFromYaml(yamlPath)

yamlText = fileread(yamlPath);

expression = [ ...
    'T_end_effector_camera\s*:\s*\[' ...
    '([^\]]+)\]'];

tokens = regexp( ...
    yamlText, ...
    expression, ...
    'tokens', ...
    'once');

if isempty(tokens)
    error([ ...
        'YAML中找不到T_end_effector_camera。' ...
        '请确认velocity_mapper_node配置存在该字段。']);
end

numberText = regexprep( ...
    tokens{1}, ...
    '[,\r\n]', ...
    ' ');

values = sscanf(numberText,'%f');

if numel(values) ~= 16
    error( ...
        'T_end_effector_camera应包含16个数，实际为%d。', ...
        numel(values));
end

% YAML按行展开，因此使用转置恢复4x4行主序矩阵。
transform = reshape(double(values),4,4)';

if any(~isfinite(transform(:)))
    error('手眼标定矩阵包含非有限值。');
end

if norm(transform(4,:)-[0 0 0 1]) > 1e-8
    error('手眼标定矩阵最后一行不是[0 0 0 1]。');
end

end


%% ============================================================
% MATLAB Function代码
% =============================================================

function code = mapperJointVelocityUnpackCode()

code = join([
"function [qdot_command,qdot_count,qdot_age] = fcn(data,is_new,t)"
"%#codegen"
"% 解包mapper发布的7维关节速度命令。"
""
"persistent last_qdot"
"persistent last_count"
"persistent last_time"
""
"if isempty(last_count)"
"    last_qdot=zeros(7,1);"
"    last_count=0.0;"
"    last_time=-1.0e6;"
"end"
""
"if is_new"
"    n=numel(data);"
"    last_qdot=zeros(7,1);"
""
"    copy_count=min(n,7);"
""
"    for k=1:copy_count"
"        last_qdot(k)=data(k);"
"    end"
""
"    last_count=double(copy_count);"
"    last_time=t;"
"end"
""
"qdot_command=last_qdot;"
"qdot_count=last_count;"
"qdot_age=max(0.0,t-last_time);"
"end"
],newline);

code = char(code);

end


function code = cameraVelocityEstimatorCode()

code = join([
"function [camera_velocity_feedback_xy,feedback_valid] = fcn( ..."
"    q, ..."
"    q_count, ..."
"    q_age, ..."
"    qdot_command, ..."
"    qdot_count, ..."
"    qdot_age, ..."
"    fr3_origin_xyz, ..."
"    fr3_origin_rpy, ..."
"    fr3_axis, ..."
"    fr3_joint8_origin_xyz, ..."
"    fr3_joint8_origin_rpy, ..."
"    T_link8_camera, ..."
"    feedback_timeout)"
"%#codegen"
"% 使用FR3相机Jacobian，把mapper关节速度命令转换为相机XY速度。"
"% Jacobian顺序：[linear velocity; angular velocity]。"
""
"camera_velocity_feedback_xy=zeros(2,1);"
"feedback_valid=false;"
""
"q_safe=reshape(q,7,1);"
"qdot_safe=reshape(qdot_command,7,1);"
""
"input_valid = ..."
"    q_count>=7 && ..."
"    qdot_count>=7 && ..."
"    isfinite(q_age) && ..."
"    isfinite(qdot_age) && ..."
"    q_age<=feedback_timeout && ..."
"    qdot_age<=feedback_timeout && ..."
"    all(isfinite(q_safe)) && ..."
"    all(isfinite(qdot_safe)) && ..."
"    all(isfinite(fr3_origin_xyz(:))) && ..."
"    all(isfinite(fr3_origin_rpy(:))) && ..."
"    all(isfinite(fr3_axis(:))) && ..."
"    all(isfinite(fr3_joint8_origin_xyz(:))) && ..."
"    all(isfinite(fr3_joint8_origin_rpy(:))) && ..."
"    all(isfinite(T_link8_camera(:))) && ..."
"    isfinite(feedback_timeout) && ..."
"    feedback_timeout>0;"
""
"if ~input_valid"
"    return"
"end"
""
"T=eye(4);"
"joint_origins=zeros(3,7);"
"joint_axes=zeros(3,7);"
""
"for joint_index=1:7"
"    T=T*makeTransform( ..."
"        fr3_origin_xyz(:,joint_index), ..."
"        fr3_origin_rpy(:,joint_index));"
""
"    joint_origins(:,joint_index)=T(1:3,4);"
""
"    axis_local=fr3_axis(:,joint_index);"
"    axis_norm=norm(axis_local);"
""
"    if ~isfinite(axis_norm) || axis_norm<=1e-12"
"        return"
"    end"
""
"    axis_local=axis_local/axis_norm;"
""
"    joint_axes(:,joint_index)= ..."
"        T(1:3,1:3)*axis_local;"
""
"    T=T*axisRotation(axis_local,q_safe(joint_index));"
"end"
""
"T_world_link8 = T*makeTransform( ..."
"    fr3_joint8_origin_xyz, ..."
"    fr3_joint8_origin_rpy);"
""
"T_world_camera = ..."
"    T_world_link8*T_link8_camera;"
""
"p_world_camera=T_world_camera(1:3,4);"
"R_world_camera=T_world_camera(1:3,1:3);"
""
"J_world=zeros(6,7);"
""
"for joint_index=1:7"
"    J_world(1:3,joint_index)=cross( ..."
"        joint_axes(:,joint_index), ..."
"        p_world_camera-joint_origins(:,joint_index));"
""
"    J_world(4:6,joint_index)= ..."
"        joint_axes(:,joint_index);"
"end"
""
"R_camera_world=R_world_camera';"
""
"J_camera=[ ..."
"    R_camera_world, zeros(3,3); ..."
"    zeros(3,3), R_camera_world] * J_world;"
""
"camera_twist=J_camera*qdot_safe;"
""
"if all(isfinite(camera_twist(:)))"
"    camera_velocity_feedback_xy=camera_twist(1:2);"
"    feedback_valid=true;"
"end"
"end"
""
""
"function T=makeTransform(xyz,rpy)"
"T=eye(4);"
"T(1:3,1:3)= ..."
"    rotz3(rpy(3))* ..."
"    roty3(rpy(2))* ..."
"    rotx3(rpy(1));"
"T(1:3,4)=xyz;"
"end"
""
""
"function T=axisRotation(axis,q)"
"axis_norm=norm(axis);"
"unit_axis=axis/axis_norm;"
"K=skew3(unit_axis);"
"R=eye(3)+sin(q)*K+(1-cos(q))*(K*K);"
"T=eye(4);"
"T(1:3,1:3)=R;"
"end"
""
""
"function R=rotx3(angle)"
"c=cos(angle);"
"s=sin(angle);"
"R=[1 0 0;0 c -s;0 s c];"
"end"
""
""
"function R=roty3(angle)"
"c=cos(angle);"
"s=sin(angle);"
"R=[c 0 s;0 1 0;-s 0 c];"
"end"
""
""
"function R=rotz3(angle)"
"c=cos(angle);"
"s=sin(angle);"
"R=[c -s 0;s c 0;0 0 1];"
"end"
""
""
"function S=skew3(vector)"
"S=[ ..."
"    0 -vector(3) vector(2); ..."
"    vector(3) 0 -vector(1); ..."
"    -vector(2) vector(1) 0];"
"end"
],newline);

code = char(code);

end


%% ============================================================
% EKF脚本补丁
% =============================================================

function updatedScript = patchEkfCoreScript(originalScript)

updatedScript = char(originalScript);

if contains( ...
        updatedScript, ...
        'v_camera_feedback_xy')

    error([ ...
        'EKF函数已经包含v_camera_feedback_xy。' ...
        '请不要对同一模型重复运行本build。']);
end

% 1. 在函数签名最后追加两个输入。
signaturePattern = ...
    'R_ekf\s*\)';

signatureReplacement = [ ...
    'R_ekf,' ...
    'v_camera_feedback_xy,' ...
    'camera_velocity_feedback_valid)'];

[updatedScript,replacementCount] = ...
    regexprepOnceWithCount( ...
        updatedScript, ...
        signaturePattern, ...
        signatureReplacement);

if replacementCount ~= 1
    error( ...
        '无法在EKF函数签名中定位R_ekf输入。');
end

% 2. 替换原来的命令速度积分。
compensationPattern = [ ...
    'p_camera_hat\s*=\s*' ...
    '(?:\.\.\.\s*)?' ...
    'p_camera_hat\s*\+\s*' ...
    'Ts\s*\*\s*v_previous\s*;'];

compensationReplacement = sprintf([ ...
    'if camera_velocity_feedback_valid && ...\n' ...
    '        numel(v_camera_feedback_xy)==2 && ...\n' ...
    '        all(isfinite(v_camera_feedback_xy(:)))\n' ...
    '    p_camera_hat = ...\n' ...
    '        p_camera_hat + ...\n' ...
    '        Ts*reshape(v_camera_feedback_xy,2,1);\n' ...
    'else\n' ...
    '    %% 反馈暂时不可用时保持原逻辑，避免控制中断。\n' ...
    '    p_camera_hat = ...\n' ...
    '        p_camera_hat + Ts*v_previous;\n' ...
    'end']);

[updatedScript,replacementCount] = ...
    regexprepOnceWithCount( ...
        updatedScript, ...
        compensationPattern, ...
        compensationReplacement);

if replacementCount ~= 1
    error([ ...
        '无法定位p_camera_hat积分语句。' ...
        '请确认EKF函数中仍存在p_camera_hat+Ts*v_previous。']);
end

end


function [outputText,replacementCount] = ...
    regexprepOnceWithCount(inputText,pattern,replacement)

[startIndex,endIndex] = ...
    regexp(inputText,pattern,'start','end','once');

if isempty(startIndex)
    outputText = inputText;
    replacementCount = 0;
    return;
end

outputText = [ ...
    inputText(1:startIndex-1), ...
    replacement, ...
    inputText(endIndex+1:end)];

replacementCount = 1;

end


%% ============================================================
% Simulink辅助函数
% =============================================================

function addInport( ...
    parentPath, ...
    blockName, ...
    portNumber, ...
    dimensions, ...
    position)

add_block( ...
    'simulink/Ports & Subsystems/In1', ...
    [parentPath '/' blockName], ...
    'Port',num2str(portNumber), ...
    'PortDimensions',dimensions, ...
    'OutDataTypeStr','double', ...
    'Position',position);

end


function addOutport( ...
    parentPath, ...
    blockName, ...
    portNumber, ...
    dimensions, ...
    position)

add_block( ...
    'simulink/Ports & Subsystems/Out1', ...
    [parentPath '/' blockName], ...
    'Port',num2str(portNumber), ...
    'PortDimensions',dimensions, ...
    'Position',position);

end


function addRootWorkspaceLog( ...
    model, ...
    variableName, ...
    position)

blockPath = ...
    [model '/' variableName];

if blockExists(blockPath)
    delete_block(blockPath);
end

add_block( ...
    'simulink/Sinks/To Workspace', ...
    blockPath, ...
    'VariableName',variableName, ...
    'SaveFormat','Structure With Time', ...
    'MaxDataPoints','inf', ...
    'Position',position);

end


function renameBlockIfPresent(blockPath,newName)

if blockExists(blockPath)
    set_param(blockPath,'Name',newName);
else
    error('没有找到需要重命名的模块：%s',blockPath);
end

end


function deleteBlockIfPresent(blockPath)

if blockExists(blockPath)
    delete_block(blockPath);
end

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

    if strcmp( ...
            canonicalPath(loadedFile), ...
            canonicalInput)

        if strcmp(get_param(model,'Dirty'),'on')
            save_system(model);
        end

        close_system(model,0);
        return;
    end
end

end


function pathOut = canonicalPath(pathIn)

pathOut = char( ...
    java.io.File(pathIn).getCanonicalPath());

end


function tf = blockExists(blockPath)

tf = ...
    getSimulinkBlockHandle(blockPath) ~= -1;

end


function assertBlockExists(blockPath)

if ~blockExists(blockPath)
    error('模型中没有找到模块：%s',blockPath);
end

end


function addLineIfMissing( ...
    systemPath, ...
    sourcePort, ...
    destinationPort)

sourceBlockName = ...
    extractBefore(sourcePort,'/');

destinationBlockName = ...
    extractBefore(destinationPort,'/');

sourceBlock = ...
    [systemPath '/' char(sourceBlockName)];

destinationBlock = ...
    [systemPath '/' char(destinationBlockName)];

assertBlockExists(sourceBlock);
assertBlockExists(destinationBlock);

destinationHandles = ...
    get_param(destinationBlock,'PortHandles');

destinationIndex = ...
    str2double(extractAfter(destinationPort,'/'));

if destinationIndex > ...
        numel(destinationHandles.Inport)

    error( ...
        '目标端口不存在：%s/%s', ...
        systemPath, ...
        destinationPort);
end

destinationHandle = ...
    destinationHandles.Inport(destinationIndex);

existingLine = ...
    get_param(destinationHandle,'Line');

if existingLine ~= -1
    sourceHandle = ...
        get_param(existingLine,'SrcPortHandle');

    expectedSourceHandles = ...
        get_param(sourceBlock,'PortHandles');

    sourceIndex = ...
        str2double(extractAfter(sourcePort,'/'));

    expectedSourceHandle = ...
        expectedSourceHandles.Outport(sourceIndex);

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


function chart = findEmbeddedMatlabChart(blockPath)

root = sfroot;

charts = root.find( ...
    '-isa','Stateflow.EMChart');

chart = [];

for index = 1:numel(charts)
    if strcmp(charts(index).Path,blockPath)
        chart = charts(index);
        break;
    end
end

if isempty(chart)
    error( ...
        '没有找到MATLAB Function：%s', ...
        blockPath);
end

end


function expression = matrixExpression(value)

expression = mat2str( ...
    double(value), ...
    16);

end