%% 使用 Simulink 官方 API 从原模型生成四状态 CV EKF 模型
build_dir=fileparts(mfilename('fullpath'));
simulink_dir=fileparts(fileparts(build_dir));
model_dir=fullfile(simulink_dir,'single_vision_xy');
config_dir=fullfile(simulink_dir,'config');
script_dir=fullfile(build_dir,'block_scripts');
source_file=fullfile(model_dir,'single_camera_xy_tracking_isnew.slx');
target_file=fullfile(model_dir,'single_camera_xy_tracking_cv_ekf.slx');
assert(isfile(source_file),'找不到原模型: %s',source_file);

[~,source_model]=fileparts(source_file);
load_system(source_file);
save_system(source_model,target_file);
close_system(source_model,0);
[~,model]=fileparts(target_file);
load_system(target_file);
rt=sfroot;

%% InitFcn：加入配置目录并调用新初始化脚本
init_code=sprintf([ ...
    'config_dir=fullfile(fileparts(get_param(bdroot,''FileName'')),''..'',''config'');\n' ...
    'addpath(config_dir);\n' ...
    'init_single_camera_xy_tracking_cv_ekf;']);
set_param(model,'InitFcn',init_code);
set_param(model,'Description', ...
    'Camera-frame 4-state CV Kalman Filter with commanded camera motion input.');

%% velocity_command Subscriber：增加时间、新鲜度、计数和年龄
command_sub=[model '/velocity_command Subscriber'];
command_chart=get_chart(rt,[command_sub '/MATLAB Function']);
command_chart.Script=fileread(fullfile(script_dir,'velocity_command_subscriber_fcn.m'));
set_size(command_chart,'command_velocity','Output','[7 1]');
set_size(command_chart,'command_acceleration','Output','[7 1]');
set_size(command_chart,'command_count','Output','1');
set_size(command_chart,'command_age','Output','1');
set_size(command_chart,'command_is_new','Output','1');
set_size(command_chart,'is_new','Input','1');
set_size(command_chart,'t','Input','1');

clock_path=[command_sub '/Clock'];
if isempty(find_system(command_sub,'SearchDepth',1,'Name','Clock'))
    add_block('simulink/Sources/Clock',clock_path,'Position',[155 160 185 180]);
end
ph_chart=get_param([command_sub '/MATLAB Function'],'PortHandles');
ph_clock=get_param(clock_path,'PortHandles');
connect(command_sub,ph_clock.Outport(1),ph_chart.Inport(3));
add_outport(command_sub,'command_count','3',[480 115 510 135]);
add_outport(command_sub,'command_age','4',[480 155 510 175]);
add_outport(command_sub,'command_is_new','5',[480 195 510 215]);
ph_chart=get_param([command_sub '/MATLAB Function'],'PortHandles');
connect_block_output(command_sub,ph_chart,3,[command_sub '/command_count']);
connect_block_output(command_sub,ph_chart,4,[command_sub '/command_age']);
connect_block_output(command_sub,ph_chart,5,[command_sub '/command_is_new']);

%% 复用 Jacobian 子系统：命令关节速度 -> 相机系XY速度
old_converter=[model '/v_measured Subscribe'];
set_param(old_converter,'Name','Command Velocity to Camera Velocity');
converter=[model '/Command Velocity to Camera Velocity'];
set_param([converter '/qdot_measured'],'Name','command_velocity');
set_param([converter '/v_camera_measured'],'Name','v_camera_commanded');
set_param([converter '/feedback_valid'],'Name','camera_command_valid');
add_inport(converter,'command_count','5',[25 205 55 225]);
add_inport(converter,'command_age','6',[25 245 55 265]);

converter_chart=get_chart(rt,[converter '/MATLAB Function']);
converter_chart.Script=fileread(fullfile(script_dir,'command_velocity_to_camera_fcn.m'));
set_size(converter_chart,'v_camera_commanded','Output','[2 1]');
set_size(converter_chart,'camera_command_valid','Output','1');
set_size(converter_chart,'T_camera2base_out','Output','[4 4]');
set_size(converter_chart,'J_camera_base','Input','[6 7]');
set_size(converter_chart,'command_velocity','Input','[7 1]');
set_size(converter_chart,'T_camera2base','Input','[4 4]');
for scalar_name={ 'joint_count','joint_age','command_count','command_age', ...
        'joint_state_timeout_sec','command_velocity_timeout_sec'}
    set_size(converter_chart,scalar_name{1},'Input','1');
end

old_constant=[converter '/Constant'];
set_param(old_constant,'Name','Joint State Timeout','Value','joint_state_timeout_sec');
command_timeout=[converter '/Command Velocity Timeout'];
if isempty(find_system(converter,'SearchDepth',1,'Name','Command Velocity Timeout'))
    add_block('simulink/Sources/Constant',command_timeout, ...
        'Value','command_velocity_timeout_sec','Position',[255 270 335 300]);
end
converter_core=[converter '/MATLAB Function'];
disconnect_all_inputs(converter,converter_core);
ph=get_param(converter_core,'PortHandles');
assert(numel(ph.Inport)==9,'命令速度换算核心应有9个输入');
connect_ports(converter,[converter '/Get Jacobian'],1,converter_core,1);
connect_ports(converter,[converter '/command_velocity'],1,converter_core,2);
connect_ports(converter,[converter '/Get Transform'],1,converter_core,3);
connect_ports(converter,[converter '/joint_count'],1,converter_core,4);
connect_ports(converter,[converter '/joint_age'],1,converter_core,5);
connect_ports(converter,[converter '/command_count'],1,converter_core,6);
connect_ports(converter,[converter '/command_age'],1,converter_core,7);
connect_ports(converter,[converter '/Joint State Timeout'],1,converter_core,8);
connect_ports(converter,command_timeout,1,converter_core,9);

%% Planar Target EKF -> Planar Target CV EKF
old_ekf=[model '/Planar Target EKF'];
set_param(old_ekf,'Name','Planar Target CV EKF');
ekf=[model '/Planar Target CV EKF'];
set_param([ekf '/T_camera2base'],'Name','v_camera_input');
add_inport(ekf,'camera_input_valid','6',[30 250 60 270]);

ekf_chart=get_chart(rt,[ekf '/EKF and Controller Core']);
ekf_chart.Script=fileread(fullfile(script_dir,'cv_ekf_controller_fcn.m'));
set_size(ekf_chart,'ekf_state','Output','[4 1]');
set_size(ekf_chart,'ekf_covariance','Output','[4 4]');
set_size(ekf_chart,'v_camera_input','Input','[2 1]');
set_size(ekf_chart,'camera_input_valid','Input','1');
set_size(ekf_chart,'v_norm_limited','Output','[2 1]');
set_size(ekf_chart,'z_meas','Input','[2 1]');
set_size(ekf_chart,'e','Input','[2 1]');
set_size(ekf_chart,'R_ekf','Input','[2 2]');
set_size(ekf_chart,'P0','Input','[4 4]');
set_size(ekf_chart,'Q_ekf','Input','[4 4]');
set_size(ekf_chart,'parameters','Input','[12 1]');
for scalar_name={ 'safe_valid','measurement_is_new','camera_input_valid', ...
        'required_valid_frames','measurement_reset_timeout','ff_ramp_time'}
    set_size(ekf_chart,scalar_name{1},'Input','1');
end

ekf_core=[ekf '/EKF and Controller Core'];
disconnect_all_inputs(ekf,ekf_core);
ph=get_param(ekf_core,'PortHandles');
assert(numel(ph.Inport)==13,'CV EKF核心应有13个输入');
connect_ports(ekf,[ekf '/z_meas'],1,ekf_core,1);
connect_ports(ekf,[ekf '/e'],1,ekf_core,2);
connect_ports(ekf,[ekf '/safe_valid'],1,ekf_core,3);
connect_ports(ekf,[ekf '/measurement_is_new'],1,ekf_core,4);
connect_ports(ekf,[ekf '/v_camera_input'],1,ekf_core,5);
connect_ports(ekf,[ekf '/camera_input_valid'],1,ekf_core,6);
connect_ports(ekf,[ekf '/R EKF'],1,ekf_core,7);
connect_ports(ekf,[ekf '/P0'],1,ekf_core,8);
connect_ports(ekf,[ekf '/Q EKF'],1,ekf_core,9);
connect_ports(ekf,[ekf '/Vector Concatenate1'],1,ekf_core,10);
connect_ports(ekf,[ekf '/Constant7'],1,ekf_core,11);
connect_ports(ekf,[ekf '/Constant8'],1,ekf_core,12);
connect_ports(ekf,[ekf '/Constant9'],1,ekf_core,13);
set_param(ekf_core,'Name','CV KF and Controller Core');

%% 顶层重连：command_velocity -> Jacobian -> v_camera_commanded -> CV EKF
ph_converter=get_param(converter,'PortHandles');
ph_command=get_param(command_sub,'PortHandles');
ph_ekf=get_param(ekf,'PortHandles');
disconnect_input(model,ph_converter.Inport(1)); % 不再使用实测joint_velocity。
connect(model,ph_command.Outport(1),ph_converter.Inport(1));
connect(model,ph_command.Outport(3),ph_converter.Inport(5));
connect(model,ph_command.Outport(4),ph_converter.Inport(6));
disconnect_input(model,ph_ekf.Inport(5));       % 删除T_camera2base到EKF的旧连接。
connect(model,ph_converter.Outport(1),ph_ekf.Inport(5));
connect(model,ph_converter.Outport(2),ph_ekf.Inport(6));

% T_camera2base仅保留在速度换算模块中；顶层输出接终止器。
t_term=[model '/T_camera2base Terminator'];
if isempty(find_system(model,'SearchDepth',1,'Name','T_camera2base Terminator'))
    add_block('simulink/Sinks/Terminator',t_term,'Position',[910 700 930 720]);
end
connect(model,ph_converter.Outport(3),get_param(t_term,'PortHandles').Inport(1));

%% 日志命名与新增诊断日志
set_workspace_by_sid(model,264,'log_z_meas');
set_workspace_by_sid(model,265,'log_e');
set_workspace_by_sid(model,266,'log_v_camera_commanded');
set_workspace_by_sid(model,267,'log_v_camera_commanded_valid');
set_workspace_by_sid(model,286,'log_command_velocity');
set_workspace_by_sid(model,287,'log_command_acceleration');
set_param([model '/Logging/log_ekf_velocity'],'VariableName','log_v_target_hat');
add_workspace_log(model,'log_camera_input_valid',[1010 650 1130 680], ...
    ph_converter.Outport(2));
add_workspace_log(model,'log_command_count',[1010 735 1130 765], ...
    ph_command.Outport(3));
add_workspace_log(model,'log_command_age',[1010 775 1130 805], ...
    ph_command.Outport(4));
add_workspace_log(model,'log_command_is_new',[1010 815 1130 845], ...
    ph_command.Outport(5));

save_system(model,target_file);
addpath(config_dir);
run(fullfile(config_dir,'init_single_camera_xy_tracking_cv_ekf.m'));
set_param(model,'SimulationCommand','update');
save_system(model,target_file);
close_system(model,0);
fprintf('BUILT %s\n',target_file);

function ch=get_chart(rt,path)
all=rt.find('-isa','Stateflow.EMChart'); ch=all(strcmp({all.Path},path));
assert(numel(ch)==1,'找不到唯一MATLAB Function: %s',path);
end

function set_size(ch,name,scope,size_text)
d=ch.find('-isa','Stateflow.Data','Name',name); d=d(strcmp({d.Scope},scope));
assert(numel(d)==1,'找不到唯一数据 %s (%s)',name,scope);
d.Props.Array.Size=size_text;
end

function add_inport(parent,name,port,pos)
path=[parent '/' name];
if isempty(find_system(parent,'SearchDepth',1,'Name',name))
    add_block('simulink/Sources/In1',path,'Port',port,'Position',pos);
else
    set_param(path,'Port',port,'Position',pos);
end
end

function add_outport(parent,name,port,pos)
path=[parent '/' name];
if isempty(find_system(parent,'SearchDepth',1,'Name',name))
    add_block('simulink/Sinks/Out1',path,'Port',port,'Position',pos);
else
    set_param(path,'Port',port,'Position',pos);
end
end

function connect_block_output(parent,source_handles,index,destination)
dst=get_param(destination,'PortHandles'); connect(parent,source_handles.Outport(index),dst.Inport(1));
end

function connect_ports(parent,source,source_port,destination,destination_port)
src=get_param(source,'PortHandles'); dst=get_param(destination,'PortHandles');
connect(parent,src.Outport(source_port),dst.Inport(destination_port));
end

function connect(parent,source_port,destination_port)
if get_param(destination_port,'Line')==-1
    add_line(parent,source_port,destination_port,'autorouting','on');
end
end

function disconnect_all_inputs(parent,block)
ph=get_param(block,'PortHandles');
for k=1:numel(ph.Inport), disconnect_input(parent,ph.Inport(k)); end
end

function disconnect_input(parent,input_port)
line=get_param(input_port,'Line');
if line~=-1
    source=get_param(line,'SrcPortHandle');
    delete_line(parent,source,input_port);
end
end

function set_workspace_by_sid(model,sid,name)
handle=Simulink.ID.getHandle([model ':' num2str(sid)]);
assert(handle~=-1,'找不到SID %d',sid);
set_param(handle,'VariableName',name);
end

function add_workspace_log(model,name,pos,source_port)
path=[model '/' name];
if isempty(find_system(model,'SearchDepth',1,'Name',name))
    add_block('simulink/Sinks/To Workspace',path,'VariableName',name, ...
        'SaveFormat','Timeseries','Position',pos);
end
dst=get_param(path,'PortHandles'); connect(model,source_port,dst.Inport(1));
end
