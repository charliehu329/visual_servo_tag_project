%% 原位更新带 ROS 源时间同步的 Base 系四状态 CV KF 模型
build_dir=fileparts(mfilename('fullpath'));
simulink_dir=fileparts(fileparts(build_dir));
model_dir=fullfile(simulink_dir,'single_vision_xy');
config_dir=fullfile(simulink_dir,'config');
script_dir=fullfile(build_dir,'block_scripts');
target_file=fullfile(model_dir,'single_camera_xy_tracking_base_cv_kf.slx');
assert(isfile(target_file),'Final model not found: %s',target_file)

[~,model]=fileparts(target_file);
load_system(target_file);
rt=sfroot;
addpath(config_dir);

%% 八元素视觉消息
target_sub=[model '/Target ROS2 Subscriber'];
target_chart=get_chart(rt,[target_sub '/Unpack']);
target_chart.Script=fileread(fullfile(script_dir,'target_measurement_unpack_fcn.m'));
set_size(target_chart,'target_data','Output','[8 1]');

%% JointState Header 源时间戳
joint_sub=[model '/Joint State ROS2 Subscriber'];
selector=[joint_sub '/Select Joint Arrays'];
set_param(selector,'OutputSignals', ...
    'position,velocity,header.stamp.sec,header.stamp.nanosec');

stamp_block=[joint_sub '/Joint Source Timestamp'];
delete_block_if_present(stamp_block);
stamp_chart=add_matlab_function(joint_sub,'Joint Source Timestamp', ...
    [365 175 555 255],fullfile(script_dir,'joint_source_stamp_fcn.m'));
for name={'sec','nsec'}
    set_size(stamp_chart,name{1},'Input','1');
end
for name={'joint_source_stamp','joint_source_stamp_valid'}
    set_size(stamp_chart,name{1},'Output','1');
end
ensure_outport(joint_sub,'joint_source_stamp',5,[620 180 650 194]);
ensure_outport(joint_sub,'joint_source_stamp_valid',6,[620 220 650 234]);
ensure_outport(joint_sub,'joint_message_is_new',7,[620 260 650 274]);
connect_force(joint_sub,block_out(joint_sub,'Select Joint Arrays',3), ...
    get_port(stamp_chart.Path,'Inport',1));
connect_force(joint_sub,block_out(joint_sub,'Select Joint Arrays',4), ...
    get_port(stamp_chart.Path,'Inport',2));
wire_chart_outputs(joint_sub,stamp_chart, ...
    {'joint_source_stamp','joint_source_stamp_valid'});
connect_force(joint_sub,get_port([joint_sub '/Subscribe'],'Outport',1), ...
    get_port([joint_sub '/joint_message_is_new'],'Inport',1));

%% 视觉源时间戳校验
validation=[model '/Target Measurement Validation'];
delete_block_if_present(validation);
validation=create_empty_subsystem(model,'Target Measurement Validation', ...
    [305 70 545 310]);
validation_inputs={'target_data','target_count','target_age'};
for k=1:numel(validation_inputs)
    add_inport(validation,validation_inputs{k},k,[20 35+50*(k-1) 50 49+50*(k-1)]);
end
add_block('simulink/Sources/Constant',[validation '/Target Timeout'], ...
    'Value','target_timeout_sec','Position',[70 195 155 220]);
validation_core=add_matlab_function(validation,'Validate Timestamped Visual', ...
    [190 65 390 215], ...
    fullfile(script_dir,'timestamped_visual_validation_fcn.m'));
validation_outputs={'position_valid','velocity_valid','position_C', ...
    'relative_velocity_C','visual_source_stamp','visual_source_stamp_valid'};
for k=1:numel(validation_outputs)
    add_outport(validation,validation_outputs{k},k, ...
        [445 30+40*(k-1) 475 44+40*(k-1)]);
end
set_size(validation_core,'target_data','Input','[8 1]');
set_size(validation_core,'position_C','Output','[2 1]');
set_size(validation_core,'relative_velocity_C','Output','[2 1]');
for name={'target_count','target_age','target_timeout_sec'}
    set_size(validation_core,name{1},'Input','1');
end
for name={'position_valid','velocity_valid','visual_source_stamp', ...
        'visual_source_stamp_valid'}
    set_size(validation_core,name{1},'Output','1');
end
wire_chart_inputs(validation,validation_core,[ ...
    block_out(validation,'target_data',1), ...
    block_out(validation,'target_count',1), ...
    block_out(validation,'target_age',1), ...
    block_out(validation,'Target Timeout',1)]);
wire_chart_outputs(validation,validation_core,validation_outputs);
connect_force(model,subsystem_out(target_sub,'target_data'), ...
    subsystem_in(validation,'target_data'));
connect_force(model,subsystem_out(target_sub,'target_count'), ...
    subsystem_in(validation,'target_count'));
connect_force(model,subsystem_out(target_sub,'target_age'), ...
    subsystem_in(validation,'target_age'));

%% 固定容量 JointState/视觉时间同步
aligner=[model '/Visual Joint Time Alignment'];
delete_block_if_present(aligner);
aligner=create_empty_subsystem(model,'Visual Joint Time Alignment', ...
    [650 55 995 650]);
aligner_inputs={'position_C','relative_velocity_C','position_valid', ...
    'velocity_valid','visual_source_stamp','visual_source_stamp_valid', ...
    'visual_message_is_new','joint_position','joint_velocity','joint_count', ...
    'joint_age','joint_source_stamp','joint_source_stamp_valid', ...
    'joint_message_is_new','sim_time'};
for k=1:numel(aligner_inputs)
    col=floor((k-1)/8);
    row=mod(k-1,8);
    add_inport(aligner,aligner_inputs{k},k, ...
        [20+145*col 20+45*row 50+145*col 34+45*row]);
end
flag_input_indices=[3 4 6 7 13 14];
for k=1:numel(flag_input_indices)
    index=flag_input_indices(k);
    name=aligner_inputs{index};
    conversion=[aligner '/' name ' to double'];
    add_block('simulink/Signal Attributes/Data Type Conversion',conversion, ...
        'OutDataTypeStr','double', ...
        'Position',[300 20+28*(k-1) 380 38+28*(k-1)]);
    connect_force(aligner,block_out(aligner,name,1),get_port(conversion,'Inport',1));
end
add_block('simulink/Sources/Constant',[aligner '/Parameters'], ...
    'Value',['[joint_buffer_capacity;pending_visual_capacity;' ...
    'sync_max_bracket_span_sec;sync_max_visual_wait_sec;' ...
    'sync_clock_reset_threshold_sec;double(sync_allow_extrapolation);' ...
    'double(sync_required_for_control);joint_state_timeout_sec]'], ...
    'Position',[315 390 500 430]);
aligner_core=add_matlab_function(aligner,'Align JointState to Visual Time', ...
    [545 100 790 500],fullfile(script_dir,'joint_visual_time_aligner_fcn.m'));
aligner_outputs={'aligned_position_C','aligned_relative_velocity_C', ...
    'aligned_q','aligned_qdot','aligned_visual_stamp', ...
    'aligned_joint_stamp_0','aligned_joint_stamp_1','sync_alpha', ...
    'sync_bracket_span','sync_wait_time','sync_valid', ...
    'aligned_measurement_is_new','aligned_position_valid', ...
    'aligned_velocity_valid','sync_drop_reason','joint_buffer_count', ...
    'pending_visual_count','joint_out_of_order_drop_count', ...
    'visual_sync_drop_count','clock_reset_count','sync_reset_required'};
for k=1:numel(aligner_outputs)
    col=floor((k-1)/11);
    row=mod(k-1,11);
    add_outport(aligner,aligner_outputs{k},k, ...
        [845+190*col 20+43*row 875+190*col 34+43*row]);
end
for name={'position_C','relative_velocity_C'}
    set_size(aligner_core,name{1},'Input','[2 1]');
end
for name={'joint_position','joint_velocity'}
    set_size(aligner_core,name{1},'Input','[7 1]');
end
set_size(aligner_core,'parameters','Input','[8 1]');
scalar_aligner_inputs={'position_valid','velocity_valid', ...
    'visual_source_stamp','visual_source_stamp_valid', ...
    'visual_message_is_new','joint_count','joint_age','joint_source_stamp', ...
    'joint_source_stamp_valid','joint_message_is_new','sim_time'};
for k=1:numel(scalar_aligner_inputs)
    set_size(aligner_core,scalar_aligner_inputs{k},'Input','1');
end
for name={'aligned_position_C','aligned_relative_velocity_C'}
    set_size(aligner_core,name{1},'Output','[2 1]');
end
for name={'aligned_q','aligned_qdot'}
    set_size(aligner_core,name{1},'Output','[7 1]');
end
scalar_aligner_outputs=setdiff(aligner_outputs,{ ...
    'aligned_position_C','aligned_relative_velocity_C','aligned_q','aligned_qdot'}, ...
    'stable');
for k=1:numel(scalar_aligner_outputs)
    set_size(aligner_core,scalar_aligner_outputs{k},'Output','1');
end
aligner_sources=zeros(1,numel(aligner_inputs));
for k=1:numel(aligner_inputs)
    if any(k==flag_input_indices)
        aligner_sources(k)=block_out(aligner,[aligner_inputs{k} ' to double'],1);
    else
        aligner_sources(k)=block_out(aligner,aligner_inputs{k},1);
    end
end
aligner_sources=[aligner_sources block_out(aligner,'Parameters',1)];
wire_chart_inputs(aligner,aligner_core,aligner_sources);
wire_chart_outputs(aligner,aligner_core,aligner_outputs);

sync_clock=[model '/Synchronization Clock'];
ensure_block('simulink/Sources/Clock',sync_clock,'Position',[565 610 595 630]);
connect_force(model,subsystem_out(validation,'position_C'), ...
    subsystem_in(aligner,'position_C'));
connect_force(model,subsystem_out(validation,'relative_velocity_C'), ...
    subsystem_in(aligner,'relative_velocity_C'));
connect_force(model,subsystem_out(validation,'position_valid'), ...
    subsystem_in(aligner,'position_valid'));
connect_force(model,subsystem_out(validation,'velocity_valid'), ...
    subsystem_in(aligner,'velocity_valid'));
connect_force(model,subsystem_out(validation,'visual_source_stamp'), ...
    subsystem_in(aligner,'visual_source_stamp'));
connect_force(model,subsystem_out(validation,'visual_source_stamp_valid'), ...
    subsystem_in(aligner,'visual_source_stamp_valid'));
connect_force(model,subsystem_out(target_sub,'target_is_new'), ...
    subsystem_in(aligner,'visual_message_is_new'));
connect_force(model,subsystem_out(joint_sub,'joint_position'), ...
    subsystem_in(aligner,'joint_position'));
connect_force(model,subsystem_out(joint_sub,'joint_velocity'), ...
    subsystem_in(aligner,'joint_velocity'));
connect_force(model,subsystem_out(joint_sub,'joint_count'), ...
    subsystem_in(aligner,'joint_count'));
connect_force(model,subsystem_out(joint_sub,'joint_age'), ...
    subsystem_in(aligner,'joint_age'));
connect_force(model,subsystem_out(joint_sub,'joint_source_stamp'), ...
    subsystem_in(aligner,'joint_source_stamp'));
connect_force(model,subsystem_out(joint_sub,'joint_source_stamp_valid'), ...
    subsystem_in(aligner,'joint_source_stamp_valid'));
connect_force(model,subsystem_out(joint_sub,'joint_message_is_new'), ...
    subsystem_in(aligner,'joint_message_is_new'));
connect_force(model,get_port(sync_clock,'Outport',1), ...
    subsystem_in(aligner,'sim_time'));

%% Transform/Jacobian 仅使用插值后关节状态
twist=[model '/Measured Camera Twist in Base'];
connect_force(model,subsystem_out(aligner,'aligned_qdot'), ...
    subsystem_in(twist,'joint_velocity_measured'));
connect_force(model,subsystem_out(aligner,'aligned_q'), ...
    subsystem_in(twist,'joint_position'));
connect_force(model,subsystem_out(aligner,'sync_valid'), ...
    subsystem_in(twist,'joint_motion_valid'));
connect_force(model,subsystem_out(aligner,'sync_valid'), ...
    subsystem_in(twist,'joint_geometry_valid'));

%% 同步测量 -> Base 坐标系
measurement=[model '/Camera Measurement to Base'];
connect_force(model,subsystem_out(aligner,'aligned_position_C'), ...
    subsystem_in(measurement,'position_C'));
connect_force(model,subsystem_out(aligner,'aligned_relative_velocity_C'), ...
    subsystem_in(measurement,'relative_velocity_C'));
connect_force(model,subsystem_out(twist,'T_camera2base'), ...
    subsystem_in(measurement,'T_camera2base'));
connect_force(model,subsystem_out(twist,'camera_twist_base'), ...
    subsystem_in(measurement,'camera_twist_base'));
connect_force(model,subsystem_out(aligner,'aligned_position_valid'), ...
    subsystem_in(measurement,'position_valid'));
connect_force(model,subsystem_out(aligner,'aligned_velocity_valid'), ...
    subsystem_in(measurement,'velocity_valid'));
connect_force(model,subsystem_out(twist,'camera_motion_valid'), ...
    subsystem_in(measurement,'camera_motion_valid'));

%% KF 仅在同步新测量脉冲上更新
kf=[model '/Base-Frame Target CV KF'];
ensure_inport(kf,'sync_reset_required',8,[20 270 50 284]);
kf_core=get_chart(rt,[kf '/Base CV KF and Controller']);
kf_core.Script=fileread(fullfile(script_dir,'base_cv_kf_controller_fcn.m'));
set_size(kf_core,'position_C','Input','[2 1]');
set_size(kf_core,'position_B_measured','Input','[2 1]');
set_size(kf_core,'velocity_B_measured','Input','[2 1]');
set_size(kf_core,'T_camera2base','Input','[4 4]');
set_size(kf_core,'R_position','Input','[2 2]');
set_size(kf_core,'R_velocity','Input','[2 2]');
set_size(kf_core,'P0','Input','[4 4]');
set_size(kf_core,'Q_kf','Input','[4 4]');
set_size(kf_core,'parameters','Input','[12 1]');
kf_input_names={'position_C','position_B_measured','velocity_B_measured', ...
    'position_measurement_valid','velocity_measurement_valid', ...
    'measurement_is_new','T_camera2base','sync_reset_required'};
constant_names={'R Position','R Velocity','P0','Q KF','Parameters', ...
    'Required Frames','Reset Timeout','FF Ramp Time'};
kf_sources=arrayfun(@(k)block_out(kf,kf_input_names{k},1),1:numel(kf_input_names));
for k=1:numel(constant_names)
    kf_sources(end+1)=block_out(kf,constant_names{k},1); %#ok<SAGROW>
end
wire_chart_inputs(kf,kf_core,kf_sources);
connect_force(model,subsystem_out(aligner,'aligned_position_C'), ...
    subsystem_in(kf,'position_C'));
connect_force(model,subsystem_out(measurement,'position_B_measured'), ...
    subsystem_in(kf,'position_B_measured'));
connect_force(model,subsystem_out(measurement,'velocity_B_measured'), ...
    subsystem_in(kf,'velocity_B_measured'));
connect_force(model,subsystem_out(measurement,'position_measurement_valid'), ...
    subsystem_in(kf,'position_measurement_valid'));
connect_force(model,subsystem_out(measurement,'velocity_measurement_valid'), ...
    subsystem_in(kf,'velocity_measurement_valid'));
connect_force(model,subsystem_out(aligner,'aligned_measurement_is_new'), ...
    subsystem_in(kf,'measurement_is_new'));
connect_force(model,subsystem_out(twist,'T_camera2base'), ...
    subsystem_in(kf,'T_camera2base'));
connect_force(model,subsystem_out(aligner,'sync_reset_required'), ...
    subsystem_in(kf,'sync_reset_required'));

watchdog=[model '/Watchdog and Safety'];
connect_force(model,subsystem_out(measurement,'position_measurement_valid'), ...
    get_port(watchdog,'Inport',1));

%% 接收年龄差仅作诊断日志
delta=[model '/Measurement Joint Age Delta'];
age_difference=[model '/Measurement Joint Age Difference'];
ensure_block('simulink/Math Operations/Sum',delta, ...
    'Inputs','+-','Position',[1060 680 1090 720]);
ensure_block('simulink/Math Operations/Abs',age_difference, ...
    'Position',[1125 685 1160 715]);
connect_force(model,subsystem_out(target_sub,'target_age'),get_port(delta,'Inport',1));
connect_force(model,subsystem_out(joint_sub,'joint_age'),get_port(delta,'Inport',2));
connect_force(model,get_port(delta,'Outport',1),get_port(age_difference,'Inport',1));

%% 日志
log_names={ ...
    'log_target_position_C','log_target_relative_velocity_C', ...
    'log_target_measurement_age','log_joint_state_age', ...
    'log_measurement_joint_age_difference','log_visual_source_stamp', ...
    'log_joint_source_stamp_latest','log_aligned_joint_stamp_0', ...
    'log_aligned_joint_stamp_1','log_sync_alpha','log_sync_bracket_span', ...
    'log_sync_wait_time','log_sync_valid','log_sync_drop_reason', ...
    'log_joint_buffer_count','log_pending_visual_count', ...
    'log_joint_out_of_order_drop_count','log_visual_sync_drop_count', ...
    'log_clock_reset_count','log_target_position_B_measured', ...
    'log_target_velocity_B_measured','log_position_measurement_valid', ...
    'log_velocity_measurement_valid','log_position_innovation', ...
    'log_velocity_innovation','log_position_nis','log_velocity_nis', ...
    'log_position_measurement_accepted','log_velocity_measurement_accepted', ...
    'log_valid_streak','log_kf_ready','log_ff_ramp','log_position_C_hold', ...
    'log_kf_state_B','log_kf_velocity_C_for_feedforward','log_v_p', ...
    'log_v_ff','log_v_raw','log_v_issued','log_camera_velocity'};
log_sources=[ ...
    subsystem_out(validation,'position_C'), ...
    subsystem_out(validation,'relative_velocity_C'), ...
    subsystem_out(target_sub,'target_age'),subsystem_out(joint_sub,'joint_age'), ...
    get_port(age_difference,'Outport',1), ...
    subsystem_out(validation,'visual_source_stamp'), ...
    subsystem_out(joint_sub,'joint_source_stamp'), ...
    subsystem_out(aligner,'aligned_joint_stamp_0'), ...
    subsystem_out(aligner,'aligned_joint_stamp_1'), ...
    subsystem_out(aligner,'sync_alpha'), ...
    subsystem_out(aligner,'sync_bracket_span'), ...
    subsystem_out(aligner,'sync_wait_time'),subsystem_out(aligner,'sync_valid'), ...
    subsystem_out(aligner,'sync_drop_reason'), ...
    subsystem_out(aligner,'joint_buffer_count'), ...
    subsystem_out(aligner,'pending_visual_count'), ...
    subsystem_out(aligner,'joint_out_of_order_drop_count'), ...
    subsystem_out(aligner,'visual_sync_drop_count'), ...
    subsystem_out(aligner,'clock_reset_count'), ...
    subsystem_out(measurement,'position_B_measured'), ...
    subsystem_out(measurement,'velocity_B_measured'), ...
    subsystem_out(measurement,'position_measurement_valid'), ...
    subsystem_out(measurement,'velocity_measurement_valid'), ...
    subsystem_out(kf,'position_innovation'),subsystem_out(kf,'velocity_innovation'), ...
    subsystem_out(kf,'position_nis'),subsystem_out(kf,'velocity_nis'), ...
    subsystem_out(kf,'position_measurement_accepted'), ...
    subsystem_out(kf,'velocity_measurement_accepted'), ...
    subsystem_out(kf,'valid_streak'),subsystem_out(kf,'kf_ready'), ...
    subsystem_out(kf,'ff_ramp'),subsystem_out(kf,'position_C_hold'), ...
    subsystem_out(kf,'kf_state_B'), ...
    subsystem_out(kf,'kf_velocity_C_for_feedforward'), ...
    subsystem_out(kf,'v_p'),subsystem_out(kf,'v_ff'), ...
    subsystem_out(kf,'v_raw'),subsystem_out(kf,'v_issued'), ...
    get_port(watchdog,'Outport',1)];
delete_block_if_present([model '/Logging']);
logging=create_logging_subsystem(model,log_names,[1260 55 1780 720]);
for k=1:numel(log_names)
    connect_force(model,log_sources(k),get_port(logging,'Inport',k));
end

save_system(model,target_file);
run(fullfile(config_dir,'init_single_camera_xy_tracking_base_cv_kf.m'));
set_param(model,'SimulationCommand','update');
save_system(model,target_file);
close_system(model,0);
fprintf('UPDATED %s\n',target_file);

function path=create_empty_subsystem(model,name,pos)
path=[model '/' name];
add_block('simulink/Ports & Subsystems/Subsystem',path,'Position',pos);
Simulink.SubSystem.deleteContents(path);
end

function chart=add_matlab_function(parent,name,pos,script_file)
path=[parent '/' name];
add_block('simulink/User-Defined Functions/MATLAB Function',path,'Position',pos);
rt=sfroot;
chart=get_chart(rt,path);
chart.Script=fileread(script_file);
end

function chart=get_chart(rt,path)
all=rt.find('-isa','Stateflow.EMChart');
chart=all(strcmp({all.Path},path));
assert(numel(chart)==1,'MATLAB Function not found: %s',path)
end

function set_size(chart,name,scope,size_text)
d=chart.find('-isa','Stateflow.Data','Name',name);
d=d(strcmp({d.Scope},scope));
assert(numel(d)==1,'Data not found: %s (%s)',name,scope)
d.Props.Array.Size=size_text;
end

function add_inport(parent,name,port,pos)
add_block('simulink/Sources/In1',[parent '/' name], ...
    'Port',num2str(port),'Position',pos);
end

function ensure_inport(parent,name,port,pos)
path=[parent '/' name];
if getSimulinkBlockHandle(path)<=0
    add_block('simulink/Sources/In1',path,'Port',num2str(port),'Position',pos);
else
    set_param(path,'Port',num2str(port));
end
end

function add_outport(parent,name,port,pos)
add_block('simulink/Sinks/Out1',[parent '/' name], ...
    'Port',num2str(port),'Position',pos);
end

function ensure_outport(parent,name,port,pos)
path=[parent '/' name];
if getSimulinkBlockHandle(path)<=0
    add_block('simulink/Sinks/Out1',path,'Port',num2str(port),'Position',pos);
else
    set_param(path,'Port',num2str(port));
end
end

function ensure_block(library,path,varargin)
if getSimulinkBlockHandle(path)<=0
    add_block(library,path,varargin{:});
else
    set_param(path,varargin{:});
end
end

function delete_block_if_present(path)
if getSimulinkBlockHandle(path)>0
    delete_block(path);
end
end

function h=block_out(parent,name,index)
p=get_param([parent '/' name],'PortHandles');
h=p.Outport(index);
end

function h=get_port(block,kind,index)
p=get_param(block,'PortHandles');
h=p.(kind)(index);
end

function h=subsystem_out(subsystem,name)
port=str2double(get_param([subsystem '/' name],'Port'));
h=get_port(subsystem,'Outport',port);
end

function h=subsystem_in(subsystem,name)
port=str2double(get_param([subsystem '/' name],'Port'));
h=get_port(subsystem,'Inport',port);
end

function connect_force(parent,source_port,destination_port)
line=get_param(destination_port,'Line');
if line~=-1
    delete_line(line);
end
add_line(parent,source_port,destination_port,'autorouting','on');
end

function wire_chart_inputs(parent,chart,sources)
p=get_param(chart.Path,'PortHandles');
assert(numel(p.Inport)==numel(sources))
for k=1:numel(sources)
    connect_force(parent,sources(k),p.Inport(k));
end
end

function wire_chart_outputs(parent,chart,names)
p=get_param(chart.Path,'PortHandles');
assert(numel(p.Outport)==numel(names))
for k=1:numel(names)
    connect_force(parent,p.Outport(k),get_port([parent '/' names{k}],'Inport',1));
end
end

function logging=create_logging_subsystem(model,names,pos)
logging=[model '/Logging'];
add_block('simulink/Ports & Subsystems/Subsystem',logging,'Position',pos);
Simulink.SubSystem.deleteContents(logging);
for k=1:numel(names)
    row=mod(k-1,14);
    col=floor((k-1)/14);
    x=20+280*col;
    y=20+35*row;
    add_block('simulink/Sources/In1',[logging '/' names{k}], ...
        'Port',num2str(k),'Position',[x y x+30 y+14]);
    sink=[logging '/' names{k} '_sink'];
    add_block('simulink/Sinks/To Workspace',sink, ...
        'VariableName',names{k},'SaveFormat','Timeseries', ...
        'Position',[x+105 y-3 x+230 y+17]);
    add_line(logging,[names{k} '/1'],[names{k} '_sink/1']);
end
end
