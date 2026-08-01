%% 四状态CV EKF自动化测试
% 运行方法：在MATLAB中执行 test_single_camera_xy_tracking_cv_ekf
test_dir=fileparts(mfilename('fullpath'));
simulink_dir=fileparts(test_dir);
config_dir=fullfile(simulink_dir,'config');
model_dir=fullfile(simulink_dir,'single_vision_xy');
addpath(config_dir,model_dir);
run(fullfile(config_dir,'init_single_camera_xy_tracking_cv_ekf.m'));

assert(isequal(size(P0),[4 4]) && isequal(size(Q_ekf),[4 4]));
assert(isequal(size(R_ekf),[2 2]));
fprintf('TEST 1 参数维度: PASS\n');

% 纯算法测试：已知相机输入必须以负号进入相对位置预测。
A=[1 0 Ts 0;0 1 0 Ts;0 0 1 0;0 0 0 1];
B=[-Ts 0;0 -Ts;0 0;0 0]; H=[eye(2) zeros(2)];
x0=[1;0;0;0];
[x1,~,~,~]=cv_ekf_step(x0,eye(4),zeros(2,1),[.05;0],false, ...
    A,B,H,zeros(4),R_ekf,ekf_gate_threshold);
assert(abs(x1(1)-(1-.05*Ts))<1e-12);
fprintf('TEST 2 相机输入负号: PASS\n');

[x_static,P_static]=run_case([0;0],[0;0],[.35;-.20],800, ...
    Ts,P0,Q_ekf,R_ekf,ekf_gate_threshold);
assert(norm(x_static(1:2)-[.35;-.20])<2e-3 && norm(x_static(3:4))<2e-3);
assert(all(isfinite([x_static;P_static(:)])));
fprintf('TEST 3 静止目标/静止相机: PASS\n');

[x_cam,~]=run_case([0;0],[.05;-.03],[.35;-.20],800, ...
    Ts,P0,Q_ekf,R_ekf,ekf_gate_threshold);
assert(norm(x_cam(3:4))<3e-3);
fprintf('TEST 4 静止目标/匀速相机，目标自身速度接近0: PASS\n');

v_target=[.04;-.025];
[x_target,~]=run_case(v_target,[0;0],[.10;.15],800, ...
    Ts,P0,Q_ekf,R_ekf,ekf_gate_threshold);
assert(norm(x_target(3:4)-v_target)<3e-3);
fprintf('TEST 5 匀速目标/静止相机: PASS\n');

[x_both,~]=run_case(v_target,[.03;.02],[.10;.15],800, ...
    Ts,P0,Q_ekf,R_ekf,ekf_gate_threshold);
assert(norm(x_both(3:4)-v_target)<3e-3);
fprintf('TEST 6 目标与相机同时运动: PASS\n');

% Jacobian换算和命令超时。
J=zeros(6,7); J(4,1)=1; J(5,2)=1; T=eye(4);
command=[.06;-.04;zeros(5,1)];
[v_cmd,valid]=command_to_camera_velocity(J,command,T,7,.01,7,.02,.1,.1);
assert(valid && norm(v_cmd-command(1:2))<1e-12);
[v_stale,valid_stale]=command_to_camera_velocity(J,command,T,7,.01,7,.11,.1,.1);
assert(~valid_stale && isequal(v_stale,zeros(2,1)));
fprintf('TEST 7 Jacobian换算与命令超时: PASS\n');

%% 模型结构、更新和短时仿真
model='single_camera_xy_tracking_cv_ekf';
model_file=fullfile(model_dir,[model '.slx']);
source_file=fullfile(model_dir,'single_camera_xy_tracking_isnew.slx');
assert(isfile(source_file) && isfile(model_file));
load_system(model_file);
init_fcn=get_param(model,'InitFcn');
assert(contains(init_fcn,'init_single_camera_xy_tracking_cv_ekf'));
assert(strcmp(get_param([model '/Target ROS2 Subscriber/Subscribe'],'topic'), ...
    '/apriltag_detector/target_position'));
assert(strcmp(get_param([model '/Joint State ROS2 Subscriber/Subscribe'],'topic'), ...
    '/franka/joint_states'));
assert(strcmp(get_param([model '/velocity_command Subscriber/Subscribe'],'topic'), ...
    '/joint_velocity_example_controller/commands'));
assert(strcmp(get_param([model '/Camera Velocity Publisher/Publish'],'topic'), ...
    '/simulink/camera_velocity'));

rt=sfroot; charts=rt.find('-isa','Stateflow.EMChart');
core=charts(strcmp({charts.Path},[model '/Planar Target CV EKF/CV KF and Controller Core']));
assert(numel(core)==1);
d=core.find('-isa','Stateflow.Data','Name','ekf_state');
d=d(strcmp({d.Scope},'Output')); assert(strcmp(d.Props.Array.Size,'[4 1]'));
d=core.find('-isa','Stateflow.Data','Name','ekf_covariance');
d=d(strcmp({d.Scope},'Output')); assert(strcmp(d.Props.Array.Size,'[4 4]'));
d=core.find('-isa','Stateflow.Data','Name','camera_velocity');
d=d(strcmp({d.Scope},'Output')); assert(strcmp(d.Props.Array.Size,'[6 1]'));
assert(contains(core.Script,'B=[-Ts 0;0 -Ts;0 0;0 0]'));
assert(~contains(core.Script,'z_base') && ~contains(core.Script,'p_target_base'));
assert(contains(core.Script,'camera_velocity=[v_issued;0;0;0;0]'));

converter=[model '/Command Velocity to Camera Velocity'];
command_sub=[model '/velocity_command Subscriber'];
ekf=[model '/Planar Target CV EKF'];
ph_converter=get_param(converter,'PortHandles');
ph_command=get_param(command_sub,'PortHandles');
ph_ekf=get_param(ekf,'PortHandles');
assert(get_param(get_param(ph_converter.Inport(1),'Line'),'SrcPortHandle')==ph_command.Outport(1));
assert(get_param(get_param(ph_ekf.Inport(5),'Line'),'SrcPortHandle')==ph_converter.Outport(1));
assert(get_param(get_param(ph_ekf.Inport(6),'Line'),'SrcPortHandle')==ph_converter.Outport(2));
assert(get_param(get_param(ph_converter.Inport(5),'Line'),'SrcPortHandle')==ph_command.Outport(3));
assert(get_param(get_param(ph_converter.Inport(6),'Line'),'SrcPortHandle')==ph_command.Outport(4));
% command_acceleration只保留日志，不进入换算、EKF或控制器。
assert(ph_command.Outport(2)~=get_param(get_param(ph_converter.Inport(1),'Line'),'SrcPortHandle'));

expected_logs={ ...
    'log_z_meas','log_e','log_ekf_state','log_v_target_hat', ...
    'log_v_camera_commanded','log_v_camera_commanded_valid', ...
    'log_command_velocity','log_command_acceleration', ...
    'log_innovation_nis','log_measurement_accepted', ...
    'log_camera_input_valid','log_v_p','log_v_ff','log_v_issued', ...
    'log_camera_velocity'};
logs=find_system(model,'LookUnderMasks','all','BlockType','ToWorkspace');
variables=cellfun(@(b)get_param(b,'VariableName'),logs,'UniformOutput',false);
assert(all(ismember(expected_logs,variables)));

% 仅测试时切换到隔离话题，不保存这些改动。
set_param([model '/Target ROS2 Subscriber/Subscribe'],'topic','/cv_ekf_test/target');
set_param([model '/Joint State ROS2 Subscriber/Subscribe'],'topic','/cv_ekf_test/joint_states');
set_param([model '/velocity_command Subscriber/Subscribe'],'topic','/cv_ekf_test/commands');
set_param([model '/Camera Velocity Publisher/Publish'],'topic','/cv_ekf_test/camera_velocity');
set_param(model,'SimulationCommand','update');
fprintf('TEST 8 模型更新、端口和日志: PASS\n');
sim(model,'StopTime','0.05','ReturnWorkspaceOutputs','on');
fprintf('TEST 9 0.05秒隔离ROS2短时仿真: PASS\n');
close_system(model,0);
fprintf('ALL CV EKF TESTS PASSED\n');

function [x,P]=run_case(v_target,v_camera,z0,N,Ts,P0,Q,R,gate)
A=[1 0 Ts 0;0 1 0 Ts;0 0 1 0;0 0 0 1];
B=[-Ts 0;0 -Ts;0 0;0 0]; H=[eye(2) zeros(2)];
z=z0; x=[z0;0;0]; P=P0;
for k=1:N
    z=z+Ts*(v_target-v_camera);
    [x,P,~,~]=cv_ekf_step(x,P,z,v_camera,true,A,B,H,Q,R,gate);
end
end
