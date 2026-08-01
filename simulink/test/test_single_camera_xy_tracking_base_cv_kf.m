%% Base CV KF 视觉/JointState 源时间同步回归测试
test_dir=fileparts(mfilename('fullpath'));
simulink_dir=fileparts(test_dir);
config_dir=fullfile(simulink_dir,'config');
model_dir=fullfile(simulink_dir,'single_vision_xy');
block_dir=fullfile(simulink_dir,'build','single_vision_xy','block_scripts');
addpath(config_dir,model_dir,block_dir);
run(fullfile(config_dir,'init_single_camera_xy_tracking_base_cv_kf.m'));
sync_parameters=[joint_buffer_capacity;pending_visual_capacity; ...
    sync_max_bracket_span_sec;sync_max_visual_wait_sec; ...
    sync_clock_reset_threshold_sec;double(sync_allow_extrapolation); ...
    double(sync_required_for_control);joint_state_timeout_sec];

%% 1. 八元素消息解析
data=[1 1 .1 -.2 .03 -.04 123 456789123];
[pv,vv,pC,vC,t,stamp_ok]=parse_target_measurement( ...
    data,8,.01,target_timeout_sec);
assert(pv&&vv&&stamp_ok)
assert(isequal(pC,[.1;-.2])&&isequal(vC,[.03;-.04]))
assert(abs(t-(123+.456789123))<1e-12)
assert(~parse_target_measurement(data,6,.01,target_timeout_sec))
fprintf('TEST 1 timestamped eight-element payload: PASS\n')

%% 2. 非法源时间戳
bad_stamps=[-1 0;1 -1;1 1e9;NaN 0;Inf 0;0 0];
for k=1:size(bad_stamps,1)
    bad=data;
    bad(7:8)=bad_stamps(k,:);
    [bp,bv,~,~,~,bok]=parse_target_measurement( ...
        bad,8,.01,target_timeout_sec);
    assert(~bp&&~bv&&~bok)
end
fprintf('TEST 2 invalid source stamps: PASS\n')

%% 3. 精确时间匹配
clear joint_visual_time_aligner_step
align_step(zeros(2,1),zeros(2,1),0,0,0,0,0, ...
    ones(7,1),2*ones(7,1),7,0,10,1,1,0,sync_parameters);
s=align_step([.1;.2],[0;0],1,1,10,1,1, ...
    zeros(7,1),zeros(7,1),0,0,0,0,0,.01,sync_parameters);
assert(s.sync&&s.new&&s.pvalid&&s.vvalid)
assert(isequal(s.q,ones(7,1))&&isequal(s.qdot,2*ones(7,1)))
assert(s.alpha==0&&s.t0==10&&s.t1==10)
fprintf('TEST 3 exact JointState match: PASS\n')

%% 4. 中点插值
clear joint_visual_time_aligner_step
q0=(1:7)'; q1=q0+2; qd0=2*q0; qd1=qd0+4;
align_step(zeros(2,1),zeros(2,1),0,0,0,0,0, ...
    q0,qd0,7,0,20,1,1,0,sync_parameters);
align_step(zeros(2,1),zeros(2,1),0,0,0,0,0, ...
    q1,qd1,7,0,20.01,1,1,.01,sync_parameters);
s=align_step([.2;-.1],[.01;.02],1,1,20.005,1,1, ...
    zeros(7,1),zeros(7,1),0,0,0,0,0,.02,sync_parameters);
assert(s.sync&&abs(s.alpha-.5)<1e-12)
assert(norm(s.q-(q0+q1)/2)<1e-12)
assert(norm(s.qdot-(qd0+qd1)/2)<1e-10)
fprintf('TEST 4 midpoint interpolation: PASS\n')

%% 5. 等待未来 JointState，不外推
clear joint_visual_time_aligner_step
align_step(zeros(2,1),zeros(2,1),0,0,0,0,0, ...
    zeros(7,1),zeros(7,1),7,0,30,1,1,0,sync_parameters);
s=align_step([.1;.1],[0;0],1,0,30.005,1,1, ...
    zeros(7,1),zeros(7,1),0,0,0,0,0,.01,sync_parameters);
assert(~s.new&&~s.sync&&s.pending==1)
s=align_step([.1;.1],[0;0],1,0,30.005,1,0, ...
    zeros(7,1),zeros(7,1),0,0,0,0,0,.02,sync_parameters);
assert(~s.new&&s.pending==1)
s=align_step(zeros(2,1),zeros(2,1),0,0,0,0,0, ...
    ones(7,1),ones(7,1),7,0,30.01,1,1,.03,sync_parameters);
assert(s.new&&s.sync&&s.pending==0)
s=align_step(zeros(2,1),zeros(2,1),0,0,0,0,0, ...
    zeros(7,1),zeros(7,1),0,0,0,0,0,.04,sync_parameters);
assert(~s.new)
fprintf('TEST 5 wait for future bracket without extrapolation: PASS\n')

%% 6. 视觉消息早于最旧 JointState
clear joint_visual_time_aligner_step
align_step(zeros(2,1),zeros(2,1),0,0,0,0,0, ...
    zeros(7,1),zeros(7,1),7,0,40,1,1,0,sync_parameters);
s=align_step([0;0],[0;0],1,0,39.9,1,1, ...
    zeros(7,1),zeros(7,1),0,0,0,0,0,.01,sync_parameters);
assert(~s.sync&&s.new&&s.reason==1&&s.visual_drops==1)
fprintf('TEST 6 too-old visual drop: PASS\n')

%% 7. 包围间隔过大
clear joint_visual_time_aligner_step
align_step(zeros(2,1),zeros(2,1),0,0,0,0,0, ...
    zeros(7,1),zeros(7,1),7,0,50,1,1,0,sync_parameters);
align_step(zeros(2,1),zeros(2,1),0,0,0,0,0, ...
    ones(7,1),ones(7,1),7,0,50.05,1,1,.01,sync_parameters);
s=align_step([0;0],[0;0],1,0,50.025,1,1, ...
    zeros(7,1),zeros(7,1),0,0,0,0,0,.02,sync_parameters);
assert(~s.sync&&s.new&&s.reason==2)
fprintf('TEST 7 bracket-too-wide rejection: PASS\n')

%% 8. 视觉等待超时
clear joint_visual_time_aligner_step
align_step(zeros(2,1),zeros(2,1),0,0,0,0,0, ...
    zeros(7,1),zeros(7,1),7,0,60,1,1,0,sync_parameters);
align_step([0;0],[0;0],1,0,61,1,1, ...
    zeros(7,1),zeros(7,1),0,0,0,0,0,0,sync_parameters);
s=align_step(zeros(2,1),zeros(2,1),0,0,0,0,0, ...
    zeros(7,1),zeros(7,1),0,0,0,0,0, ...
    sync_max_visual_wait_sec+.001,sync_parameters);
assert(~s.sync&&s.new&&s.reason==3&&s.pending==0)
fprintf('TEST 8 pending visual timeout: PASS\n')

%% 9. 重复 JointState 时间戳替换最后样本
clear joint_visual_time_aligner_step
align_step(zeros(2,1),zeros(2,1),0,0,0,0,0, ...
    ones(7,1),ones(7,1),7,0,70,1,1,0,sync_parameters);
align_step(zeros(2,1),zeros(2,1),0,0,0,0,0, ...
    9*ones(7,1),3*ones(7,1),7,0,70,1,1,.01,sync_parameters);
s=align_step([0;0],[0;0],1,1,70,1,1, ...
    zeros(7,1),zeros(7,1),0,0,0,0,0,.02,sync_parameters);
assert(s.joint_count==1&&isequal(s.q,9*ones(7,1)))
fprintf('TEST 9 duplicate JointState replacement: PASS\n')

%% 10. 小幅乱序 JointState 丢弃计数
clear joint_visual_time_aligner_step
align_step(zeros(2,1),zeros(2,1),0,0,0,0,0, ...
    zeros(7,1),zeros(7,1),7,0,80,1,1,0,sync_parameters);
s=align_step(zeros(2,1),zeros(2,1),0,0,0,0,0, ...
    ones(7,1),ones(7,1),7,0,79.9,1,1,.01,sync_parameters);
assert(s.joint_count==1&&s.joint_drops==1&&s.resets==0)
fprintf('TEST 10 out-of-order JointState drop: PASS\n')

%% 11. ROS 时钟向后跳变清空队列并复位 KF
clear joint_visual_time_aligner_step
align_step(zeros(2,1),zeros(2,1),0,0,0,0,0, ...
    zeros(7,1),zeros(7,1),7,0,90,1,1,0,sync_parameters);
align_step([0;0],[0;0],1,0,91,1,1, ...
    zeros(7,1),zeros(7,1),0,0,0,0,0,.01,sync_parameters);
s=align_step(zeros(2,1),zeros(2,1),0,0,0,0,0, ...
    ones(7,1),ones(7,1),7,0,89,1,1,.02,sync_parameters);
assert(s.reset&&s.resets==1&&s.pending==0&&s.joint_count==1&&~s.sync)
clear base_cv_kf_controller_fcn
parameters=controller_parameters(Ts,Z_hat,Kpx,Kpy,k_ff, ...
    nis_position_threshold,nis_velocity_threshold,v_xy_max,a_xy_max);
c=controller_step([.1;.1],[.1;.1],[0;0],1,0,1,eye(4),0, ...
    R_position,R_velocity,P0,Q_kf,parameters,1, ...
    measurement_reset_timeout,ff_ramp_time);
assert(c.pa)
c=controller_step([.1;.1],[.1;.1],[0;0],0,0,0,eye(4),1, ...
    R_position,R_velocity,P0,Q_kf,parameters,1, ...
    measurement_reset_timeout,ff_ramp_time);
assert(~c.initialized&&~c.ready&&c.ramp==0&&isequal(c.x,zeros(4,1)))
fprintf('TEST 11 ROS clock reset and KF reset: PASS\n')

%% 12. 异步静止目标补偿
clear joint_visual_time_aligner_step
t0=100; dt=.01; a=.8; tv=t0+dt/2;
c0=.5*a*t0^0; %#ok<NASGU>
q0=zeros(7,1); q1=zeros(7,1); qd0=zeros(7,1); qd1=zeros(7,1);
q0(1)=0; q1(1)=.5*a*dt^2; qd0(1)=0; qd1(1)=a*dt;
align_step(zeros(2,1),zeros(2,1),0,0,0,0,0, ...
    q0,qd0,7,0,t0,1,1,0,sync_parameters);
align_step(zeros(2,1),zeros(2,1),0,0,0,0,0, ...
    q1,qd1,7,0,t0+dt,1,1,.01,sync_parameters);
camera_position=.5*a*(dt/2)^2;
camera_velocity=a*dt/2;
s=align_step([.6-camera_position;0],[-camera_velocity;0], ...
    1,1,tv,1,1,zeros(7,1),zeros(7,1),0,0,0,0,0,.02,sync_parameters);
T=eye(4); T(1:2,4)=s.q(1:2);
twist=[0;0;0;s.qdot(1:2);0];
[~,v_new,~,vok]=camera_measurement_to_base( ...
    s.p,s.v,Z_hat,T,twist,1,1,1);
v_old=[qd1(1);0]+s.v;
assert(vok&&norm(v_new)<1e-12&&norm(v_new)<norm(v_old))
fprintf('TEST 12 asynchronous stationary-target compensation: PASS\n')

%% 13. 目标与相机同时运动时 KF 收敛
clear joint_visual_time_aligner_step
A=[1 0 Ts 0;0 1 0 Ts;0 0 1 0;0 0 0 1];
x=zeros(4,1); P=P0; target0=[.4;-.2]; vt=[.03;-.02]; vc=[.08;.04];
for k=1:300
    ta=200+(k-1)*Ts; tb=ta+Ts; tm=(ta+tb)/2;
    qa=zeros(7,1); qb=zeros(7,1); qda=zeros(7,1); qdb=zeros(7,1);
    qa(1:2)=vc*(ta-200); qb(1:2)=vc*(tb-200);
    qda(1:2)=vc; qdb(1:2)=vc;
    align_step(zeros(2,1),zeros(2,1),0,0,0,0,0, ...
        qa,qda,7,0,ta,1,1,(k-1)*Ts,sync_parameters);
    align_step(zeros(2,1),zeros(2,1),0,0,0,0,0, ...
        qb,qdb,7,0,tb,1,1,(k-.5)*Ts,sync_parameters);
    target=target0+vt*(tm-200); camera=vc*(tm-200);
    s=align_step(target-camera,vt-vc,1,1,tm,1,1, ...
        zeros(7,1),zeros(7,1),0,0,0,0,0,k*Ts,sync_parameters);
    T=eye(4); T(1:2,4)=s.q(1:2);
    twist=[0;0;0;s.qdot(1:2);0];
    [pB,vB,pok,vok]=camera_measurement_to_base( ...
        s.p,s.v,Z_hat,T,twist,s.pvalid,s.vvalid,s.sync);
    [x,P]=base_cv_kf_step(x,P,pB,vB,pok,vok,A,Q_kf, ...
        R_position,R_velocity,1e6,1e6);
end
assert(norm(x(3:4)-vt)<2e-3)
fprintf('TEST 13 asynchronous moving-target KF convergence: PASS\n')

%% 14. 同步失败安全门控
clear base_cv_kf_controller_fcn
c1=controller_step([.2;-.1],[.2;-.1],[0;0],1,0,1,eye(4),0, ...
    R_position,R_velocity,P0,Q_kf,parameters,1, ...
    measurement_reset_timeout,ff_ramp_time);
c2=controller_step([99;99],[99;99],[99;99],0,0,1,eye(4),0, ...
    R_position,R_velocity,P0,Q_kf,parameters,1, ...
    measurement_reset_timeout,ff_ramp_time);
assert(c1.pa&&~c2.pa&&isequal(c2.hold,c1.hold))
assert(isequal(c2.camera,zeros(6,1)))
fprintf('TEST 14 synchronization failure safety gate: PASS\n')

%% 15. 禁止最新 JointState 旁路
model='single_camera_xy_tracking_base_cv_kf';
model_file=fullfile(model_dir,[model '.slx']);
load_system(model_file);
aligner_path=[model '/Visual Joint Time Alignment'];
twist_path=[model '/Measured Camera Twist in Base'];
assert_source(subsystem_in(twist_path,'joint_position'),aligner_path)
assert_source(subsystem_in(twist_path,'joint_velocity_measured'),aligner_path)
assert_source(subsystem_in(twist_path,'joint_geometry_valid'),aligner_path)
assert_source(subsystem_in(twist_path,'joint_motion_valid'),aligner_path)
selector=[model '/Joint State ROS2 Subscriber/Select Joint Arrays'];
assert(strcmp(get_param(selector,'OutputSignals'), ...
    'position,velocity,header.stamp.sec,header.stamp.nanosec'))
fprintf('TEST 15 no latest-JointState bypass: PASS\n')

%% 16. 512 样本环形缓冲回绕
clear joint_visual_time_aligner_step
for k=1:520
    qk=k*ones(7,1);
    s=align_step(zeros(2,1),zeros(2,1),0,0,0,0,0, ...
        qk,qk,7,0,300+k*.001,1,1,k*.001,sync_parameters);
end
assert(s.joint_count==512)
s=align_step([0;0],[0;0],1,0,300.009,1,1, ...
    zeros(7,1),zeros(7,1),0,0,0,0,0,.6,sync_parameters);
assert(s.sync&&isequal(s.q,9*ones(7,1)))
s=align_step([0;0],[0;0],1,0,300.008,1,1, ...
    zeros(7,1),zeros(7,1),0,0,0,0,0,.61,sync_parameters);
assert(~s.sync&&s.reason==1)
fprintf('TEST 16 fixed ring-buffer wrap: PASS\n')

%% 17. Update Diagram 和短时隔离 ROS2 仿真
set_param(model,'SimulationCommand','update');
rt=sfroot;
charts=rt.find('-isa','Stateflow.EMChart');
target_chart=charts(strcmp({charts.Path},[model '/Target ROS2 Subscriber/Unpack']));
d=target_chart.find('-isa','Stateflow.Data','Name','target_data');
d=d(strcmp({d.Scope},'Output'));
assert(strcmp(d.Props.Array.Size,'[8 1]'))
required_logs={'log_visual_source_stamp','log_joint_source_stamp_latest', ...
    'log_aligned_joint_stamp_0','log_aligned_joint_stamp_1','log_sync_alpha', ...
    'log_sync_bracket_span','log_sync_wait_time','log_sync_valid', ...
    'log_sync_drop_reason','log_joint_buffer_count','log_pending_visual_count', ...
    'log_joint_out_of_order_drop_count','log_visual_sync_drop_count', ...
    'log_clock_reset_count'};
logs=find_system([model '/Logging'],'LookUnderMasks','all','BlockType','ToWorkspace');
variables=cellfun(@(b)get_param(b,'VariableName'),logs,'UniformOutput',false);
assert(all(ismember(required_logs,variables)))
set_param([model '/Target ROS2 Subscriber/Subscribe'],'topic','/sync_test/target');
set_param([model '/Joint State ROS2 Subscriber/Subscribe'],'topic','/sync_test/joints');
set_param([model '/velocity_command Subscriber/Subscribe'],'topic','/sync_test/commands');
set_param([model '/Camera Velocity Publisher/Publish'],'topic','/sync_test/camera_velocity');
sim_out=sim(model,'StopTime','.05','ReturnWorkspaceOutputs','on');
camera_log=sim_out.log_camera_velocity;
sync_log=sim_out.log_sync_valid;
assert(all(isfinite(camera_log.Data(:)))&&all(camera_log.Data(:)==0))
assert(all(sync_log.Data(:)==0))
close_system(model,0);
fprintf('TEST 17 model update and isolated ROS2 simulation: PASS\n')
fprintf('ALL 17 TIME-SYNCHRONIZATION TESTS PASSED\n')

function s=align_step(position,velocity,pv,vv,tv,tvok,vnew,q,qd, ...
    count,age,tj,tjok,jnew,now,parameters)
[s.p,s.v,s.q,s.qdot,s.tv,s.t0,s.t1,s.alpha,s.span,s.wait, ...
    s.sync,s.new,s.pvalid,s.vvalid,s.reason,s.joint_count,s.pending, ...
    s.joint_drops,s.visual_drops,s.resets,s.reset]= ...
    joint_visual_time_aligner_step(position,velocity,pv,vv,tv,tvok,vnew,q,qd, ...
    count,age,tj,tjok,jnew,now,parameters(1),parameters(2),parameters(3), ...
    parameters(4),parameters(5),parameters(6),parameters(7),parameters(8));
end

function p=controller_parameters(Ts,Z,Kpx,Kpy,kff,gp,gv,vmax,amax)
p=[Ts;Z;Kpx;Kpy;kff;gp;gv;1;1;1;vmax;amax];
end

function c=controller_step(pC,pB,vB,pv,vv,isnew,T,sync_reset,Rp,Rv,P0,Q, ...
    parameters,frames,reset_timeout,ramp_time)
[c.x,c.P,~,~,c.vC,c.vp,c.vff,~,~,~,c.camera,~,~,c.initialized, ...
    ~,~,~,~,c.pa,c.va,c.ok,c.streak,c.ready,c.ramp,c.hold]= ...
    base_cv_kf_controller_fcn(pC,pB,vB,pv,vv,isnew,T,sync_reset, ...
    Rp,Rv,P0,Q,parameters,frames,reset_timeout,ramp_time);
end

function h=subsystem_in(subsystem,name)
port=str2double(get_param([subsystem '/' name],'Port'));
p=get_param(subsystem,'PortHandles');
h=p.Inport(port);
end

function assert_source(destination_port,expected_subsystem)
line=get_param(destination_port,'Line');
assert(line~=-1)
source=get_param(line,'SrcBlockHandle');
assert(strcmp(getfullname(source),expected_subsystem))
end
