function [ekf_state,ekf_covariance,v_target_hat,v_p,v_ff, ...
    v_norm_limited,v_issued,camera_velocity,saturation_active, ...
    acceleration_active,ekf_initialized,innovation_nis, ...
    measurement_accepted,controller_ok]=fcn( ...
    z_meas,e,safe_valid,measurement_is_new,v_camera_input, ...
    camera_input_valid,R_ekf,P0,Q_ekf,parameters, ...
    required_valid_frames,measurement_reset_timeout,ff_ramp_time)
%#codegen
% 带相机运动输入的四状态匀速滤波器及P+速度前馈控制器。
% 状态：[X_C;Y_C;V_tx,C;V_ty,C]，测量和速度均在相机坐标系表达。
% 适用条件：固定Z_hat、相机仅XY平移、vz=0、角速度=0、忽略姿态变化。
% 系统和测量方程均为线性；保留EKF名称仅为兼容现有项目结构。
persistent x P e_hold elapsed initialized v_previous valid_streak ekf_ready ff_ramp
if isempty(initialized)
    x=zeros(4,1); P=P0; e_hold=zeros(2,1); elapsed=0;
    initialized=false; v_previous=zeros(2,1); valid_streak=0;
    ekf_ready=false; ff_ramp=0;
end

ekf_state=x; ekf_covariance=P; v_target_hat=zeros(2,1);
v_p=zeros(2,1); v_ff=zeros(2,1); v_norm_limited=zeros(2,1);
v_issued=zeros(2,1); camera_velocity=zeros(6,1);
saturation_active=false; acceleration_active=false;
ekf_initialized=initialized&&ekf_ready; innovation_nis=NaN;
measurement_accepted=false; controller_ok=false;

if numel(parameters)<12, x(:)=0; P=P0; initialized=false; ekf_ready=false; return; end
Ts=parameters(1); Z_hat=parameters(2); Kp=parameters(3:4);
k_ff=parameters(5); gate=parameters(6); enable_p=parameters(7)>.5;
enable_ff=parameters(8)>.5; controller_enable=parameters(9)>.5;
v_xy_max=parameters(11); a_xy_max=parameters(12);
safe_valid=safe_valid>.5; measurement_is_new=measurement_is_new>.5;
required_valid_frames=max(1,floor(required_valid_frames));
parameter_ok=all(isfinite(parameters(:))) && ...
    isfinite(required_valid_frames) && isfinite(measurement_reset_timeout) && ...
    isfinite(ff_ramp_time) && Ts>0 && Z_hat>0 && gate>0 && ...
    measurement_reset_timeout>=Ts && ff_ramp_time>0 && ...
    v_xy_max>0 && a_xy_max>0 && isequal(size(P0),[4 4]) && ...
    isequal(size(Q_ekf),[4 4]) && isequal(size(R_ekf),[2 2]) && ...
    all(isfinite(P0(:))) && all(isfinite(Q_ekf(:))) && all(isfinite(R_ekf(:)));
if ~parameter_ok
    x(:)=0; P=P0; e_hold(:)=0; elapsed=0; initialized=false;
    v_previous(:)=0; valid_streak=0; ekf_ready=false; ff_ramp=0;
    ekf_state=x; ekf_covariance=P; ekf_initialized=false; return
end

A=[1 0 Ts 0;0 1 0 Ts;0 0 1 0;0 0 0 1];
B=[-Ts 0;0 -Ts;0 0;0 0];
H=[1 0 0 0;0 1 0 0];
vectors_ok=numel(z_meas)==2 && numel(e)==2 && ...
    all(isfinite(z_meas(:))) && all(isfinite(e(:)));
current_visual_valid=safe_valid&&vectors_ok;
measurement_available=current_visual_valid&&measurement_is_new;
z_camera=zeros(2,1);
if vectors_ok, z_camera=reshape(z_meas,2,1); end

input_ok=camera_input_valid>.5 && numel(v_camera_input)==2 && ...
    all(isfinite(v_camera_input(:)));
u_used=zeros(2,1);
if input_ok, u_used=reshape(v_camera_input,2,1); end

if ~initialized
    if measurement_available
        x=[z_camera;0;0]; P=P0; initialized=true; measurement_accepted=true;
    end
else
    [x,P,innovation_nis,measurement_accepted]=cv_ekf_step( ...
        x,P,z_camera,u_used,measurement_available,A,B,H,Q_ekf,R_ekf,gate);
end

if current_visual_valid&&measurement_is_new, e_hold=reshape(e,2,1); end
if measurement_accepted
    elapsed=0; valid_streak=min(valid_streak+1,required_valid_frames);
    if valid_streak>=required_valid_frames, ekf_ready=true; end
else
    if measurement_is_new&&~ekf_ready, valid_streak=0; end
    elapsed=elapsed+Ts;
end

state_ok=all(isfinite(x)) && all(isfinite(P(:))) && ...
    all(diag(P)>=0) && norm(P-P','fro')<1e-6;
if elapsed>=measurement_reset_timeout || ~state_ok
    x(:)=0; P=P0; e_hold(:)=0; elapsed=0; initialized=false;
    v_previous(:)=0; valid_streak=0; ekf_ready=false; ff_ramp=0;
    ekf_state=x; ekf_covariance=P; ekf_initialized=false; return
end

if ~current_visual_valid
    e_hold(:)=0; v_previous(:)=0; ff_ramp=0;
    ekf_state=x; ekf_covariance=P;
    if ekf_ready, v_target_hat=x(3:4); else, ekf_state(3:4)=0; end
    ekf_initialized=initialized&&ekf_ready; return
end

ekf_state=x; ekf_covariance=P;
if ekf_ready, v_target_hat=x(3:4); else, ekf_state(3:4)=0; end
ekf_initialized=initialized&&ekf_ready;
if ~controller_enable||~initialized
    v_previous(:)=0; ff_ramp=0; return
end

if enable_p, v_p=Z_hat*Kp.*e_hold; end
if enable_ff&&ekf_ready
    ff_ramp=min(1,ff_ramp+Ts/ff_ramp_time);
    v_ff=ff_ramp*k_ff*x(3:4);
else
    ff_ramp=0;
end
raw=v_p+v_ff; raw_norm=norm(raw); saturation_active=raw_norm>v_xy_max;
if saturation_active, v_norm_limited=raw*(v_xy_max/raw_norm); else, v_norm_limited=raw; end
delta=v_norm_limited-v_previous; delta_norm=norm(delta); delta_max=a_xy_max*Ts;
acceleration_active=delta_norm>delta_max;
if acceleration_active
    v_issued=v_previous+delta*(delta_max/delta_norm);
else
    v_issued=v_norm_limited;
end
controller_ok=all(isfinite([v_p;v_ff;v_norm_limited;v_issued]));
if controller_ok
    v_previous=v_issued;
    camera_velocity=[v_issued;0;0;0;0];
else
    v_p(:)=0; v_ff(:)=0; v_norm_limited(:)=0; v_issued(:)=0;
    saturation_active=false; acceleration_active=false;
    v_previous(:)=0; ff_ramp=0; camera_velocity(:)=0;
end
end

