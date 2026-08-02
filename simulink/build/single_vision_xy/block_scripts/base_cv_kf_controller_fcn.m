function [kf_state_B,kf_covariance,kf_position_B,kf_velocity_B, ...
    kf_velocity_C_for_feedforward,v_p,v_ff,v_raw,v_norm_limited,v_issued, ...
    camera_velocity,saturation_active,acceleration_active,kf_initialized, ...
    position_innovation,velocity_innovation,position_nis,velocity_nis, ...
    position_measurement_accepted,velocity_measurement_accepted, ...
    controller_ok,valid_streak_out,kf_ready_out,ff_ramp_out, ...
    position_C_hold_out] = fcn(position_C,position_B_measured, ...
    velocity_B_measured,position_measurement_valid, ...
    velocity_measurement_valid,measurement_is_new,T_camera2base, ...
    sync_reset_required, ...
    R_position,R_velocity,P0,Q_kf,parameters,required_valid_frames, ...
    measurement_reset_timeout,ff_ramp_time)
%#codegen
% 执行 Base 系四状态 CV KF、接受门控和 P+KF 速度前馈控制。
persistent x P position_C_hold elapsed initialized v_previous valid_streak kf_ready ff_ramp
if isempty(initialized)
    x=zeros(4,1);
    P=P0;
    position_C_hold=zeros(2,1);
    elapsed=0;
    initialized=false;
    v_previous=zeros(2,1);
    valid_streak=0;
    kf_ready=false;
    ff_ramp=0;
end

kf_state_B=x;
kf_covariance=P;
kf_position_B=x(1:2);
kf_velocity_B=zeros(2,1);
kf_velocity_C_for_feedforward=zeros(2,1);
v_p=zeros(2,1);
v_ff=zeros(2,1);
v_raw=zeros(2,1);
v_norm_limited=zeros(2,1);
v_issued=zeros(2,1);
camera_velocity=zeros(6,1);
saturation_active=false;
acceleration_active=false;
kf_initialized=initialized&&kf_ready;
position_innovation=zeros(2,1);
velocity_innovation=zeros(2,1);
position_nis=NaN;
velocity_nis=NaN;
position_measurement_accepted=false;
velocity_measurement_accepted=false;
controller_ok=false;
valid_streak_out=valid_streak;
kf_ready_out=kf_ready;
ff_ramp_out=ff_ramp;
position_C_hold_out=position_C_hold;

if sync_reset_required>.5
    x=zeros(4,1);
    P=P0;
    position_C_hold=zeros(2,1);
    elapsed=0;
    initialized=false;
    v_previous=zeros(2,1);
    valid_streak=0;
    kf_ready=false;
    ff_ramp=0;
    kf_state_B=x;
    kf_covariance=P;
    kf_position_B=x(1:2);
    kf_initialized=false;
    valid_streak_out=0;
    kf_ready_out=false;
    ff_ramp_out=0;
    position_C_hold_out=position_C_hold;
    return
end

if numel(parameters)<12
    return
end
Ts=parameters(1);
Z_hat=parameters(2);
Kp=parameters(3:4);
k_ff=parameters(5);
position_gate=parameters(6);
velocity_gate=parameters(7);
enable_p=parameters(8)>.5;
enable_ff=parameters(9)>.5;
controller_enable=parameters(10)>.5;
v_xy_max=parameters(11);
a_xy_max=parameters(12);
required_valid_frames=max(1,floor(required_valid_frames));

parameter_ok=all(isfinite(parameters(:))) && Ts>0 && Z_hat>0 && ...
    position_gate>0 && velocity_gate>0 && v_xy_max>0 && a_xy_max>0 && ...
    measurement_reset_timeout>0 && ff_ramp_time>0 && ...
    isequal(size(P0),[4 4]) && isequal(size(Q_kf),[4 4]) && ...
    isequal(size(R_position),[2 2]) && isequal(size(R_velocity),[2 2]) && ...
    all(isfinite([P0(:);Q_kf(:);R_position(:);R_velocity(:)]));
if ~parameter_ok
    x=zeros(4,1);
    P=P0;
    position_C_hold=zeros(2,1);
    elapsed=0;
    initialized=false;
    v_previous=zeros(2,1);
    valid_streak=0;
    kf_ready=false;
    ff_ramp=0;
    return
end

A=[1 0 Ts 0;0 1 0 Ts;0 0 1 0;0 0 0 1];
current_visual_valid=position_measurement_valid>.5 && ...
    numel(position_C)==2 && all(isfinite(position_C(:)));
new_position=current_visual_valid && measurement_is_new>.5 && ...
    numel(position_B_measured)==2 && all(isfinite(position_B_measured(:)));
new_velocity=velocity_measurement_valid>.5 && measurement_is_new>.5 && ...
    numel(velocity_B_measured)==2 && all(isfinite(velocity_B_measured(:)));

if ~initialized
    if new_position
        x=[reshape(position_B_measured,2,1);zeros(2,1)];
        P=P0;
        initialized=true;
        position_measurement_accepted=true;
    end
else
    [x,P,position_innovation,velocity_innovation,position_nis,velocity_nis, ...
        position_measurement_accepted,velocity_measurement_accepted]= ...
        base_cv_kf_step(x,P,position_B_measured,velocity_B_measured, ...
        new_position,new_velocity,A,Q_kf,R_position,R_velocity, ...
        position_gate,velocity_gate);
end

P=(P+P')/2;
state_ok=all(isfinite(x)) && all(isfinite(P(:))) && ...
    all(diag(P)>=-1e-12);
if measurement_is_new>.5
    if position_measurement_accepted
        valid_streak=min(valid_streak+1,required_valid_frames);
    else
        valid_streak=0;
        kf_ready=false;
        ff_ramp=0;
    end
end
if position_measurement_accepted
    elapsed=0;
    position_C_hold=reshape(position_C,2,1);
else
    elapsed=elapsed+Ts;
end
if valid_streak>=required_valid_frames
    kf_ready=true;
end
if ~current_visual_valid
    valid_streak=0;
    kf_ready=false;
    ff_ramp=0;
end

reset_required=elapsed>=measurement_reset_timeout || ~state_ok;
if reset_required
    x=zeros(4,1);
    P=P0;
    position_C_hold=zeros(2,1);
    elapsed=0;
    initialized=false;
    v_previous=zeros(2,1);
    valid_streak=0;
    kf_ready=false;
    ff_ramp=0;
    kf_state_B=x;
    kf_covariance=P;
    kf_position_B=x(1:2);
    kf_initialized=false;
    valid_streak_out=0;
    kf_ready_out=false;
    ff_ramp_out=0;
    position_C_hold_out=position_C_hold;
    return
end

kf_state_B=x;
kf_covariance=P;
kf_position_B=x(1:2);
if kf_ready
    kf_velocity_B=x(3:4);
end
kf_initialized=initialized&&kf_ready;
valid_streak_out=valid_streak;
kf_ready_out=kf_ready;
ff_ramp_out=ff_ramp;
position_C_hold_out=position_C_hold;

transform_ok=isequal(size(T_camera2base),[4 4]) && ...
    all(isfinite(T_camera2base(:)));
if transform_ok&&kf_ready
    velocity_C=T_camera2base(1:3,1:3)'*[x(3:4);0];
    if all(isfinite(velocity_C))
        kf_velocity_C_for_feedforward=velocity_C(1:2);
    end
end

if ~current_visual_valid || ~controller_enable || ~initialized
    v_previous=zeros(2,1);
    ff_ramp=0;
    ff_ramp_out=0;
    return
end

if enable_p
    v_p=Kp.*position_C_hold;
end
if enable_ff&&kf_ready&&transform_ok
    ff_ramp=min(1,ff_ramp+Ts/ff_ramp_time);
    v_ff=ff_ramp*k_ff*kf_velocity_C_for_feedforward;
else
    ff_ramp=0;
end
ff_ramp_out=ff_ramp;
v_raw=v_p+v_ff;
raw_norm=norm(v_raw);
saturation_active=raw_norm>v_xy_max;
if saturation_active
    v_norm_limited=v_raw*(v_xy_max/raw_norm);
else
    v_norm_limited=v_raw;
end

delta=v_norm_limited-v_previous;
delta_norm=norm(delta);
delta_max=a_xy_max*Ts;
acceleration_active=delta_norm>delta_max;
if acceleration_active
    v_issued=v_previous+delta*(delta_max/delta_norm);
else
    v_issued=v_norm_limited;
end

controller_ok=all(isfinite([v_p;v_ff;v_raw;v_norm_limited;v_issued]));
if controller_ok
    v_previous=v_issued;
    camera_velocity=[v_issued;0;0;0;0];
else
    v_p=zeros(2,1);
    v_ff=zeros(2,1);
    v_raw=zeros(2,1);
    v_norm_limited=zeros(2,1);
    v_issued=zeros(2,1);
    camera_velocity=zeros(6,1);
    v_previous=zeros(2,1);
    ff_ramp=0;
    ff_ramp_out=0;
end
end
