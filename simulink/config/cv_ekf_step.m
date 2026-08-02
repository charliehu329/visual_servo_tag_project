function [x_next,P_next,innovation_nis,measurement_accepted] = ...
    cv_ekf_step(x_hat,P,z_meas,v_camera_input,measurement_available, ...
    A,B,H,Q_ekf,R_ekf,gate)
%CV_EKF_STEP 四状态匀速滤波器的一步预测和可选测量更新。
% 输入位置、目标速度和相机速度均在当前相机坐标系中表达。

x_pred=A*x_hat+B*v_camera_input;
P_pred=A*P*A'+Q_ekf;
P_pred=.5*(P_pred+P_pred');
x_next=x_pred; P_next=P_pred;
innovation_nis=NaN; measurement_accepted=false;
if ~measurement_available, return; end

innovation=z_meas-H*x_pred;
S=H*P_pred*H'+R_ekf;
if ~all(isfinite(S(:))) || rcond(S)<=1e-12
    innovation_nis=inf;
    return
end
innovation_nis=innovation'*(S\innovation);
if ~isfinite(innovation_nis) || innovation_nis>gate, return; end

K=(P_pred*H')/S;
x_next=x_pred+K*innovation;
I4=eye(4); A_joseph=I4-K*H;
P_next=A_joseph*P_pred*A_joseph'+K*R_ekf*K';
P_next=.5*(P_next+P_next');
measurement_accepted=true;
end

