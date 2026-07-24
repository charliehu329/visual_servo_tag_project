function [qDotCmd,centerError,depthError,rcondJc,rcondArho,qDotCenter,qDotDepthRaw,qDotDepthWeighted,qDotNullUsed] = original_stage1_controller(q,cLMeasured,rhoHatL,vHatLeft,JL,cLd,rhoD,validLeft,ekfPredictionValid,centerEnable,depthEnable,armEnable,depthTaskWeight, cfg_numericalEpsilon, cfg_robustEnable, cfg_betaC, cfg_epsilonC, cfg_Kc, cfg_lambdaC, cfg_betaRho, cfg_epsilonRho, cfg_kRho, cfg_lambdaRho, cfg_nullspaceEnable, cfg_kNull, cfg_qMid, cfg_leftOnlyCenterControlEnable)
%#codegen
% Damped approximate priority: left normalized center primary, inverse depth secondary.
qDotCmd=zeros(7,1); centerError=zeros(2,1); depthError=0; rcondJc=0; rcondArho=0; qDotCenter=zeros(7,1); qDotDepthRaw=zeros(7,1); qDotDepthWeighted=zeros(7,1); qDotNullUsed=zeros(7,1);
x=cLMeasured(1); y=cLMeasured(2); rho=rhoD; vFeed=zeros(3,1); if ekfPredictionValid>0.5&&isfinite(rhoHatL)&&rhoHatL>0, rho=max(rhoHatL,cfg_numericalEpsilon); vFeed=vHatLeft; end
centerError=[x;y]-cLd; depthError=rho-rhoD;
Lc=[-rho 0 x*rho x*y -(1+x*x) y;0 -rho y*rho 1+y*y -x*y -x];
Hc=rho*[1 0 -x;0 1 -y]; bhatC=Hc*vFeed; Jc=Lc*JL;
rc=zeros(2,1);
if cfg_robustEnable>0.5, rc=cfg_betaC*centerError/(sqrt(centerError'*centerError)+cfg_epsilonC); end
nuC=-cfg_Kc*centerError-bhatC-rc;
Gc=Jc*Jc'+(cfg_lambdaC^2)*eye(2); JcSharp=Jc'/Gc;
qC=JcSharp*nuC; Nc=eye(7)-JcSharp*Jc; rcondJc=det(Gc)/(trace(Gc)*trace(Gc)+cfg_numericalEpsilon); qDotCenter=qC;
Lrho=[0 0 rho*rho rho*y -rho*x 0]; Hrho=[0 0 -rho*rho];
Jrho=Lrho*JL; bhatRho=Hrho*vFeed;
rr=0; if cfg_robustEnable>0.5, rr=cfg_betaRho*depthError/(abs(depthError)+cfg_epsilonRho); end
nuRho=-cfg_kRho*depthError-bhatRho-rr; A=Jrho*Nc; denom=A*A'+cfg_lambdaRho^2; rcondArho=denom/(1+denom);
w=min(max(depthTaskWeight,0),1)*double(ekfPredictionValid>0.5); qDotDepthRaw=Nc*A'*((nuRho-Jrho*qC)/denom); qDotDepthWeighted=w*qDotDepthRaw;
qDot0=zeros(7,1); if cfg_nullspaceEnable>0.5, qDot0=-cfg_kNull*(q-cfg_qMid); end
ArhoDagger=A'/denom; NcRho=Nc*(eye(7)-ArhoDagger*A); Nnull=(1-w)*Nc+w*NcRho; qDotNullUsed=Nnull*qDot0;
if centerEnable>0.5, qDotCmd=qDotCenter+qDotNullUsed; end
if (centerEnable>0.5)&&(depthEnable>0.5), qDotCmd=qDotCenter+qDotDepthWeighted+qDotNullUsed; end
if (centerEnable<=0.5)&&(depthEnable>0.5), d=Jrho*Jrho'+cfg_lambdaRho^2; qDotDepthRaw=Jrho'*(nuRho/d); qDotDepthWeighted=w*qDotDepthRaw; qDotCenter=zeros(7,1); qDotNullUsed=zeros(7,1); qDotCmd=qDotDepthWeighted; end
qDotCmd=qDotCmd+0*q;
centerAllowed=validLeft>0.5&&cfg_leftOnlyCenterControlEnable>0.5; if (~centerAllowed)||(armEnable<=0.5), qDotCmd=zeros(7,1); qDotCenter=zeros(7,1); qDotDepthWeighted=zeros(7,1); qDotNullUsed=zeros(7,1); end
if any(~isfinite(qDotCmd))
    qDotCmd=zeros(7,1);
end
end