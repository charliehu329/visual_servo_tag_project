function [x,P,yp,yv,np,nv,ap,av] = base_cv_kf_step( ...
    x,P,zp,zv,usep,usev,A,Q,Rp,Rv,gp,gv)
Hp=[1 0 0 0;0 1 0 0];
Hv=[0 0 1 0;0 0 0 1];

x=A*x;
P=A*P*A'+Q;
P=(P+P')/2;

[x,P,yp,np,ap]=update2(x,P,zp,usep,Hp,Rp,gp);
[x,P,yv,nv,av]=update2(x,P,zv,usev,Hv,Rv,gv);
end

function [x,P,y,nis,accepted] = update2(x,P,z,use,H,R,gate)
y=zeros(2,1);
nis=NaN;
accepted=false;

if ~(use>.5 && numel(z)==2 && all(isfinite(z(:))))
    return
end

y=reshape(z,2,1)-H*x;
S=H*P*H'+R;
if ~all(isfinite(S(:))) || rcond(S)<=1e-12
    return
end

nis=y'*(S\y);
if ~isfinite(nis) || nis>gate
    return
end

K=(P*H')/S;
x=x+K*y;
M=eye(4)-K*H;
P=M*P*M'+K*R*K';
P=(P+P')/2;
accepted=true;
end
