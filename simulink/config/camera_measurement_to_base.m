function [pB,vB,pok,vok] = camera_measurement_to_base( ...
    pC,vrelC,Z,T,twist,pv,vv,motion_ok)
pB=zeros(2,1);
vB=zeros(2,1);
pok=false;
vok=false;

ok=pv>.5 && numel(pC)==2 && all(isfinite(pC(:))) && ...
    isfinite(Z) && Z>0 && isequal(size(T),[4 4]) && ...
    all(isfinite(T(:)));
if ~ok
    return
end

R=T(1:3,1:3);
r=R*[reshape(pC,2,1);Z];
p=r+T(1:3,4);
if ~all(isfinite(p))
    return
end

pB=p(1:2);
pok=true;

ok=vv>.5 && motion_ok>.5 && numel(vrelC)==2 && ...
    all(isfinite(vrelC(:))) && numel(twist)==6 && ...
    all(isfinite(twist(:)));
if ~ok
    return
end

omega=reshape(twist(1:3),3,1);
vc=reshape(twist(4:6),3,1);
vt=vc+R*[reshape(vrelC,2,1);0]+cross(omega,r);
if all(isfinite(vt))
    vB=vt(1:2);
    vok=true;
end
end
