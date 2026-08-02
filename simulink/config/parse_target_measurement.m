function [pv,vv,p,v,t,stamp_ok] = parse_target_measurement( ...
    data,count,age,timeout)
pv=false;
vv=false;
p=zeros(2,1);
v=zeros(2,1);
t=0;
stamp_ok=false;

ok=numel(data)>=8 && isfinite(count) && count==8 && ...
    isfinite(age) && age>=0 && isfinite(timeout) && ...
    timeout>=0 && age<=timeout && all(isfinite(data(1:8)));
if ~ok
    return
end

sec_ok=data(7)>=0 && data(7)==floor(data(7));
nsec_ok=data(8)>=0 && data(8)<1e9 && data(8)==floor(data(8));
if ~(sec_ok&&nsec_ok)
    return
end

t=double(data(7))+1e-9*double(data(8));
stamp_ok=isfinite(t)&&t>0;
if ~stamp_ok
    return
end

pv=data(1)>.5;
if ~pv
    return
end
p=reshape(double(data(3:4)),2,1);
vv=data(2)>.5;
if vv
    v=reshape(double(data(5:6)),2,1);
end
end
