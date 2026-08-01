function [pC,vC,q,qdot,tv,t0,t1,alpha,span,wait_time, ...
    sync_valid,is_new,pvalid,vvalid,drop_reason,joint_count_out, ...
    pending_count_out,joint_drop_count,visual_drop_count,reset_count, ...
    reset_required] = joint_visual_time_aligner_step( ...
    p_in,v_in,pv_in,vv_in,tv_in,tv_ok,visual_new,q_in,qdot_in, ...
    joint_count,joint_age,tj_in,tj_ok,joint_new,sim_time, ...
    joint_capacity,pending_capacity,max_span,max_wait,reset_threshold, ...
    allow_extrapolation,sync_required,joint_timeout)
% 在视觉源时刻包围插值固定容量 JointState 缓冲。
persistent jt jq jqd jhead jcount
persistent pt pp pv pvv ppv pvvv parr pcount
persistent hold_p hold_v hold_q hold_qd hold_tv hold_t0 hold_t1
persistent hold_alpha hold_span hold_pv hold_vv hold_sync
persistent joint_drops visual_drops resets initialized

MAX_JOINT=512;
MAX_PENDING=16;
if isempty(initialized)
    jt=zeros(MAX_JOINT,1);
    jq=zeros(7,MAX_JOINT);
    jqd=zeros(7,MAX_JOINT);
    jhead=1;
    jcount=0;
    pt=zeros(MAX_PENDING,1);
    pp=zeros(2,MAX_PENDING);
    pv=zeros(2,MAX_PENDING);
    pvv=false(MAX_PENDING,1);
    ppv=false(MAX_PENDING,1);
    pvvv=false(MAX_PENDING,1);
    parr=zeros(MAX_PENDING,1);
    pcount=0;
    hold_p=zeros(2,1);
    hold_v=zeros(2,1);
    hold_q=zeros(7,1);
    hold_qd=zeros(7,1);
    hold_tv=0;
    hold_t0=0;
    hold_t1=0;
    hold_alpha=0;
    hold_span=0;
    hold_pv=false;
    hold_vv=false;
    hold_sync=false;
    joint_drops=0;
    visual_drops=0;
    resets=0;
    initialized=true;
end

pC=hold_p;
vC=hold_v;
q=hold_q;
qdot=hold_qd;
tv=hold_tv;
t0=hold_t0;
t1=hold_t1;
alpha=hold_alpha;
span=hold_span;
wait_time=0;
sync_valid=hold_sync;
is_new=false;
pvalid=hold_sync&&hold_pv&&(pv_in>.5);
vvalid=pvalid&&hold_vv;
drop_reason=0;
reset_required=false;

parameter_ok=isfinite(joint_capacity)&&joint_capacity>=4&& ...
    joint_capacity<=MAX_JOINT&&joint_capacity==floor(joint_capacity)&& ...
    isfinite(pending_capacity)&&pending_capacity>=1&& ...
    pending_capacity<=MAX_PENDING&&pending_capacity==floor(pending_capacity)&& ...
    isfinite(max_span)&&max_span>0&&isfinite(max_wait)&&max_wait>0&& ...
    isfinite(reset_threshold)&&reset_threshold>0&& ...
    allow_extrapolation==0&&sync_required>.5&& ...
    isfinite(joint_timeout)&&joint_timeout>0;
if ~parameter_ok
    hold_sync=false;
    sync_valid=false;
    pvalid=false;
    vvalid=false;
    drop_reason=7;
    joint_count_out=jcount;
    pending_count_out=pcount;
    joint_drop_count=joint_drops;
    visual_drop_count=visual_drops;
    reset_count=resets;
    return
end

jcap=floor(joint_capacity);
pcap=floor(pending_capacity);
joint_ok=joint_new>.5&&tj_ok>.5&&isfinite(tj_in)&&tj_in>0&& ...
    joint_count>=7&&numel(q_in)>=7&&numel(qdot_in)>=7&& ...
    all(isfinite(q_in(1:7)))&&all(isfinite(qdot_in(1:7)))&& ...
    isfinite(joint_age)&&joint_age>=0&&joint_age<=joint_timeout;
if joint_ok
    clock_reset=false;
    if jcount>0
        last_index=ring_index(jhead,jcount,jcap);
        last_stamp=jt(last_index);
        clock_reset=tj_in<last_stamp-reset_threshold;
    end
    if clock_reset
        jhead=1;
        jcount=0;
        pcount=0;
        hold_sync=false;
        resets=resets+1;
        reset_required=true;
        drop_reason=5;
    end

    accept_joint=true;
    replace_last=false;
    if jcount>0
        last_index=ring_index(jhead,jcount,jcap);
        last_stamp=jt(last_index);
        replace_last=tj_in==last_stamp;
        if tj_in<last_stamp
            accept_joint=false;
            joint_drops=joint_drops+1;
        end
    end
    if accept_joint
        if replace_last
            write_index=ring_index(jhead,jcount,jcap);
        elseif jcount<jcap
            write_index=ring_index(jhead,jcount+1,jcap);
            jcount=jcount+1;
        else
            write_index=jhead;
            jhead=mod(jhead,jcap)+1;
        end
        jt(write_index)=tj_in;
        jq(:,write_index)=reshape(q_in(1:7),7,1);
        jqd(:,write_index)=reshape(qdot_in(1:7),7,1);
    end
end

if visual_new>.5
    visual_ok=tv_ok>.5&&isfinite(tv_in)&&tv_in>0&& ...
        numel(p_in)==2&&numel(v_in)==2&& ...
        all(isfinite(p_in(:)))&&all(isfinite(v_in(:)));
    if visual_ok
        if pcount>=pcap
            [pt,pp,pv,pvv,ppv,pvvv,parr,pcount]=drop_pending( ...
                pt,pp,pv,pvv,ppv,pvvv,parr,pcount);
            visual_drops=visual_drops+1;
            drop_reason=6;
        end
        pcount=pcount+1;
        pt(pcount)=tv_in;
        pp(:,pcount)=reshape(p_in,2,1);
        pv(:,pcount)=reshape(v_in,2,1);
        ppv(pcount)=pv_in>.5;
        pvvv(pcount)=vv_in>.5;
        parr(pcount)=sim_time;
    else
        visual_drops=visual_drops+1;
        hold_sync=false;
        sync_valid=false;
        pvalid=false;
        vvalid=false;
        is_new=true;
        drop_reason=4;
    end
end

if pcount>0
    wait_time=max(0,sim_time-parr(1));
    if wait_time>max_wait
        [pt,pp,pv,pvv,ppv,pvvv,parr,pcount]=drop_pending( ...
            pt,pp,pv,pvv,ppv,pvvv,parr,pcount);
        visual_drops=visual_drops+1;
        hold_sync=false;
        sync_valid=false;
        pvalid=false;
        vvalid=false;
        is_new=true;
        drop_reason=3;
    elseif jcount>0
        oldest=jt(ring_index(jhead,1,jcap));
        newest=jt(ring_index(jhead,jcount,jcap));
        if pt(1)<oldest
            [pt,pp,pv,pvv,ppv,pvvv,parr,pcount]=drop_pending( ...
                pt,pp,pv,pvv,ppv,pvvv,parr,pcount);
            visual_drops=visual_drops+1;
            hold_sync=false;
            sync_valid=false;
            pvalid=false;
            vvalid=false;
            is_new=true;
            drop_reason=1;
        elseif pt(1)<=newest
            [found,i0,i1,a,s]=find_bracket(pt(1),jt,jhead,jcount,jcap);
            if found&&s<=max_span
                idx0=ring_index(jhead,i0,jcap);
                idx1=ring_index(jhead,i1,jcap);
                q_new=(1-a)*jq(:,idx0)+a*jq(:,idx1);
                qd_new=(1-a)*jqd(:,idx0)+a*jqd(:,idx1);
                finite_ok=all(isfinite(q_new))&&all(isfinite(qd_new));
                if finite_ok
                    hold_p=pp(:,1);
                    hold_v=pv(:,1);
                    hold_q=q_new;
                    hold_qd=qd_new;
                    hold_tv=pt(1);
                    hold_t0=jt(idx0);
                    hold_t1=jt(idx1);
                    hold_alpha=a;
                    hold_span=s;
                    hold_pv=ppv(1);
                    hold_vv=pvvv(1);
                    hold_sync=true;
                    pC=hold_p;
                    vC=hold_v;
                    q=hold_q;
                    qdot=hold_qd;
                    tv=hold_tv;
                    t0=hold_t0;
                    t1=hold_t1;
                    alpha=hold_alpha;
                    span=hold_span;
                    sync_valid=true;
                    pvalid=hold_pv;
                    vvalid=hold_pv&&hold_vv;
                    is_new=true;
                    [pt,pp,pv,pvv,ppv,pvvv,parr,pcount]=drop_pending( ...
                        pt,pp,pv,pvv,ppv,pvvv,parr,pcount);
                else
                    hold_sync=false;
                    sync_valid=false;
                    pvalid=false;
                    vvalid=false;
                    is_new=true;
                    drop_reason=8;
                    visual_drops=visual_drops+1;
                    [pt,pp,pv,pvv,ppv,pvvv,parr,pcount]=drop_pending( ...
                        pt,pp,pv,pvv,ppv,pvvv,parr,pcount);
                end
            elseif found
                hold_sync=false;
                sync_valid=false;
                pvalid=false;
                vvalid=false;
                is_new=true;
                drop_reason=2;
                visual_drops=visual_drops+1;
                [pt,pp,pv,pvv,ppv,pvvv,parr,pcount]=drop_pending( ...
                    pt,pp,pv,pvv,ppv,pvvv,parr,pcount);
            end
        end
    end
end

joint_count_out=jcount;
pending_count_out=pcount;
joint_drop_count=joint_drops;
visual_drop_count=visual_drops;
reset_count=resets;
end

function index=ring_index(head,order,capacity)
index=mod(head+order-2,capacity)+1;
end

function [found,i0,i1,alpha,span]=find_bracket( ...
    target,stamps,head,count,capacity)
found=false;
i0=1;
i1=1;
alpha=0;
span=0;
for k=1:count
    index=ring_index(head,k,capacity);
    if target==stamps(index)
        found=true;
        i0=k;
        i1=k;
        return
    end
end
for k=1:count-1
    index0=ring_index(head,k,capacity);
    index1=ring_index(head,k+1,capacity);
    if stamps(index0)<target&&target<stamps(index1)
        span=stamps(index1)-stamps(index0);
        alpha=(target-stamps(index0))/span;
        found=isfinite(alpha)&&alpha>=0&&alpha<=1;
        i0=k;
        i1=k+1;
        return
    end
end
end

function [pt,pp,pv,pvv,ppv,pvvv,parr,count]=drop_pending( ...
    pt,pp,pv,pvv,ppv,pvvv,parr,count)
if count<=0
    return
end
for k=1:count-1
    pt(k)=pt(k+1);
    pp(:,k)=pp(:,k+1);
    pv(:,k)=pv(:,k+1);
    ppv(k)=ppv(k+1);
    pvvv(k)=pvvv(k+1);
    parr(k)=parr(k+1);
end
pt(count)=0;
pp(:,count)=0;
pv(:,count)=0;
ppv(count)=false;
pvvv(count)=false;
parr(count)=0;
count=count-1;
end
