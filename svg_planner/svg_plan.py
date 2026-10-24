"""Strict SVG -> simultaneous T1 trajectories. Never sends hardware commands."""
from pathlib import Path
import argparse, json, sys, time, hashlib
import numpy as np
from scipy.interpolate import CubicSpline
from svg_input import parse_svg, validate_masks
from t1_model import build_t1_scene, CollisionChecker, collision_pairs
from t1_kinematics import Chain, mujoco
from planning_primitives import solve, rrt_transfer
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parent
SIDES=('left','right')

class PlanningError(RuntimeError): pass

def validate_calibration(canvas):
    cfg=json.loads((canvas/'calibration.json').read_text())
    if cfg.get('robot')!='Astribot T1' or cfg.get('width')!=616 or cfg.get('height')!=560:
        raise PlanningError('Calibration must be the supported Astribot T1 616x560 canvas')
    if not np.isclose(cfg.get('meters_per_pixel',0),.0025,atol=1e-12,rtol=0) or not np.isclose(cfg.get('drawing_plane_world_y_m',0),-.42,atol=1e-12,rtol=0):
        raise PlanningError('Unsupported canvas scale or drawing plane')
    if not np.allclose(cfg['world_bounds_m']['x'],[-.77,.77],atol=1e-12,rtol=0) or not np.allclose(cfg['world_bounds_m']['z'],[.49,1.89],atol=1e-12,rtol=0):
        raise PlanningError('Unsupported canvas world bounds')
    return cfg

def shared_targets(data):
    # Merge only numerically coincident normalized progress knots (not geometry).
    # Mirror-image SVG decimal roundoff otherwise makes cubic IK splines ill-conditioned.
    raw=np.unique(np.concatenate([data['arms'][s]['progress'] for s in SIDES]))
    keep=[raw[0]]
    for t in raw[1:]:
        if t-keep[-1]>1e-12:keep.append(t)
    if 1-keep[-1]<=1e-12:keep[-1]=1.
    else:keep.append(1.)
    u=np.array(keep)
    p=[]
    for side in SIDES:
        a=data['arms'][side];uv=np.asarray(a['points_uv']);v=np.asarray(a['progress'])
        xy=np.column_stack([np.interp(u,v,uv[:,j]) for j in range(2)])
        p.append(np.c_[-.77+.0025*xy[:,0],np.full(len(u),-.42),1.89-.0025*xy[:,1]])
    data['metadata']['progress_knot_merge_tolerance']=1e-12
    data['metadata']['progress_knot_merge_max_path_displacement_m']=max(a['length_m'] for a in data['arms'].values())*1e-12
    return u,p

def torso_candidates(u,targets):
    z=np.mean([p[:,2] for p in targets],axis=0)
    a=np.clip(.08+.50*(1.60-z),.06,.5)
    yield 'height_coupled', np.c_[a,-2*a,a,np.zeros(len(u))]
    for bend in (.45,.35,.55):
        b=-bend-.12*np.sin(np.pi*u)**2
        yield 'bend_'+str(bend),np.c_[np.full(len(u),.04),b,-.04-b,np.zeros(len(u))]

def ik_candidate(m,u,targets,torso,seed_count,initial_qpos=None):
    d=mujoco.MjData(m);cs=[Chain(m,s,margin=.04) for s in SIDES]
    if initial_qpos is None:
        home=np.zeros(m.nq)
        for j in np.flatnonzero(m.jnt_limited):
            home[m.jnt_qposadr[j]]=np.clip(0.,m.jnt_range[j,0]+.001,m.jnt_range[j,1]-.001)
        for c in cs:home[c.qids]=np.clip(np.zeros(7),c.lo,c.hi)
    else:
        home=np.array(initial_qpos,dtype=float,copy=True)
        if home.shape!=(m.nq,) or not np.isfinite(home).all():
            raise PlanningError('Measured initial_qpos must be a finite full model pose')
        limited=np.flatnonzero(m.jnt_limited);values=home[m.jnt_qposadr[limited]]
        if np.any(values<m.jnt_range[limited,0]) or np.any(values>m.jnt_range[limited,1]):
            raise PlanningError('Measured initial pose exceeds model limits')
        for c in cs:
            if np.any(home[c.qids]<c.lo-1e-9) or np.any(home[c.qids]>c.hi+1e-9):
                raise PlanningError('Measured arm pose is outside transfer planning margins')
    pairs=collision_pairs(m);rng=np.random.default_rng(36)
    seeds=[[home[c.qids]]+list(rng.uniform(c.lo,c.hi,(seed_count,7))) for c in cs]
    diagnostics={'seed_budget_per_arm':seed_count+1,'pair_combination_budget':169,'pair_attempts':[],'arm_branch_seeds':{s:[] for s in SIDES}}
    def branches(si):
        c=cs[si]
        sub=[(a,b) for a,b,cat in pairs if cat not in ('inter_arm','torso_head_base') and any(c.side in m.body(int(m.geom_bodyid[g])).name for g in (a,b))]
        poses=[]
        for t in torso:
            d.qpos[:]=home;d.qpos[:4]=t;mujoco.mj_forward(m,d);parent=m.body_parentid[c.bids[0]]
            poses.append((d.xpos[parent].copy(),d.xmat[parent].reshape(3,3).copy()))
        for seed_index,seed in enumerate(seeds[si]):
            q=seed.copy();qq=[]
            for k,p in enumerate(targets[si]):
                c.p0,c.R0=poses[k];q,e=solve(c,p,q)
                if e>.0002:break
                d.qpos[:]=home;d.qpos[:4]=torso[k];d.qpos[c.qids]=q;mujoco.mj_forward(m,d)
                if k%5==0 or k==len(u)-1:
                    if any(mujoco.mj_geomDistance(m,d,a,b,.05,None)<.002 for a,b in sub):break
                qq.append(q.copy())
            if len(qq)==len(u):
                diagnostics['arm_branch_seeds'][c.side].append(seed_index)
                print('IK_ARM_BRANCH',c.side,seed_index,flush=True)
                yield seed_index,np.array(qq)
    right_iter=branches(1);right_cache=[];right_exhausted=False;checker=CollisionChecker(m)
    left_seen=False
    for li,lq in branches(0):
        left_seen=True;ri=0
        while True:
            if ri==len(right_cache):
                if right_exhausted:break
                try:right_cache.append(next(right_iter))
                except StopIteration:right_exhausted=True;break
            rseed,rq=right_cache[ri];ri+=1
            if len(diagnostics['pair_attempts'])>=169:raise PlanningError('Coupled IK pair budget exhausted (169), input unchanged')
            rejected=None
            for k in sorted(set(range(0,len(u),5))|{len(u)-1}):
                d.qpos[:]=home;d.qpos[:4]=torso[k];d.qpos[cs[0].qids]=lq[k];d.qpos[cs[1].qids]=rq[k];mujoco.mj_forward(m,d)
                checked=checker.check(d);threshold=.003 if k==0 else .002
                if checked['distance_m']<threshold:
                    rejected={'knot':k,'progress':float(u[k]),'threshold_m':threshold,'distance_m':checked['distance_m'],'geom1':checked['geom1'],'geom2':checked['geom2']};break
            entry={'left_seed':li,'right_seed':rseed,'status':'rejected' if rejected else 'passed','failure':rejected}
            diagnostics['pair_attempts'].append(entry)
            print('IK_PAIR',entry,flush=True)
            if rejected is None:
                diagnostics['selected_pair']={'left_seed':li,'right_seed':rseed}
                return home,cs,[lq,rq],diagnostics
        if not right_cache:raise PlanningError(f'right: no continuous IK branch in {seed_count+1} deterministic seeds')
    if not left_seen:raise PlanningError(f'left: no continuous IK branch in {seed_count+1} deterministic seeds')
    raise PlanningError('No collision-checked coupled IK branch combination within deterministic seed/pair budgets; see IK_PAIR log')

def spline_derivative_bound(spline):
    """Exact componentwise |dq/du| maximum for piecewise cubic splines."""
    a,b,c=spline.c[:3]
    h=np.diff(spline.x).reshape((-1,)+(1,)*(a.ndim-1))
    end=3*a*h*h+2*b*h+c
    vertex=np.divide(-b,3*a,out=np.zeros_like(a),where=a!=0)
    inside=(a!=0)&(vertex>0)&(vertex<h)
    value=3*a*vertex*vertex+2*b*vertex+c
    return float(max(np.max(abs(c)),np.max(abs(end)),np.max(np.where(inside,abs(value),0))))

def progress_velocity_bound(spline):
    """Conservative |dq/dr| bound for q(smoothstep(r)), by spline interval.

    Bounds each quadratic dq/du exactly, then multiplies by the maximum
    smoothstep speed over that SAME interval. No sampled velocity maxima.
    """
    if spline.x[0]!=0 or spline.x[-1]!=1:raise PlanningError('Progress spline must span [0,1]')
    a,b,c=spline.c[:3]
    h=np.diff(spline.x).reshape((-1,)+(1,)*(a.ndim-1))
    end=3*a*h*h+2*b*h+c
    vertex=np.divide(-b,3*a,out=np.zeros_like(a),where=a!=0)
    value=3*a*vertex*vertex+2*b*vertex+c
    bounds=np.maximum(np.maximum(abs(c),abs(end)),np.where((a!=0)&(vertex>0)&(vertex<h),abs(value),0))
    lo=np.zeros(len(spline.x));hi=np.ones(len(spline.x))
    for _ in range(60):
        r=(lo+hi)/2
        z=np.minimum(r,1-r);v=z**3*(10+z*(-15+6*z));v=np.where(r<=.5,v,1-v)
        below=v<spline.x;lo=np.where(below,r,lo);hi=np.where(below,hi,r)
    left=np.maximum(0,lo[:-1]-1e-9);right=np.minimum(1,hi[1:]+1e-9)
    peak=np.clip(.5,left,right);weight=30*peak**2*(1-peak)**2
    weight=weight.reshape((-1,)+(1,)*(a.ndim-1))
    return float(np.max(bounds*weight)*(1+1e-9))

def schedule(m,u,targets,torso,home,cs,solved,*,return_home=True,timing='legacy'):
    if timing not in ('legacy','progress_bound'):raise PlanningError('Unknown timing method')
    dt=m.opt.timestep;goal=home.copy();goal[:4]=torso[0]
    for c,q in zip(cs,solved):goal[c.qids]=q[0]
    torso_joints=np.array([int(np.flatnonzero(m.jnt_qposadr==i)[0]) for i in range(4)])
    both=SimpleNamespace(qids=np.concatenate([np.arange(4)]+[c.qids for c in cs]),lo=np.concatenate([m.jnt_range[torso_joints,0]+.001]+[c.lo for c in cs]),hi=np.concatenate([m.jnt_range[torso_joints,1]-.001]+[c.hi for c in cs]),side='whole_body_18dof')
    assert len(both.qids)==18 and len(np.unique(both.qids))==18
    pairs=collision_pairs(m)
    entry=rrt_transfer(m,both,home[both.qids],goal[both.qids],torso[0],home,[(a,b) for a,b,cat in pairs],np.random.default_rng(12345))
    qs=[];dqs=[];phases=[];desired=[]
    def transfer(a,b):
        T=max(2.,np.max(abs(b-a))*1.875/.2);n=int(np.ceil(T/dt));r=np.arange(n)/n;s=10*r**3-15*r**4+6*r**5;ds=(30*r**2-60*r**3+30*r**4)/(n*dt)
        qs.extend(a+s[:,None]*(b-a));dqs.extend(ds[:,None]*(b-a));phases.extend(['approach_both']*n);desired.extend([np.full((2,3),np.nan)]*n)
    prev=home.copy()
    for qq in entry[1:]:
        nxt=home.copy();nxt[both.qids]=qq;transfer(prev,nxt);prev=nxt
    aq=np.array(qs);av=np.array(dqs)
    spl=[CubicSpline(u,q) for q in solved];ts=CubicSpline(u,torso)
    bound=(lambda s:1.875*spline_derivative_bound(s)) if timing=='legacy' else progress_velocity_bound
    T=max(2.,max(1.875*np.linalg.norm(np.diff(p,axis=0),axis=1).sum()/.05 for p in targets),max(bound(s)/.2 for s in spl),bound(ts)/.2)
    n=int(np.ceil(T/dt))
    total_steps=(2*(n+1)+2*len(qs)+1) if return_home else len(qs)+n+1
    if total_steps>200000:raise PlanningError(f'Schedule exceeds 200000-step resource budget ({n} draw steps); inspect curve conditioning/corners')
    r=np.linspace(0,1,n+1);s=10*r**3-15*r**4+6*r**5;ds=(30*r**2-60*r**3+30*r**4)/(n*dt)
    qdraw=np.tile(goal,(n+1,1));dqdraw=np.zeros_like(qdraw);qdraw[:,:4]=ts(s);dqdraw[:,:4]=ts(s,1)*ds[:,None]
    for c,ss in zip(cs,spl): qdraw[:,c.qids]=ss(s);dqdraw[:,c.qids]=ss(s,1)*ds[:,None]
    exact=np.stack([np.column_stack([np.interp(s,u,p[:,j]) for j in range(3)]) for p in targets],axis=1)
    qs.extend(qdraw);dqs.extend(dqdraw);phases.extend(['draw_both']*(n+1));desired.extend(exact)
    if return_home:
        qs.extend(qdraw[::-1]);dqs.extend(-dqdraw[::-1]);phases.extend(['return_both']*(n+1));desired.extend([np.full((2,3),np.nan)]*(n+1))
        qs.extend(aq[::-1]);dqs.extend(-av[::-1]);phases.extend(['return_both']*len(aq));desired.extend([np.full((2,3),np.nan)]*len(aq))
        qs.append(home);dqs.append(np.zeros(m.nq));phases.append('return_both');desired.append(np.full((2,3),np.nan))
    assert np.allclose(qs[0],home,atol=1e-12)
    assert np.allclose(qs[-1],home if return_home else qdraw[-1],atol=1e-12)
    assert np.allclose(dqs[-1],0.,atol=1e-12)
    print('SCHEDULE_STEPS',len(qs),'duration_s',(len(qs)-1)*dt,flush=True)
    return np.array(qs),np.array(dqs),np.array(phases),np.array(desired)

def validate_plan(m,qs,dqs,phase,desired):
    checker=CollisionChecker(m);d=mujoco.MjData(m);md=.05;worst=None;shape=0.
    if not np.isfinite(qs).all() or not np.isfinite(dqs).all():raise PlanningError('Nonfinite planned position/velocity')
    if not np.isfinite(desired[phase=='draw_both']).all():raise PlanningError('Nonfinite desired drawing target')
    limited=np.flatnonzero(m.jnt_limited);qids=m.jnt_qposadr[limited]
    margin=np.minimum(qs[:,qids]-m.jnt_range[limited,0],m.jnt_range[limited,1]-qs[:,qids]).min()
    if margin<0: raise PlanningError(f'Joint limit violation {margin}')
    # Position geometry is audited at EVERY scheduled draw timestep.
    for k in range(len(qs)):
        if k%10000==0:print('PLAN_CHECK',k,len(qs),flush=True)
        if k%10!=0 and phase[k]!='draw_both' and k!=len(qs)-1:continue
        d.qpos[:]=qs[k];mujoco.mj_forward(m,d)
        if phase[k]=='draw_both':shape=max(shape,max(np.linalg.norm(d.site(s+'_light').xpos-desired[k,si]) for si,s in enumerate(SIDES)))
        if k%10==0 or k==len(qs)-1:
            r=checker.check(d)
            if r['distance_m']<md:md=r['distance_m'];worst={**r,'time_s':k*m.opt.timestep,'phase':str(phase[k])}
            if md<.002:raise PlanningError('Whole-plan clearance below 2 mm: '+str(worst))
    if shape>.001:raise PlanningError(f'Joint interpolation changes SVG path by {shape*1000:.3f} mm (limit 1 mm); rejected, not edited')
    if np.max(abs(dqs))>.205:raise PlanningError('Interpolated joint velocity exceeds configured .2 rad/s + numerical allowance')
    return {'minimum_clearance_m':md,'worst_pair':worst,'collision_sampling_s':10*m.opt.timestep,'max_fk_vs_flattened_input_m':shape,'minimum_joint_margin_rad':float(margin),'maximum_commanded_velocity_rad_s':float(np.max(abs(dqs)))}

def simulate(m,home,qs,dqs,phase,desired,out):
    d=mujoco.MjData(m);d.qpos[:]=home;mujoco.mj_forward(m,d);checker=CollisionChecker(m)
    # Official grippers are tendon actuators, not joint transmissions. Hold their
    # preset opening; leaving ctrl=0 drives passive links onto the soft stop.
    tendon=np.where(m.actuator_trntype==mujoco.mjtTrn.mjTRN_TENDON)[0]
    for ai in tendon:
        cmd=-m.actuator_biasprm[ai,1]*d.actuator_length[ai]/m.actuator_gainprm[ai,0]
        d.ctrl[ai]=np.clip(cmd,*m.actuator_ctrlrange[ai]) if m.actuator_ctrllimited[ai] else cmd
    ja=np.where(m.actuator_trntype==mujoco.mjtTrn.mjTRN_JOINT)[0];ji=m.actuator_trnid[ja,0];qa=m.jnt_qposadr[ji];va=m.jnt_dofadr[ji];kp=m.actuator_gainprm[ja,0];kd=-m.actuator_biasprm[ja,2]
    states=[];actual=[];vel=[];md=.05;worst=None;heights=[]
    for k,(q,dq) in enumerate(zip(qs,dqs)):
        cmd=q[qa]+(kd*dq[qa]+d.qfrc_bias[va]-d.qfrc_passive[va])/kp
        lim=m.actuator_ctrllimited[ja].astype(bool);cmd[lim]=np.clip(cmd[lim],m.actuator_ctrlrange[ja[lim],0],m.actuator_ctrlrange[ja[lim],1]);d.ctrl[ja]=cmd
        mujoco.mj_step(m,d);mujoco.mj_forward(m,d)
        if np.any(d.warning.number) or not np.isfinite(d.qpos).all():raise PlanningError('Dynamic simulation warning/nonfinite state')
        if k%2==0 or k==len(qs)-1:
            r=checker.check(d)
            if r['distance_m']<md:md=r['distance_m'];worst={**r,'time_s':d.time,'phase':str(phase[k])}
            if md<0:raise PlanningError('Actual-state collision: '+str(worst))
        states.append(d.qpos.copy());vel.append(d.qvel.copy());actual.append([d.site(s+'_light').xpos.copy() for s in SIDES]);heights.append(d.body('astribot_torso_link_4').xpos[2])
        if k%5000==0:print('SIMULATION',k,len(qs),flush=True)
    states=np.array(states);actual=np.array(actual);vel=np.array(vel);draw=phase=='draw_both';stats={}
    limited=np.flatnonzero(m.jnt_limited);qi=m.jnt_qposadr[limited]
    actual_margins=np.minimum(states[:,qi]-m.jnt_range[limited,0],m.jnt_range[limited,1]-states[:,qi])
    actual_margin=float(actual_margins.min())
    if not np.isfinite(actual).all() or not np.isfinite(vel).all():raise PlanningError('Nonfinite actual kinematics')
    if actual_margin < -1e-6:
        ki,ji=np.unravel_index(actual_margins.argmin(),actual_margins.shape)
        raise PlanningError(f'Actual joint-limit violation {actual_margin} rad at {ki*m.opt.timestep}s, joint {m.joint(limited[ji]).name}')
    for si,side in enumerate(SIDES):
        err=np.linalg.norm(actual[draw,si]-desired[draw,si],axis=1);stats[side]={'rms_m':float(np.sqrt(np.mean(err**2))),'max_m':float(err.max())}
    if max(v['max_m'] for v in stats.values())>.002:raise PlanningError('Actual drawing tracking error exceeds 2 mm')
    np.savez_compressed(out/'execution.npz',qpos=states,qvel=vel,actual=actual,desired=desired,phase=phase,dt=m.opt.timestep,qdes=qs)
    return {'max_tracking_error_threshold_m':.002,'actual_minimum_joint_margin_rad':actual_margin,'maximum_actual_joint_velocity_rad_s':float(np.max(abs(vel))),'initial_qpos':home.tolist(),'final_actual_qpos':states[-1].tolist(),'maximum_final_home_error_rad':float(np.max(abs(states[-1]-home))),'duration_s':float(d.time),'minimum_clearance_m':md,'worst_pair':worst,'arms':stats,'torso_z_range_m':[min(heights),max(heights)],'collision_sampling_s':2*m.opt.timestep,'step_s':m.opt.timestep,'warning_counts':d.warning.number.tolist()}

def main(argv=None):
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('svg',type=Path);ap.add_argument('--out',type=Path,required=True);ap.add_argument('--mode',choices=['static','plan','simulate'],default='static');ap.add_argument('--canvas',type=Path,default=ROOT/'t1-canvas');ap.add_argument('--assets',type=Path);ap.add_argument('--seeds',type=int,default=12);args=ap.parse_args(argv)
    if not 0<=args.seeds<=100:ap.error('--seeds must be between 0 and 100')
    args.out.mkdir(parents=True,exist_ok=True)
    # Explicitly replace only our named generated artifacts; never expose stale success outputs.
    for filename in ('report.json','ordered-paths.json','preview.png','plan.npz','joint-trajectory.csv','execution.npz'):
        (args.out/filename).unlink(missing_ok=True)
    report={'schema':'astribot-t1-svg-plan-report-v1','input':str(args.svg),'mode':args.mode,'stages':{k:'not_run' for k in ['static','ik','full_plan','simulation']},'limitations':['Fixed chassis; identical canonical initial whole-body posture for all inputs, 18DOF approach/return is validated only when corresponding stages pass; not arbitrary measured starting states.','Finite prescribed torso candidate families, not complete whole-body optimization.','Sampled collision checks and existing compound-joint exclusions, not swept/hardware safety proof.','Position-only free-space light-tip planning; no pen orientation, surface contact or force control.','Acceleration, jerk, torque margins and singularity thresholds are not validated; actual velocity is measured but not gated; no hardware-ready claim.']}
    stage='static'
    try:
        report['calibration']=validate_calibration(args.canvas);data=parse_svg(args.svg);report['input_sha256']=hashlib.sha256(args.svg.read_bytes()).hexdigest();report['static']=validate_masks(data,args.canvas);report['stages']['static']='passed'
        (args.out/'ordered-paths.json').write_text(json.dumps(data,indent=2));report['geometry']=data.get('metadata',{})
        from PIL import Image,ImageDraw
        im=Image.new('RGB',(616,560),'white');dr=ImageDraw.Draw(im)
        for s,color in [('left','blue'),('right','orange')]:dr.line([tuple(p) for p in data['arms'][s]['points_uv']],fill=color,width=2)
        im.save(args.out/'preview.png')
        if args.mode!='static':
            m=build_t1_scene(args.out,assets=args.assets,exclude_compound_elbow=True);u,targets=shared_targets(data);failures=[];success=False
            for name,torso in torso_candidates(u,targets):
                try:
                    print('CANDIDATE',name,flush=True);stage='ik';home,cs,solved,ik_search=ik_candidate(m,u,targets,torso,args.seeds);report['ik_search']=ik_search;report['stages']['ik']='passed';stage='full_plan';qs,dqs,phase,desired=schedule(m,u,targets,torso,home,cs,solved);checked=validate_plan(m,qs,dqs,phase,desired);success=True;break
                except (PlanningError,RuntimeError) as e: report['stages'][stage]='failed';failures.append({'candidate':name,'stage':stage,'error':str(e)});print('REJECT',failures[-1],flush=True)
            report['candidate_failures']=failures
            if not success:raise PlanningError('Candidate search exhausted; input unchanged. See candidate_failures.')
            report['selected_torso_candidate']=name;report['plan']=checked;report['stages']['full_plan']='passed'
            np.savez_compressed(args.out/'plan.npz',qdes=qs,dqdes=dqs,phase=phase,desired=desired,dt=m.opt.timestep,home=home)
            names=[m.joint(i).name for i in range(m.njnt)];np.savetxt(args.out/'joint-trajectory.csv',np.c_[np.arange(len(qs))*m.opt.timestep,qs,dqs],delimiter=',',header=','.join(['time_s']+names+['velocity_'+s for s in names]),comments='')
            if args.mode=='simulate':stage='simulation';report['simulation']=simulate(m,home,qs,dqs,phase,desired,args.out);report['stages']['simulation']='passed'
        report['status']='passed_requested_mode';code=0
    except Exception as e:
        report['stages'][stage]='failed';report['status']='rejected';report['error']={'type':type(e).__name__,'message':str(e)};code=2
    (args.out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));return code
if __name__=='__main__':sys.exit(main())
