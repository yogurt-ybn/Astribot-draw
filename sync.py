"""Synchronized actuator execution, with explicit geometric collision gate."""
from pathlib import Path
import os,sys,json,math,time
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT))
from kinematics import *
import light_painting as lp
from scipy.interpolate import CubicSpline
from collision_model import build_collision_scene, CollisionChecker, collision_pairs

SIDES=['left','right']

def schedule(m,plans,torso):
 dt=float(m.opt.timestep);home=np.zeros(m.nq);home[:3]=[.05,-.40,.35]
 # Avoid initial arm elbow exactly at the position limit.
 for side in SIDES:home[lp.arm_indices(m,side)[1][3]]=.03
 qs=[];dqs=[];phases=[];desired=[];qcur=home.copy()
 def transfer(goal,phase):
  nonlocal qcur
  T=max(7.,np.max(abs(goal-qcur))*1.875/.20);n=math.ceil(T/dt);r=np.arange(n)/n;s=10*r**3-15*r**4+6*r**5;ds=(30*r**2-60*r**3+30*r**4)/(n*dt)
  qs.extend(qcur+s[:,None]*(goal-qcur));dqs.extend(ds[:,None]*(goal-qcur));phases.extend([phase]*n);desired.extend([np.full((2,3),np.nan)]*n);qcur=goal.copy()
 goal=home.copy();goal[:4]=torso[0]
 for si,side in enumerate(SIDES):goal[lp.arm_indices(m,side)[1]]=plans[si][1][0]
 transfer(goal,'approach_both')
 spl=[];pspl=[];T=15.
 for si,side in enumerate(SIDES):
  p,q=plans[si];u=np.r_[0,np.cumsum(np.linalg.norm(np.diff(p,axis=0),axis=1))];length=u[-1];u=np.linspace(0,1,len(p));s=CubicSpline(u,q);p_s=CubicSpline(u,p);spl.append(s);pspl.append(p_s);T=max(T,1.875*length/.06,1.875*abs(s(np.linspace(0,1,3001),1)).max()/.20)
 n=math.ceil(T/dt);r=np.arange(n)/n;s=10*r**3-15*r**4+6*r**5;ds=(30*r**2-60*r**3+30*r**4)/(n*dt);qdraw=np.tile(qcur,(n,1));dqdraw=np.zeros_like(qdraw);ts=CubicSpline(np.linspace(0,1,len(torso)),torso);qdraw[:,:4]=ts(s);dqdraw[:,:4]=ts(s,1)*ds[:,None]
 for si,side in enumerate(SIDES):
  qi=lp.arm_indices(m,side)[1];qdraw[:,qi]=spl[si](s);dqdraw[:,qi]=spl[si](s,1)*ds[:,None];qcur[qi]=plans[si][1][-1]
 qcur[:4]=torso[-1];qs.extend(qdraw);dqs.extend(dqdraw);phases.extend(['draw_both']*n);desired.extend(np.stack([p(s) for p in pspl],axis=1));transfer(home,'return_both')
 return home,np.array(qs),np.array(dqs),np.array(phases),np.array(desired)

def solve(c,p,q):
 q=q.copy();q[4:]=np.clip(q[4:],-.4,.4)
 for _ in range(160):
  xyz,_,J=c.fk(q,jacobian=True);err=p-xyz[0]
  if np.linalg.norm(err)<3e-5:break
  j=J[0];ji=j.T@np.linalg.inv(j@j.T+np.eye(3)*1e-5);post=np.r_[np.zeros(4),-.15*q[4:]];dq=ji@err+(np.eye(7)-ji@j)@post;q=np.clip(q+np.clip(dq,-.05,.05),c.lo,c.hi);q[4:]=np.clip(q[4:],-.4,.4)
 return q,float(np.linalg.norm(c.fk(q)[0][0]-p))

def compensate_torso(m,plans):
 N=801;s=np.linspace(0,1,N);torso=np.zeros((N,4))
 d=mujoco.MjData(m);chains=[Chain(m,side,margin=.08) for side in SIDES];targets=[];solved=[];rng=np.random.default_rng(44)
 for p,prior in plans:
  u=np.r_[0,np.cumsum(np.linalg.norm(np.diff(p,axis=0),axis=1))];u/=u[-1];targets.append(CubicSpline(u,p)(s))
 z=(targets[0][:,2]+targets[1][:,2])/2;amplitude=float(os.environ.get('TORSO_LEAN_AMPLITUDE', str({'tidal_waves':.65,'rounded_leaves':.85,'twin_galaxies':.75}[sys.argv[1]])));b=-.40-amplitude*np.exp(-((z-1.20)/.45)**4);torso[:]=np.c_[np.full(N,.05),b,-.05-b,np.zeros(N)]
 allpairs=collision_pairs(m)
 for si,c in enumerate(chains):
  side=SIDES[si];pairs=[(x,y) for x,y,cat in allpairs if cat not in ['inter_arm','torso_head_base'] and (side in m.body(int(m.geom_bodyid[x])).name or side in m.body(int(m.geom_bodyid[y])).name)]
  poses=[]
  for k in range(N):
   d.qpos[:4]=torso[k];mujoco.mj_forward(m,d);parent=m.body_parentid[c.bids[0]];poses.append((d.xpos[parent].copy(),d.xmat[parent].reshape(3,3).copy()))
  seeds=[plans[si][1][0],np.clip(np.zeros(7),c.lo,c.hi)]
  grid=np.load(ROOT/'inputs/canvas-ik-grid.npz');good=grid[side+'_reachable'].ravel();wp=grid['world_points'][good];gq=grid[side+'_q_rad'].reshape(-1,7)[good];inds=np.argsort(np.linalg.norm(wp-targets[si][0],axis=1))[:80];seeds.extend(gq[inds[::4]])
  seeds.extend(rng.uniform(c.lo,c.hi,(80,7)))
  accepted=None
  for attempt,seed in enumerate(seeds):
   q=np.clip(seed,c.lo,c.hi);qq=[];minimum=.05;bad=False
   for k in range(N):
    c.p0,c.R0=poses[k];q,e=solve(c,targets[si][k],q)
    if e>.0002:bad=True;break
    qq.append(q.copy())
    if k%5==0 or k==N-1:
     def distances(z,selected):
      d.qpos[:]=0;d.qpos[:4]=torso[k];d.qpos[c.qids]=z;mujoco.mj_forward(m,d)
      return np.array([float(mujoco.mj_geomDistance(m,d,x,y,.05,None)) for x,y in selected])
     ds=distances(q,pairs);md=float(ds.min())
     minimum=min(minimum,md)
     if md<.002 or e>.0002:bad=True;break
   print('SEED',side,attempt,'points',len(qq),'minimum',minimum,'error',e, 'pair',[(m.body(int(m.geom_bodyid[x])).name,m.body(int(m.geom_bodyid[y])).name) for x,y in [pairs[int(np.argmin(ds))]]] if len(qq) else [],flush=True)
   if not bad:
    accepted=np.array(qq);break
  assert accepted is not None, f'No collision-free branch for {side}'
  solved.append((targets[si],accepted))
 return solved,torso

def main():
 name=sys.argv[1];out=ROOT/'results'/name;out.mkdir(parents=True,exist_ok=True)
 assets=Path(os.environ.get('ASTRIBOT_ASSETS',str(ROOT/'descriptions' if (ROOT/'descriptions').exists() else DEFAULT_ASSETS)));m=build_collision_scene(assets,out);mujoco.mj_saveModel(m,str(out/'scene.mjb'),None);checker=CollisionChecker(m);d=mujoco.MjData(m)
 prior=np.load(ROOT/'inputs'/name/'seed-plans.npz');design=json.loads((ROOT/'inputs'/name/'paths.json').read_text());assert design['execution']['simultaneous'] is True;strokes={x['arm']:np.array(x['points'],dtype=float) for x in design['strokes']};plans=[]
 for side in SIDES:
  uv=strokes[side];world=np.c_[(uv[:,0]-285)*0.003328153882611492,np.full(len(uv),-.2),.95-(uv[:,1]-360)*0.003328153882611492];assert np.max(abs(world-prior[side+'_world']))<1e-12;plans.append((world,prior[side+'_q']))
 plans,torso=compensate_torso(m,plans);home,qseq,dqseq,phase,desired=schedule(m,plans,torso)
 # Explicit planned-state collision gate, no dynamics execution on rejected plan.
 minimum=1e9;worst=None;collisions=[];start=time.time()
 for k,q in enumerate(qseq):
  if k%5 and k!=len(qseq)-1:continue
  d.qpos[:]=q;mujoco.mj_forward(m,d);rec=checker.check(d)
  if rec['distance_m']<minimum:minimum=rec['distance_m'];worst={**rec,'step':k,'time_s':k*m.opt.timestep,'phase':str(phase[k])}
  if rec['distance_m']<.002:collisions.append({**rec,'step':k,'phase':str(phase[k])})
 report={'name':name,'source_schema':'astribot-drawing-v2','torso_lean_amplitude':float(os.environ.get('TORSO_LEAN_AMPLITUDE',str({'tidal_waves':.65,'rounded_leaves':.85,'twin_galaxies':.75}[name]))),'engineering_plan_clearance_gate_m':.002,'contact_margin_m':.001,'distance_cap_m':.05,'collision_pair_count':len(checker.pairs),'name':name,'planned_steps':len(qseq),'planned_minimum':worst,'planning_sampling_s':5*m.opt.timestep,'rejected_samples':len(collisions)}
 (out/'planned-collision-check.json').write_text(json.dumps(report,indent=2));print(name,report,flush=True)
 if collisions:
  (out/'planned-rejected-samples.json').write_text(json.dumps(collisions,indent=2));return
 d=mujoco.MjData(m);d.qpos[:]=home;mujoco.mj_forward(m,d)
 joints=m.actuator_trnid[:,0];qa=m.jnt_qposadr[joints];va=m.jnt_dofadr[joints];kp=m.actuator_gainprm[:,0];kd=-m.actuator_biasprm[:,2]
 states=[];actual=[];vel=[];ctrl=[];forces=[];heights=[];bases=[];mind=1e9;worst=None;maxcon=0;category_minima={}
 for k,(q,dq) in enumerate(zip(qseq,dqseq)):
  ff=d.qfrc_bias[va]-d.qfrc_passive[va];d.ctrl[:]=np.clip(q[qa]+(kd*dq[qa]+ff)/kp,m.actuator_ctrlrange[:,0],m.actuator_ctrlrange[:,1]);mujoco.mj_step(m,d);mujoco.mj_forward(m,d)
  assert np.isfinite(d.qpos).all() and not np.any(d.warning.number)
  rec=checker.check(d)
  if rec['distance_m']<mind:mind=rec['distance_m'];worst={**rec,'step':k,'time_s':float(d.time),'phase':str(phase[k])}
  assert rec['distance_m']>=0, f'Actual collision at {k}: {rec}'
  heights.append(float(d.body('astribot_torso_link_4').xpos[2]));bases.append(d.body('chassis_base').xpos.copy());
  for cat,rr in rec['category_minima'].items():
   if cat not in category_minima or rr['distance_m']<category_minima[cat]['distance_m']:category_minima[cat]={**rr,'time_s':float(d.time),'phase':str(phase[k])}
  actual.append([d.site(s+'_light').xpos.copy() for s in SIDES]);states.append(d.qpos.copy());vel.append(d.qvel.copy());ctrl.append(d.ctrl.copy());forces.append(d.actuator_force.copy());maxcon=max(maxcon,d.ncon)
 states=np.array(states);actual=np.array(actual);vel=np.array(vel);forces=np.array(forces);ix=phase=='draw_both';stats={}
 for si,s in enumerate(SIDES):
  c=Chain(m,s);err=np.linalg.norm(actual[ix,si]-desired[ix,si],axis=1);margin=np.minimum(states[:,c.qids]-c.limits[:,0],c.limits[:,1]-states[:,c.qids]);stats[s]={'tracking_rms_m':float(np.sqrt(np.mean(err**2))),'tracking_max_m':float(err.max()),'minimum_joint_margin_rad':float(margin.min()),'draw_minimum_joint_margin_rad':float(margin[ix].min()),'max_joint_speed_rad_s':float(abs(vel[:,m.jnt_dofadr[c.jids]]).max())}
 tj=np.array([m.joint(f'astribot_torso_joint_{i}').id for i in range(1,5)]);tlim=m.jnt_range[tj];tstates=states[:,m.jnt_qposadr[tj]];tmarg=np.minimum(tstates-tlim[:,0],tlim[:,1]-tstates);report.update(torso_control='Prescribed shared sagittal fold trajectory with both arm IK compensating fixed world targets; posture-guided, not fully optimized whole-body control',torso_joint_min_rad=tstates.min(axis=0).tolist(),torso_joint_max_rad=tstates.max(axis=0).tolist(),torso_minimum_joint_margin_rad=float(tmarg.min()),torso_height_range_m=[min(heights),max(heights)],base_translation_max_m=float(np.linalg.norm(np.array(bases)-bases[0],axis=1).max()),category_minima=category_minima,arms=stats,actual_minimum=worst,actual_sampling_s=float(m.opt.timestep),simulation_seconds=float(d.time),steps=len(states),max_contacts=maxcon,warning_counts=d.warning.number.tolist(),draw_start_step=int(np.where(ix)[0][0]),draw_end_step=int(np.where(ix)[0][-1]),force_limit_fraction=float((abs(forces)/m.actuator_forcerange[:,1]).max()))
 (out/'results.json').write_text(json.dumps(report,indent=2));np.savez_compressed(out/'execution.npz',qpos=states,actual=actual,desired=desired,phase=phase,dt=m.opt.timestep,ctrl=np.array(ctrl),qvel=vel,qdes=qseq)
 print('DONE',name,report,flush=True)
if __name__=='__main__':main()
