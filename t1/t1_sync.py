"""T1-specific fixed chassis, synchronized tips and shared moving torso test."""
from pathlib import Path
import os,sys,json,time
ROOT=Path(__file__).resolve().parent
from t1_kinematics import *
from scipy.interpolate import CubicSpline
from t1_model import build_t1_scene, CollisionChecker, collision_pairs

SIDES=['left','right']

def solve(c,p,q):
 q=q.copy();q[4:]=np.clip(q[4:],-.4,.4)
 for _ in range(160):
  xyz,_,J=c.fk(q,jacobian=True);err=p-xyz[0]
  if np.linalg.norm(err)<3e-5:break
  j=J[0];ji=j.T@np.linalg.inv(j@j.T+np.eye(3)*1e-5);post=np.r_[np.zeros(4),-.15*q[4:]];dq=ji@err+(np.eye(7)-ji@j)@post;q=np.clip(q+np.clip(dq,-.05,.05),c.lo,c.hi);q[4:]=np.clip(q[4:],-.4,.4)
 return q,float(np.linalg.norm(c.fk(q)[0][0]-p))


def target_paths(name,N=401):
 u=np.linspace(0,1,N);paths=[]
 for side,sgn in [('left',1),('right',-1)]:
  if name=='waves': xx=.055*np.sin(4*np.pi*u);zz=.16*(1-2*u)
  elif name=='leaves': xx=.085*np.sin(2*np.pi*u);zz=.15*np.cos(2*np.pi*u)
  else:
   r=.025+.085*u;xx=r*np.cos(3*np.pi*u);zz=r*np.sin(3*np.pi*u)
  paths.append(np.c_[sgn*(.32+xx),np.full(N,-.42),1.10+zz])
 return paths

def plan(m,name,out):
 d=mujoco.MjData(m);cs=[Chain(m,s,margin=.04) for s in SIDES];N=401;u=np.linspace(0,1,N)
 b=-float(os.getenv('T1_TORSO_BEND','.45'))-.12*np.sin(np.pi*u)**2
 torso=np.c_[np.full(N,.04),b,-.04-b,np.zeros(N)]
 home=np.zeros(m.nq);home[:4]=torso[0]
 for c in cs:home[c.qids]=np.clip(np.zeros(7),c.lo,c.hi)
 targets=target_paths(name,N);solved=[];rng=np.random.default_rng(36);pairs=collision_pairs(m)
 for si,c in enumerate(cs):
  sub=[(a,b) for a,b,cat in pairs if cat not in ('inter_arm','torso_head_base') and any(SIDES[si] in m.body(int(m.geom_bodyid[g])).name for g in (a,b))]
  poses=[]
  for t in torso:
   d.qpos[:]=home;d.qpos[:4]=t;mujoco.mj_forward(m,d);parent=m.body_parentid[c.bids[0]];poses.append((d.xpos[parent].copy(),d.xmat[parent].reshape(3,3).copy()))
  seeds=[home[c.qids]]+list(rng.uniform(c.lo,c.hi,(100,7)))
  accepted=None
  for attempt,seed in enumerate(seeds):
   q=np.clip(seed,c.lo,c.hi);qq=[];md=.05
   for k in range(N):
    c.p0,c.R0=poses[k];q,e=solve(c,targets[si][k],q)
    if e>.0002:break
    qq.append(q.copy())
    if k%5==0 or k==N-1:
     d.qpos[:]=home;d.qpos[:4]=torso[k];d.qpos[c.qids]=q;mujoco.mj_forward(m,d)
     ds=[float(mujoco.mj_geomDistance(m,d,a,b,.05,None)) for a,b in sub];md=min(md,min(ds))
     if md<.002:break
   print('SEED',name,SIDES[si],attempt,len(qq),md,e,[(m.body(int(m.geom_bodyid[a])).name,m.body(int(m.geom_bodyid[b])).name) for a,b in [sub[int(np.argmin(ds))]]] if len(qq) else [],flush=True)
   if len(qq)==N and md>=.002 and e<=.0002:accepted=np.array(qq);break
  if accepted is None:raise RuntimeError(f'No model-clear continuous T1 IK branch for {SIDES[si]}')
  solved.append(accepted)
 # In-arm seed clearance does not establish coupled clearance; full schedule screened below.
 phases=[];qs=[];dqs=[];desired=[];dt=m.opt.timestep
 goal=home.copy();goal[:4]=torso[0]
 for c,q in zip(cs,solved):goal[c.qids]=q[0]
 def transfer(a,b,label):
  T=max(6.,np.max(abs(b-a))*1.875/.2);n=int(np.ceil(T/dt));r=np.arange(n)/n;s=10*r**3-15*r**4+6*r**5;ds=(30*r**2-60*r**3+30*r**4)/(n*dt)
  qs.extend(a+s[:,None]*(b-a));dqs.extend(ds[:,None]*(b-a));phases.extend([label]*n);desired.extend([np.full((2,3),np.nan)]*n)
 transfer(home,goal,'approach_both')
 spl=[CubicSpline(u,q) for q in solved];ts=CubicSpline(u,torso);ps=[CubicSpline(u,p) for p in targets]
 T=max(25.,max(1.875*np.linalg.norm(np.diff(p,axis=0),axis=1).sum()/.05 for p in targets),max(1.875*abs(s(np.linspace(0,1,1001),1)).max()/.2 for s in spl));n=int(np.ceil(T/dt));r=np.arange(n)/n;s=10*r**3-15*r**4+6*r**5;ds=(30*r**2-60*r**3+30*r**4)/(n*dt)
 qdraw=np.tile(goal,(n,1));dqdraw=np.zeros_like(qdraw);qdraw[:,:4]=ts(s);dqdraw[:,:4]=ts(s,1)*ds[:,None]
 for c,ss in zip(cs,spl):qdraw[:,c.qids]=ss(s);dqdraw[:,c.qids]=ss(s,1)*ds[:,None]
 qs.extend(qdraw);dqs.extend(dqdraw);phases.extend(['draw_both']*n);desired.extend(np.stack([p(s) for p in ps],axis=1));end=goal.copy();end[:4]=torso[-1]
 for c,q in zip(cs,solved):end[c.qids]=q[-1]
 transfer(end,home,'return_both')
 return home,np.array(qs),np.array(dqs),np.array(phases),np.array(desired)

def main():
 name=sys.argv[1] if len(sys.argv)>1 else 'waves';out=ROOT/'t1-results'/name;out.mkdir(exist_ok=True,parents=True)
 m=build_t1_scene(out,exclude_compound_elbow=True);d=mujoco.MjData(m);checker=CollisionChecker(m);home,qs,dqs,phase,desired=plan(m,name,out);worst=None;md=.05
 for k in range(0,len(qs),10):
  d.qpos[:]=qs[k];mujoco.mj_forward(m,d);r=checker.check(d)
  if r['distance_m']<md:md=r['distance_m'];worst={**r,'time_s':k*m.opt.timestep,'phase':str(phase[k])}
  if r['distance_m']<.002:raise RuntimeError('Full scheduled model collision '+str(worst))
 planned_minimum=md;print('PLANNED_PASS',name,md,flush=True)
 np.savez_compressed(out/'plan.npz',qdes=qs,phase=phase,desired=desired)
 d=mujoco.MjData(m);d.qpos[:]=home;mujoco.mj_forward(m,d)
 jacts=np.where(m.actuator_trntype==mujoco.mjtTrn.mjTRN_JOINT)[0];jids=m.actuator_trnid[jacts,0];qa=m.jnt_qposadr[jids];va=m.jnt_dofadr[jids];kp=m.actuator_gainprm[jacts,0];kd=-m.actuator_biasprm[jacts,2]
 states=[];actual=[];vel=[];heights=[];md=.05;worst=None;maxcon=0
 for k,(q,dq) in enumerate(zip(qs,dqs)):
  ff=d.qfrc_bias[va]-d.qfrc_passive[va];cmd=q[qa]+(kd*dq[qa]+ff)/kp;limited=m.actuator_ctrllimited[jacts].astype(bool);cmd[limited]=np.clip(cmd[limited],m.actuator_ctrlrange[jacts[limited],0],m.actuator_ctrlrange[jacts[limited],1]);d.ctrl[jacts]=cmd
  mujoco.mj_step(m,d);mujoco.mj_forward(m,d)
  if np.any(d.warning.number) or not np.isfinite(d.qpos).all():raise RuntimeError('Simulation warning/nonfinite')
  if k%2==0 or k==len(qs)-1:
   r=checker.check(d)
   if r['distance_m']<md:md=r['distance_m'];worst={**r,'time_s':d.time,'phase':str(phase[k])}
   if r['distance_m']<0:raise RuntimeError('Actual model collision '+str(worst))
  states.append(d.qpos.copy());vel.append(d.qvel.copy());actual.append([d.site(s+'_light').xpos.copy() for s in SIDES]);heights.append(d.body('astribot_torso_link_4').xpos[2]);maxcon=max(maxcon,d.ncon)
 states=np.array(states);actual=np.array(actual);vel=np.array(vel);draw=phase=='draw_both';stats={}
 for si,side in enumerate(SIDES):
  c=Chain(m,side);err=np.linalg.norm(actual[draw,si]-desired[draw,si],axis=1);stats[side]={'rms_m':float(np.sqrt(np.mean(err**2))),'max_m':float(err.max()),'min_joint_margin_rad':float(np.minimum(states[:,c.qids]-c.limits[:,0],c.limits[:,1]-states[:,c.qids]).min())}
 report={'robot':'Astribot T1','name':name,'source_commit':'cab872dc00081d2c5b096c7a19e563050eb7a2ff','simulation_seconds':float(d.time),'step_s':float(m.opt.timestep),'actual_collision_sample_s':2*float(m.opt.timestep),'planned_minimum_m':planned_minimum,'actual_minimum':worst,'arms':stats,'torso_z_min_max_m':[float(min(heights)),float(max(heights))],'max_contacts':maxcon,'warning_counts':d.warning.number.tolist(),'base':'structurally fixed; wheel joints removed','scope':'new smaller T1 test curves; S1 reachable mask not transferred','method':'prescribed shared moving torso + compensated simultaneous arms; not full-body optimizer'}
 (out/'results.json').write_text(json.dumps(report,indent=2));np.savez_compressed(out/'execution.npz',qpos=states,qvel=vel,actual=actual,desired=desired,phase=phase,dt=m.opt.timestep,qdes=qs)
 print('DONE',json.dumps(report),flush=True)
if __name__=='__main__':main()
