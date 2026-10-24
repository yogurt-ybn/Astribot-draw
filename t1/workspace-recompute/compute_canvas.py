"""Sample T1 fixed-plane reachability with torso witnesses and model collision screening.
This is a finite-sampled marginal reachability study, not a continuous workspace proof.
"""
from pathlib import Path
import sys,os,json,time,hashlib
os.environ.setdefault('MUJOCO_GL','egl')
ROOT=Path(__file__).resolve().parent
MODEL_ROOT=Path(os.environ.get('T1_DRAWING_ROOT',str(Path(__file__).resolve().parents[1])))
sys.path.insert(0,str(MODEL_ROOT))
import numpy as np
import mujoco
from t1_kinematics import Chain
from t1_model import build_t1_scene,collision_pairs,check_distances
from PIL import Image,ImageDraw,ImageFont
from scipy.spatial import cKDTree
PLANE=-.42;STEP=.02;MARGIN=.04;CLEARANCE=.002;TOL=.0005
TORSO_BENDS=[.3,.45,.6,.8,.95]
OUT=ROOT/'results-final';OUT.mkdir(parents=True,exist_ok=True)

def set_torso(m,d,c,torso,home):
 d.qpos[:]=home;d.qpos[:4]=torso;mujoco.mj_forward(m,d)
 p=int(m.body_parentid[c.bids[0]]);c.p0=d.xpos[p].copy();c.R0=d.xmat[p].reshape(3,3).copy()

def ik_batch(c,points,seeds,nsteps=100):
 q=np.asarray(seeds).copy();points=np.asarray(points);done=np.zeros(len(q),bool)
 for it in range(nsteps):
  xyz,_,J=c.fk(q,jacobian=True);e=points-xyz;err=np.linalg.norm(e,axis=1);done|=err<TOL
  ids=np.flatnonzero(~done)
  if len(ids)==0:break
  jj=J[ids];ee=e[ids]
  sol=np.linalg.solve(jj@jj.transpose(0,2,1)+np.eye(3)[None]*1e-5,ee[...,None])[...,0]
  dq=np.einsum('nij,ni->nj',jj,sol)
  q[ids]=np.clip(q[ids]+np.clip(dq,-.12,.12),c.lo,c.hi)
 err=np.linalg.norm(c.fk(q)[0]-points,axis=1)
 return q,err

def check_subset(m,d,pairs,threshold=CLEARANCE):
 md=.05
 for a,b,_ in pairs:
  dist=float(mujoco.mj_geomDistance(m,d,a,b,.05,None));md=min(md,dist)
  if dist<threshold:return False,md
 return True,md

def main():
 start=time.time();m=build_t1_scene(OUT/'model');d=mujoco.MjData(m);pairs=collision_pairs(m)
 cs=[Chain(m,s,margin=MARGIN) for s in ['left','right']];rng=np.random.default_rng(20261024)
 home=np.zeros(m.nq)
 for c in cs:home[c.qids]=np.clip(0,c.lo,c.hi)
 torsos=np.array([[.04,-b,b-.04,0] for b in TORSO_BENDS]+[[b,-2*b,b,0] for b in [.2,.35,.5,.65]]+[[.35,-.7,.35,yaw] for yaw in [-.4,.4]]);parks=[];parkok=[];park_reports=[]
 for ti,t in enumerate(torsos):
  qq=home.copy();qq[:4]=t;d.qpos[:]=qq;mujoco.mj_forward(m,d);valid,md=check_subset(m,d,pairs)
  if valid:
   parks.append(qq);parkok.append(True);park_reports.append({'torso_index':ti,'torso':t.tolist(),'valid':True,'method':'zero arms clipped 0.04rad inside limits','minimum_checked_clearance_m':md});print('PARK',ti,park_reports[-1],flush=True);continue
  cand=[]
  for si,c in enumerate(cs):
   set_torso(m,d,c,t,home)
   targets=np.array([[(-1 if si else 1)*x,y,z] for x in [.44,.55,.65] for y in [-.20,-.35,-.50] for z in [.65,.85,1.05]])
   p=np.repeat(targets,12,axis=0);seed=rng.uniform(c.lo,c.hi,(len(p),7));seed[::12]=home[c.qids]
   q,err=ik_batch(c,p,seed)
   relevant=[pair for pair in pairs if pair[2]!='inter_arm' and (pair[2]=='torso_head_base' or any(c.side in m.body(int(m.geom_bodyid[g])).name for g in pair[:2]))]
   ok=[]
   for ix in np.flatnonzero(err<TOL):
    d.qpos[:]=home;d.qpos[:4]=t;d.qpos[c.qids]=q[ix];mujoco.mj_forward(m,d)
    valid,md=check_subset(m,d,relevant)
    if valid:ok.append((q[ix],p[ix],md))
    if len(ok)>=16:break
   cand.append(ok)
  chosen=None
  for a in cand[0]:
   for b in cand[1]:
    qq=home.copy();qq[:4]=t;qq[cs[0].qids]=a[0];qq[cs[1].qids]=b[0];d.qpos[:]=qq;mujoco.mj_forward(m,d)
    good,md=check_subset(m,d,pairs)
    if good:chosen=qq;report={'torso_index':ti,'torso':t.tolist(),'left_tip':a[1].tolist(),'right_tip':b[1].tolist(),'minimum_checked_clearance_m':md};break
   if chosen is not None:break
  if chosen is None:
   parks.append(home.copy());parkok.append(False);park_reports.append({'torso_index':ti,'valid':False,'candidate_counts':[len(x) for x in cand]})
  else:parks.append(chosen);parkok.append(True);park_reports.append(dict(report,valid=True))
  print('PARK',ti,park_reports[-1],flush=True)
 if not any(parkok):raise RuntimeError('No collision-clear parked state, cannot claim collision-screened marginal workspace')
 parks=np.array(parks)
 # Conservative search rectangle from triangle inequality of chain offsets over all sampled torsos.
 centers=[];radii=[]
 for ti,t in enumerate(torsos):
  if not parkok[ti]:continue
  for c in cs:
   set_torso(m,d,c,t,parks[ti]);centers.append(c.p0+c.R0@c.pos[0]);radii.append(float(np.linalg.norm(c.pos[1:],axis=1).sum()+.23))
 centers=np.array(centers);rad=np.array(radii)
 xmin=np.floor(np.min(centers[:,0]-rad)/STEP)*STEP;xmax=np.ceil(np.max(centers[:,0]+rad)/STEP)*STEP
 zmin=max(0.,np.floor(np.min(centers[:,2]-rad)/STEP)*STEP);zmax=np.ceil(np.max(centers[:,2]+rad)/STEP)*STEP
 xs=np.arange(xmin,xmax+STEP/2,STEP);zs=np.arange(zmin,zmax+STEP/2,STEP);xx,zz=np.meshgrid(xs,zs);points=np.c_[xx.ravel(),np.full(xx.size,PLANE),zz.ravel()]
 witness_q=np.full((2,len(points),m.nq),np.nan);witness_t=np.full((2,len(points)),-1,dtype=np.int16);residual=np.full((2,len(points)),np.nan);clearance=residual.copy();stats=[]
 for si,c in enumerate(cs):
  for ti,t in enumerate(torsos):
   if not parkok[ti]:continue
   set_torso(m,d,c,t,parks[ti]);center=c.p0+c.R0@c.pos[0];rmax=np.linalg.norm(c.pos[1:],axis=1).sum()+.23
   possible=np.linalg.norm(points-center,axis=1)<=rmax+TOL
   ids=np.flatnonzero(possible & (witness_t[si]<0));attempted=0;passed=0
   # Multiple deterministic postures; no random success is treated as absence proof.
   bank=rng.uniform(c.lo,c.hi,(60000,7));xyz=c.fk(bank)[0];tree=cKDTree(xyz)
   near_dist,near_idx=tree.query(points,k=6)
   seeds=[parks[ti,c.qids]]+list(range(6))
   for seed_index,seed in enumerate(seeds):
    unresolved=ids[witness_t[si,ids]<0]
    if not len(unresolved):break
    if seed_index==0: initial=np.tile(seed,(len(unresolved),1))
    else:
     unresolved=unresolved[near_dist[unresolved,seed]<.10]
     initial=bank[near_idx[unresolved,seed]]
    if not len(unresolved):continue
    q,err=ik_batch(c,points[unresolved],initial,nsteps=75)
    for ii in np.flatnonzero(err<TOL):
     idx=unresolved[ii];qq=parks[ti].copy();qq[c.qids]=q[ii];d.qpos[:]=qq;mujoco.mj_forward(m,d);attempted+=1
     valid,md=check_subset(m,d,pairs)
     if valid:
      witness_q[si,idx]=qq;witness_t[si,idx]=ti;residual[si,idx]=np.linalg.norm(d.site(c.side+'_light').xpos-points[idx]);clearance[si,idx]=md;passed+=1
    print('GRID',c.side,ti,seed_index,'new',passed,'total',int((witness_t[si]>=0).sum()),'elapsed',round(time.time()-start,1),flush=True)
    np.savez_compressed(OUT/'workspace-witnesses.partial.npz',xs=xs,zs=zs,points=points,qpos=witness_q,torso_index=witness_t,residual_m=residual,clearance_m=clearance,torso_samples=torsos,park_qpos=parks,park_valid=parkok)
   stats.append({'arm':c.side,'torso_index':ti,'collision_candidates':attempted,'new_acceptances':passed})
 np.savez_compressed(OUT/'workspace-witnesses.npz',xs=xs,zs=zs,points=points,qpos=witness_q,torso_index=witness_t,residual_m=residual,clearance_m=clearance,torso_samples=torsos,park_qpos=parks,park_valid=parkok)
 meta={'robot':'Astribot T1','upstream_commit':'cab872dc00081d2c5b096c7a19e563050eb7a2ff','plane_y_m':PLANE,'grid_spacing_m':STEP,'joint_margin_rad':MARGIN,'ik_tolerance_m':TOL,'collision_clearance_threshold_m':CLEARANCE,'full_model_pair_count':len(pairs),'torso_samples':torsos.tolist(),'park_reports':park_reports,'search_bound_m':{'x':[xmin,xmax],'z':[zmin,zmax]},'grid_shape':[len(zs),len(xs)],'accepted_points_per_arm':[int((a>=0).sum()) for a in witness_t],'max_fk_residual_m':float(np.nanmax(residual)),'minimum_witness_clearance_m':float(np.nanmin(clearance)),'elapsed_s':time.time()-start,'statistics':stats,'limitations':['Finite grid, finite torso samples, finite multi-start IK search; rejected/unfound points are not a proof of unreachable positions.','Each accepted point has one full collision-screened pose with opposite arm parked for that sampled torso.','Marginal union over torso: two arbitrary accepted left/right points need not share a feasible torso posture.','Position-only reachability with free tool orientation; not a continuous path or execution guarantee.','Collision proxies and explicit exclusions are model-specific; no hardware safety guarantee.']}
 (OUT/'workspace-summary.json').write_text(json.dumps(meta,indent=2));print('DONE',json.dumps(meta),flush=True)
if __name__=='__main__':main()
