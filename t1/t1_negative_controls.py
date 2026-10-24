"""Intentional simulation collisions to verify enabled T1 contact categories."""
from pathlib import Path
import numpy as np,mujoco,json
from t1_model import classify,check_distances,build_t1_scene
ROOT=Path(__file__).parent
cache=ROOT/'t1_model_check/scene.t1.mjb'
m=mujoco.MjModel.from_binary_path(str(cache)) if cache.exists() else build_t1_scene(cache.parent)
d=mujoco.MjData(m);rng=np.random.default_rng(20261024)
names=[f'astribot_torso_joint_{i}' for i in range(1,5)]+[f'astribot_arm_{s}_joint_{i}' for s in ['left','right'] for i in range(1,8)]
jids=np.array([m.joint(n).id for n in names]);qids=m.jnt_qposadr[jids];lim=m.jnt_range[jids];records={};targets={'inter_arm','same_arm_nonadjacent','arm_body','torso_head_base','wand'}
for k in range(2500):
 mujoco.mj_resetData(m,d);d.qpos[qids]=rng.uniform(lim[:,0]+.03,lim[:,1]-.03);mujoco.mj_forward(m,d)
 for ci in range(d.ncon):
  co=d.contact[ci];a,b=int(co.geom1),int(co.geom2);cat,_=classify(m,a,b,True)
  if 'wand' in m.geom(a).name or 'wand' in m.geom(b).name:cat='wand'
  if cat not in targets or cat in records or co.dist>=-.001:continue
  distance=float(mujoco.mj_geomDistance(m,d,a,b,.1,None));force=np.zeros(6);mujoco.mj_contactForce(m,d,ci,force)
  if np.linalg.norm(force[:3])<=1e-6:continue
  records[cat]={'sample':k,'geom1':m.geom(a).name,'geom2':m.geom(b).name,'contact_distance_m':float(co.dist),'distance_query_m':distance,'contact_force_norm_N':float(np.linalg.norm(force[:3])),'constraint_address':int(co.efc_address),'qpos':d.qpos.tolist()}
 if targets<=records.keys():break
for cat,r in records.items():
 mujoco.mj_resetData(m,d);d.qpos[:]=r['qpos'];mujoco.mj_forward(m,d);mujoco.mj_step(m,d);r['step_finite']=bool(np.isfinite(d.qpos).all());r['constraint_force_norm']=float(np.linalg.norm(d.qfrc_constraint));r['pass']=r['contact_distance_m']<0 and r['distance_query_m']<0 and r['contact_force_norm_N']>0 and r['constraint_address']>=0 and r['constraint_force_norm']>0 and r['step_finite']
report={'purpose':'Deliberate invalid simulation postures, not executable trajectories','pair_count':m.npair,'all_required_detected':targets<=records.keys(),'all_checks_pass':targets<=records.keys() and all(x['pass'] for x in records.values()),'tests':records}
(ROOT/'t1_model_check/negative-controls.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k!='tests'},indent=2))
for cat,r in records.items():print(cat,r['contact_distance_m'],r['distance_query_m'],r['contact_force_norm_N'],r['pass'])
