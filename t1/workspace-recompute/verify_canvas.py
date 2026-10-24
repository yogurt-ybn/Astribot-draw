from compute_canvas import *
g=np.load(OUT/'workspace-witnesses.npz');m=mujoco.MjModel.from_binary_path(str(OUT/'model/scene.t1.mjb'));d=mujoco.MjData(m);pairs=collision_pairs(m);report={'model_pairs':len(pairs),'samples':{}}
for si,side in enumerate(['left','right']):
 ids=np.flatnonzero(g['torso_index'][si]>=0);maxerr=0;minmargin=1e9;mind=.05;c=Chain(m,side);sample_ids=ids[np.linspace(0,len(ids)-1,min(160,len(ids)),dtype=int)]
 for i in ids:
  q=g['qpos'][si,i];d.qpos[:]=q;mujoco.mj_forward(m,d);err=np.linalg.norm(d.site(side+'_light').xpos-g['points'][i]);maxerr=max(maxerr,err);margin=np.minimum(q[c.qids]-c.limits[:,0],c.limits[:,1]-q[c.qids]).min();minmargin=min(minmargin,margin)
  assert err<TOL+1e-10 and margin>=MARGIN-1e-10
 for i in sample_ids:
  d.qpos[:]=g['qpos'][si,i];mujoco.mj_forward(m,d);valid,md=check_subset(m,d,pairs);assert valid;mind=min(mind,md)
 report['samples'][side]={'all_fk_witnesses_checked':len(ids),'max_fk_error_m':maxerr,'min_active_arm_limit_margin_rad':minmargin,'collision_witnesses_rechecked':len(sample_ids),'min_sampled_distance_m':mind}
cal=json.loads((OUT/'calibration.json').read_text());scale=cal['meters_per_pixel'];lo=cal['world_bounds_m']['x'][0];top=cal['world_bounds_m']['z'][1];err=[]
for si in range(2):
 for i in np.flatnonzero(g['torso_index'][si]>=0):
  p=g['points'][i];u=(p[0]-lo)/scale;v=(top-p[2])/scale;back=np.array([lo+u*scale,PLANE,top-v*scale]);err.append(np.linalg.norm(back-p))
report['pixel_world_roundtrip_max_m']=max(err);report['passed']=True
(OUT/'independent-checks.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
