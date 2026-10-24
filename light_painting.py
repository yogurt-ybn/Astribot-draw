#!/usr/bin/env python3
"""Camera-space light painting with Astribot data and genuine MuJoCo dynamics."""
from __future__ import annotations
import argparse, json, os, time, csv, math, xml.etree.ElementTree as ET
from pathlib import Path
os.environ.setdefault('MUJOCO_GL','egl')
os.environ.setdefault('PYOPENGL_PLATFORM','egl')
os.environ.setdefault('LIBGL_ALWAYS_SOFTWARE','1')
os.environ.setdefault('MESA_SHADER_CACHE_DISABLE','true')
import numpy as np
import mujoco
from PIL import Image, ImageDraw, ImageFilter, ImageFont
ROOT=Path(__file__).resolve().parent
DEFAULT_ASSETS=Path(os.environ.get('ASTRIBOT_ASSETS', str(ROOT/'descriptions')))
W,H=960,720
CAM=np.array([0.,-4.0,.95]); FOV=35.0
PLANE_Y=-.20
FONT='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'

def expand(path):
    tree=ET.parse(path).getroot()
    def visit(node):
        for i,child in reversed(list(enumerate(list(node)))):
            if child.tag=='include':
                content=expand((path.parent/child.attrib['file']).resolve())
                node.remove(child)
                for sub in reversed(list(content)):node.insert(i,sub)
            else:
                if child.tag in ('mesh','texture') and child.get('file'):
                    child.set('file',str((path.parent/child.get('file')).resolve()))
                visit(child)
    visit(tree); return tree

def build_scene(assets,output):
    modelpath=assets/'mjcf/astribot_s1_mjcf/astribot_s1_chassis_fixed_with_gripper.xml'
    root=expand(modelpath)
    vis=ET.SubElement(root,'visual');ET.SubElement(vis,'global',offwidth=str(W),offheight=str(H))
    ET.SubElement(vis,'headlight',ambient='.25 .25 .25',diffuse='.45 .45 .45',specular='.1 .1 .1')
    ET.SubElement(vis,'rgba',haze='.025 .035 .055 1')
    world=root.find('worldbody')
    # Local camera X is global +X; local Y is global +Z; it looks along +Y.
    ET.SubElement(world,'camera',name='photography',pos='0 -4 .95',xyaxes='1 0 0 0 0 1',fovy=str(FOV))
    asset=root.find('asset')
    ET.SubElement(asset,'material',name='human_cloth',rgba='.13 .30 .45 1',roughness='.8')
    ET.SubElement(asset,'material',name='human_skin',rgba='.72 .52 .37 1',roughness='.7')
    # Static geometric mannequin, 1.74 m tall, 0.70 m ahead of chassis centre.
    human=ET.SubElement(world,'body',name='mannequin',pos='0 -.70 0')
    def geom(name,typ,size,pos=None,frm=None,mat='human_cloth'):
        kw=dict(name='human_'+name,type=typ,size=size,material=mat,contype='1',conaffinity='1',group='4')
        if pos:kw['pos']=pos
        if frm:kw['fromto']=frm
        ET.SubElement(human,'geom',**kw)
    geom('torso','ellipsoid','.175 .115 .255','0 0 1.22')
    geom('pelvis','ellipsoid','.16 .115 .135','0 0 .94')
    geom('neck','capsule','.045',frm='0 0 1.44 0 0 1.51',mat='human_skin')
    geom('head','ellipsoid','.105 .095 .125','0 0 1.61',mat='human_skin')
    for s,name in [(1,'left'),(-1,'right')]:
        geom(name+'_upper_arm','capsule','.052',frm=f'{s*.19} 0 1.40 {s*.245} 0 1.13')
        geom(name+'_forearm','capsule','.043',frm=f'{s*.245} 0 1.13 {s*.255} -.015 .90',mat='human_skin')
        geom(name+'_thigh','capsule','.074',frm=f'{s*.085} 0 .94 {s*.095} 0 .55')
        geom(name+'_shin','capsule','.055',frm=f'{s*.095} 0 .55 {s*.10} 0 .13')
        geom(name+'_foot','box','.067 .125 .045',f'{s*.1} -.045 .05')
    # Visible face direction marker toward the camera.
    geom('nose','sphere','.022','0 -.093 1.61',mat='human_skin')
    for side,color in [('left','.15 .85 1 1'),('right','1 .40 .1 1')]:
        arm=root.find(f".//body[@name='astribot_arm_{side}_link_7']")
        wand=ET.SubElement(arm,'body',name=f'{side}_wand',pos='0 -.15 0')
        ET.SubElement(wand,'inertial',pos='0 -.04 0',mass='.025',diaginertia='.00002 .000005 .00002')
        ET.SubElement(wand,'geom',name=f'{side}_wand_geom',type='capsule',fromto='0 0 0 0 -.08 0',size='.007',rgba=color,contype='1',conaffinity='1',group='5')
        ET.SubElement(wand,'site',name=f'{side}_light',pos='0 -.08 0',size='.012',rgba=color)
    # Dim background; physical geometry and light tips stay visible.
    floor=world.find("geom[@name='floor']");floor.attrib.pop('material',None);floor.set('rgba','.055 .075 .105 1')
    for light in world.findall('light'):light.set('diffuse','.5 .5 .5');light.set('ambient','.18 .18 .18');light.set('castshadow','false')
    xml=ET.tostring(root,encoding='unicode');(output/'scene.generated.xml').write_text(xml)
    return mujoco.MjModel.from_xml_string(xml)

def projection(xyz):
    pts=np.asarray(xyz); depth=pts[...,1]-CAM[1]
    f=H/(2*np.tan(np.deg2rad(FOV)/2))
    return np.stack([W/2+f*pts[...,0]/depth,H/2-f*(pts[...,2]-CAM[2])/depth],axis=-1)

def ray_plane(uv):
    uv=np.asarray(uv);f=H/(2*np.tan(np.deg2rad(FOV)/2));d=PLANE_Y-CAM[1]
    return np.stack([(uv[...,0]-W/2)*d/f,np.full(uv.shape[:-1],PLANE_Y),CAM[2]-(uv[...,1]-H/2)*d/f],axis=-1)

def bezier(points,n=55):
    t=np.linspace(0,1,n)[:,None];p=np.asarray(points)
    return (1-t)**3*p[0]+3*(1-t)**2*t*p[1]+3*(1-t)*t*t*p[2]+t**3*p[3]

def design_strokes():
    # Designer controls are camera image points. Below are intuitive normalized
    # image offsets from root; convert them into actual pixel coordinates first.
    # Only the first outline closes; ribs are separate light-on strokes.
    curves=[
      [[.21,1.34],[.30,1.56],[.52,1.80],[.64,1.78]],
      [[.64,1.78],[.67,1.68],[.69,1.55],[.65,1.43]],
      [[.65,1.43],[.62,1.51],[.59,1.52],[.58,1.47]],
      [[.58,1.47],[.59,1.36],[.59,1.28],[.54,1.20]],
      [[.54,1.20],[.52,1.32],[.49,1.36],[.46,1.30]],
      [[.46,1.30],[.46,1.19],[.44,1.11],[.39,1.05]],
      [[.39,1.05],[.35,1.20],[.28,1.29],[.21,1.34]],
    ]
    outline=np.concatenate([bezier(p,35)[:-1] for p in curves]+[np.array([[.21,1.34]])])
    strokes=[outline]
    for end,mid in [((.64,1.78),(.48,1.55)),((.65,1.43),(.44,1.46)),((.54,1.20),(.39,1.34)),((.39,1.05),(.30,1.28))]:
        strokes.append(bezier([[.21,1.34],[.27,1.39],mid,end],60))
    # Explicit design->pixels->camera rays->selected reachable world plane.
    return [projection(np.c_[p[:,0],np.full(len(p),PLANE_Y),p[:,1]]) for p in strokes]

def arm_indices(m,side):
    js=[m.joint(f'astribot_arm_{side}_joint_{i}').id for i in range(1,8)]
    return np.array(js),m.jnt_qposadr[js],m.jnt_dofadr[js]

def solve_ik(m,d,side,target,seed,iterations=180):
    js,qs,vs=arm_indices(m,side);sid=m.site(f'{side}_light').id
    d.qpos[qs]=seed;lo=m.jnt_range[js,0]+.01;hi=m.jnt_range[js,1]-.01
    jp=np.zeros((3,m.nv));jr=np.zeros_like(jp)
    for it in range(iterations):
        mujoco.mj_forward(m,d);err=target-d.site_xpos[sid]
        if np.linalg.norm(err)<.00015:break
        mujoco.mj_jacSite(m,d,jp,jr,sid);j=jp[:,vs]
        dq=j.T@np.linalg.solve(j@j.T+np.eye(3)*.0002,err)
        d.qpos[qs]=np.clip(d.qpos[qs]+np.clip(dq,-.09,.09),lo,hi)
    mujoco.mj_forward(m,d)
    return d.qpos[qs].copy(),float(np.linalg.norm(target-d.site_xpos[sid]))

def plan(m,output):
    uv_strokes=design_strokes();world_strokes=[ray_plane(s) for s in uv_strokes]
    d=mujoco.MjData(m);plan_strokes=[];all_err=[]
    seeds={s:np.array([-.3,-.2,-.5,1.1,0,0,0]) for s in ('left','right')}
    seeds['right'][[0,2]]*=-1
    for k,stroke in enumerate(world_strokes):
        q=[];positions=[]
        for p in stroke:
            full=np.zeros(m.nq);paired=[]
            for side,sgn in [('left',1),('right',-1)]:
                target=p.copy();target[0]*=sgn
                value,error=solve_ik(m,d,side,target,seeds[side])
                if error>.003:
                    # Deterministic cold restarts for hard start points only.
                    rng=np.random.default_rng(5)
                    best=(value,error)
                    for _ in range(8):
                        trial=seeds[side]+rng.normal(0,.35,7)
                        v,e=solve_ik(m,d,side,target,trial,300)
                        if e<best[1]:best=(v,e)
                        if e<.0002:break
                    value,error=best
                seeds[side]=value
                full[arm_indices(m,side)[1]]=value;all_err.append(error);paired.append(target)
            q.append(full);positions.append(paired)
        plan_strokes.append({'q':np.array(q),'p':np.array(positions)})
    stats={'waypoints_per_arm':sum(len(s['q']) for s in plan_strokes),'maximum_ik_error_m':max(all_err),'mean_ik_error_m':float(np.mean(all_err))}
    print('IK',json.dumps(stats),flush=True)
    (output/'ik_stats.json').write_text(json.dumps(stats,indent=2))
    if max(all_err)>.005:raise RuntimeError(f'Infeasible design; max IK error {max(all_err):.5f} m')
    return plan_strokes,stats

def render_preview(m,output,plan_strokes):
    d=mujoco.MjData(m);d.qpos[:]=plan_strokes[0]['q'][len(plan_strokes[0]['q'])//3];mujoco.mj_forward(m,d)
    with mujoco.Renderer(m,H,W) as r:
        opt=mujoco.MjvOption();opt.geomgroup[:]=1
        r.update_scene(d,camera='photography',scene_option=opt);im=Image.fromarray(r.render());im.save(output/'scene-preview.png')
        overview=mujoco.MjvCamera();overview.lookat[:]=[0,-.25,1.0];overview.distance=3.6;overview.azimuth=145;overview.elevation=-17
        r.update_scene(d,camera=overview,scene_option=opt);Image.fromarray(r.render()).save(output/'scene-overview.png')
    return im

def point_segment_distance(p,a,b):
    ab=b-a;t=np.clip(np.dot(p-a,ab)/max(np.dot(ab,ab),1e-12),0,1)
    return np.linalg.norm(p-a-t*ab)

def capsule_clearance(m,d):
    # Approximate robot capsules sampled against conservative mannequin capsules.
    # This is a screening metric, NOT whole-body collision certification.
    human=[];y=-.70
    human.append((np.array([0,y,1.03]),np.array([0,y,1.41]),.18))
    human.append((np.array([0,y,1.61]),np.array([0,y,1.61]),.13))
    for s in (-1,1):
        human.extend([(np.array([s*.19,y,1.4]),np.array([s*.255,y,.9]),.06),
                      (np.array([s*.085,y,.94]),np.array([s*.1,y,.13]),.08)])
    best=10.
    for side in ('left','right'):
        xyz=[d.body(f'astribot_arm_{side}_link_{j}').xpos.copy() for j in (1,4,7)]+[d.site(f'{side}_light').xpos.copy()]
        for a,b,r in zip(xyz[:-1],xyz[1:],[.08,.065,.065]):
            for p in np.linspace(a,b,15):
                for ha,hb,hr in human:best=min(best,point_segment_distance(p,ha,hb)-r-hr)
    return best

def create_timeline(m,strokes,dt):
    arrays=[];light=[];ids=[];targets=[]
    current=np.zeros(m.nq)
    def transfer(goal,duration,label):
        nonlocal current
        n=max(2,math.ceil(duration/dt));u=np.linspace(0,1,n,endpoint=False);v=10*u**3-15*u**4+6*u**5
        arrays.append(current[None,:]+v[:,None]*(goal-current));light.extend([False]*n);ids.extend([label]*n)
        targets.extend([np.full((2,3),np.nan)]*n);current=goal.copy()
    transfer(strokes[0]['q'][0],6.,'approach')
    for i,s in enumerate(strokes):
        if i:transfer(s['q'][0],max(2.5,float(np.max(np.abs(s['q'][0]-current)))/.22),'reposition')
        q=s['q'];p=s['p'];ds=np.linalg.norm(np.diff(p[:,0,:],axis=0),axis=1)
        arc=np.r_[0,np.cumsum(ds)];length=arc[-1];arc/=length
        # Quintic arclength scheduling, max 0.10 m/s and conservative joint speed.
        slope=np.max(np.abs(np.diff(q,axis=0))/np.maximum(np.diff(arc)[:,None],1e-8))
        duration=max(4.,1.875*length/.10,1.875*slope/.45)
        n=math.ceil(duration/dt);u=np.linspace(0,1,n,endpoint=False);v=10*u**3-15*u**4+6*u**5
        arrays.append(np.array([np.interp(v,arc,q[:,j]) for j in range(m.nq)]).T)
        targets.extend(np.stack([np.interp(v,arc,p[:,si,j]) for si in range(2) for j in range(3)],axis=1).reshape(n,2,3))
        light.extend([True]*n);ids.extend([f'stroke_{i}']*n);current=q[-1].copy()
    transfer(current,2.,'settle')
    q=np.concatenate(arrays);dq=np.gradient(q,dt,axis=0)
    return q,dq,np.array(light),ids,np.array(targets)

def execute(m,strokes,output,stats):
    dt=float(m.opt.timestep);qdes,dqdes,on,ids,desired=create_timeline(m,strokes,dt)
    d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    # Initial qpos stays at model zero. Only ctrl is changed during execution.
    joint_act=[i for i in range(m.nu) if m.actuator_trntype[i]==mujoco.mjtTrn.mjTRN_JOINT]
    joints=m.actuator_trnid[joint_act,0];qa=m.jnt_qposadr[joints];va=m.jnt_dofadr[joints]
    kp=m.actuator_gainprm[joint_act,0];kd=-m.actuator_biasprm[joint_act,2]
    sites=[m.site(s+'_light').id for s in ('left','right')]
    geomgroup=np.ones(6,dtype=np.uint8)
    actual=[];state=[];velocities=[];forces=[];controls=[];visible=[];timestamps=[];contacts=0;contact_human=0;max_vel=0.;max_force=0.;minclear=10.;warnings=[]
    gid=np.zeros(1,dtype=np.int32);start=time.perf_counter()
    for k,(q,dq) in enumerate(zip(qdes,dqdes)):
        ff=d.qfrc_bias[va]-d.qfrc_passive[va]
        d.ctrl[joint_act]=q[qa]+(kd*dq[qa]+ff)/kp
        d.ctrl[:]=np.clip(d.ctrl,m.actuator_ctrlrange[:,0],m.actuator_ctrlrange[:,1])
        mujoco.mj_step(m,d)
        if not (np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all() and np.isfinite(d.qacc).all()):raise RuntimeError('Non-finite dynamics')
        if np.any(d.warning.number):raise RuntimeError(f'MuJoCo warning: {d.warning.number}')
        # Refresh Cartesian state after integration, no state mutation.
        mujoco.mj_forward(m,d)
        pts=d.site_xpos[sites].copy();actual.append(pts);timestamps.append(float(d.time));state.append(d.qpos.copy());velocities.append(d.qvel.copy());forces.append(d.actuator_force.copy());controls.append(d.ctrl.copy())
        vis=[]
        for pt in pts:
            vec=pt-CAM;distance=np.linalg.norm(vec)
            hit=mujoco.mj_ray(m,d,CAM,vec/distance,geomgroup,1,-1,gid)
            vis.append(hit<0 or hit>=distance-.018)
        visible.append(vis)
        contacts=max(contacts,d.ncon)
        for c in d.contact:
            if m.geom(c.geom1).name.startswith('human_') or m.geom(c.geom2).name.startswith('human_'):contact_human+=1
        max_vel=max(max_vel,float(np.max(np.abs(d.qvel))));max_force=max(max_force,float(np.max(np.abs(d.actuator_force))))
        if k%50==0:minclear=min(minclear,capsule_clearance(m,d))
    actual=np.array(actual);state=np.array(state);velocities=np.array(velocities);forces=np.array(forces);controls=np.array(controls);visible=np.array(visible);timestamps=np.array(timestamps)
    errors=np.linalg.norm(actual[on]-desired[on],axis=2);px=np.linalg.norm(projection(actual[on])-projection(desired[on]),axis=2)
    qerr=np.abs(state[on][:,qa]-qdes[on][:,qa]);qs=m.jnt_qposadr[joints];limits=m.jnt_range[joints]
    margin=np.minimum(state[:,qs]-limits[:,0],limits[:,1]-state[:,qs]);arm_joints=np.array([m.joint(f'astribot_arm_{s}_joint_{i}').id for s in ('left','right') for i in range(1,8)])
    armqs=m.jnt_qposadr[arm_joints];armranges=m.jnt_range[arm_joints]
    armmargin=np.minimum(state[:,armqs]-armranges[:,0],armranges[:,1]-state[:,armqs])
    report={**stats,'run_utc':__import__('datetime').datetime.now(__import__('datetime').timezone.utc).isoformat(),'python_simulation':'MuJoCo '+mujoco.__version__,'steps':len(qdes),'simulation_seconds':float(d.time),'physics_wall_seconds':time.perf_counter()-start,
      'all_states_finite':True,'warning_counts':d.warning.number.tolist(),'maximum_position_error_m':float(errors.max()),'rms_position_error_m':float(np.sqrt(np.mean(errors**2))),
      'maximum_projection_error_px':float(px.max()),'rms_projection_error_px':float(np.sqrt(np.mean(px**2))),
      'maximum_joint_tracking_error_rad':float(qerr.max()),'maximum_joint_speed_rad_s':max_vel,'maximum_actuator_force_abs':max_force,
      'minimum_arm_joint_limit_margin_rad':float(armmargin.min()),'minimum_arm_joint_limit_margin_light_on_rad':float(armmargin[on].min()),'maximum_arm_joint_speed_rad_s':float(np.abs(velocities[:,m.jnt_dofadr[arm_joints]]).max()),'maximum_light_on_tip_speed_m_s':float(np.linalg.norm(np.gradient(actual,dt,axis=0)[on],axis=2).max()),'maximum_actuator_force_limit_fraction':float((np.abs(forces)/m.actuator_forcerange[:,1]).max()),'maximum_contacts':int(contacts),'human_contact_observations':int(contact_human),
      'approximate_arm_human_min_clearance_m':float(minclear),'light_on_samples_per_arm':int(on.sum()),'visible_fraction_by_arm':visible[on].mean(axis=0).tolist(),
      'camera':{'position_m':CAM.tolist(),'fovy_deg':FOV,'width':W,'height':H,'look_direction':[0,1,0]},
      'mannequin_position_m':[0,-.70,0],'painting_plane_y_m':PLANE_Y,'fixed_base_position_m':[0,0,.097],
      'controller':'Upstream bounded position actuators, velocity feedforward and model bias/passive-force feedforward in ctrl. No qpos assignment after initial state; no qfrc_applied force shortcut.',
      'render_method':'Tip point emissions accumulated from every actual dynamic step, with ray visibility against current robot and mannequin geometries. Gaussian bloom is illustrative postprocessing. This is ground-truth synthetic exposure, not extraction from compressed video.',
      'limitations':['Fixed chassis; relocation/navigation not executed.','Geometric static mannequin; no perception, human pose noise or human motion.','Light wands are rigidly attached 25 g bodies, no grasp/slippage simulation.','IK constrains light-tip position, not full tool orientation.','Most upstream robot arm/torso collision geoms are disabled. Zero contacts does NOT validate collision safety.','Arm-human capsule-distance screen is approximate; self-collision and full-body collision safety are not validated.','Ideal model feedforward, known calibration and noise-free geometry; not a hardware-ready controller.']}
    np.savez_compressed(output/'execution.npz',time=timestamps,qpos=state,qvel=velocities,ctrl=controls,actuator_force=forces,qdes=qdes,actual=actual,desired=desired,light_on=on,visible=visible,phase=np.array(ids))
    with (output/'actual-tool-trace.csv').open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(['time_s','phase','light_on','side','x_m','y_m','z_m','desired_x_m','desired_y_m','desired_z_m','u_px','v_px','desired_u_px','desired_v_px','visible','error_m','projection_error_px'])
        uv=projection(actual);duv=projection(desired)
        for k,t in enumerate(timestamps):
            for si,side in enumerate(('left','right')):
                err=float(np.linalg.norm(actual[k,si]-desired[k,si]));pe=float(np.linalg.norm(uv[k,si]-duv[k,si]))
                writer.writerow([f'{t:.4f}',ids[k],int(on[k]),side,*actual[k,si],*desired[k,si],*uv[k,si],*duv[k,si],int(visible[k,si]),err,pe])
    (output/'results.json').write_text(json.dumps(report,indent=2)+'\n')
    print('DYNAMICS',json.dumps(report),flush=True)
    return report,{'time':timestamps,'qpos':state,'actual':actual,'desired':desired,'on':on,'visible':visible,'phase':ids}

def light_layer(points,visible,on,width=W,height=H):
    # Direct emissions at sampled positions; never draw connectors across occlusion
    # or light-off repositioning. Every visible sample is a physical simulated tip.
    canvas=Image.new('RGB',(width,height));draw=ImageDraw.Draw(canvas)
    uv=projection(points);colors=[(40,210,255),(255,120,45)]
    for k in np.flatnonzero(on):
        for si,color in enumerate(colors):
            if visible[k,si]:
                x,y=uv[k,si];draw.ellipse((x-1.3,y-1.3,x+1.3,y+1.3),fill=color)
    core=np.asarray(canvas,dtype=float);near=np.asarray(canvas.filter(ImageFilter.GaussianBlur(3)),dtype=float);far=np.asarray(canvas.filter(ImageFilter.GaussianBlur(9)),dtype=float)
    return np.clip(core+near*.9+far*1.2,0,255).astype(np.uint8)

def label(im,title,subtitle):
    im=im.copy();dr=ImageDraw.Draw(im);dr.rectangle((0,0,im.width,77),fill=(11,17,27))
    dr.text((25,16),title,font=ImageFont.truetype(FONT,23),fill=(238,244,255));dr.text((25,48),subtitle,font=ImageFont.truetype(FONT,14),fill=(155,177,199));return im

def make_outputs(m,strokes,run,report,output,video=False):
    d=mujoco.MjData(m);opt=mujoco.MjvOption();opt.geomgroup[:]=1
    # Reference exposure base is the actual final state. Robot is intentionally
    # present; this is not a claim that motion vanished during long exposure.
    d.qpos[:]=run['qpos'][-1];mujoco.mj_forward(m,d)
    with mujoco.Renderer(m,H,W) as r:
        r.update_scene(d,camera='photography',scene_option=opt);base=r.render().copy()
        Image.fromarray(base).save(output/'camera-final-frame.png')
        layer=light_layer(run['actual'],run['visible'],run['on'])
        composite=np.clip(base.astype(float)*.72+layer,0,255).astype(np.uint8)
        Image.fromarray(layer).save(output/'visible-actual-trail.png')
        label(Image.fromarray(composite),'ASTRIBOT / LIGHT-PAINTING PROTOTYPE','Actual simulated tip path | occlusion-aware synthetic exposure | fixed base').save(output/'light-painting-actual.png')
        # Desired dotted versus actual solid in same calibrated image, unoccluded
        # desired shown explicitly as a geometric diagnostic, not as exposure.
        compare=Image.fromarray((base*.45).astype(np.uint8));dr=ImageDraw.Draw(compare)
        for stroke in strokes:
            for si in range(2):
                uv=projection(stroke['p'][:,si]);
                for j in range(0,len(uv)-1,5):dr.line([tuple(v) for v in uv[j:j+3]],fill=(245,245,245),width=3)
        uv=projection(run['actual'])
        for k in np.flatnonzero(run['on']):
            for si,col in enumerate([(40,210,255),(255,120,45)]):
                if run['visible'][k,si]:
                    x,y=uv[k,si];dr.ellipse((x-1,y-1,x+1,y+1),fill=col)
        label(compare,'DESIRED / ACTUAL CAMERA PROJECTION',f'White dashed: target | Color: visible actual | RMS {report["rms_projection_error_px"]:.2f} px, max {report["maximum_projection_error_px"]:.2f} px').save(output/'desired-vs-actual.png')
        # Occlusion diagnostic: all actual projected samples versus visible subset.
        occl=Image.fromarray((base*.4).astype(np.uint8));dr=ImageDraw.Draw(occl)
        for k in np.flatnonzero(run['on']):
            for si in range(2):
                x,y=uv[k,si];col=(45,215,180) if run['visible'][k,si] else (245,70,95)
                dr.ellipse((x-1,y-1,x+1,y+1),fill=col)
        label(occl,'VISIBILITY CHECK','Green: visible emission | Red: occluded actual sample, excluded from final exposure').save(output/'occlusion-check.png')
        overview=mujoco.MjvCamera();overview.lookat[:]=[0,-.30,1.0];overview.distance=3.4;overview.azimuth=145;overview.elevation=-17
        k=np.flatnonzero(run['on'])[len(np.flatnonzero(run['on']))//3];d.qpos[:]=run['qpos'][k];mujoco.mj_forward(m,d)
        r.update_scene(d,camera=overview,scene_option=opt)
        label(Image.fromarray(r.render()),'SCENE / SIDE VIEW','Camera in front of mannequin | robot behind | rigid light wands on both grippers').save(output/'scene-overview.png')
    panels=[Image.open(output/x).resize((640,480)) for x in ['light-painting-actual.png','desired-vs-actual.png','scene-overview.png','occlusion-check.png']]
    sheet=Image.new('RGB',(1280,960))
    for im,xy in zip(panels,[(0,0),(640,0),(0,480),(640,480)]):sheet.paste(im,xy)
    sheet.save(output/'final-summary.png')
    if video:make_video(m,run,output,opt)

def make_video(m,run,output,opt):
    import subprocess
    n=80;indices=np.linspace(0,len(run['time'])-1,n).astype(int)
    dest=output/'light-painting-demo.mp4'
    cmd=['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s','640x480','-r','10','-i','-','-an','-c:v','libx264','-crf','21','-pix_fmt','yuv420p',str(dest)]
    proc=subprocess.Popen(cmd,stdin=subprocess.PIPE)
    raw_cmd=cmd.copy();raw_cmd[-1]=str(output/'raw-camera-replay.mp4');raw_proc=subprocess.Popen(raw_cmd,stdin=subprocess.PIPE)
    d=mujoco.MjData(m)
    with mujoco.Renderer(m,480,640) as r:
        for ix,k in enumerate(indices):
            d.qpos[:]=run['qpos'][k];mujoco.mj_forward(m,d);r.update_scene(d,camera='photography',scene_option=opt)
            base=r.render().copy();raw_proc.stdin.write(base.tobytes());layer=np.asarray(Image.fromarray(light_layer(run['actual'][:k+1],run['visible'][:k+1],run['on'][:k+1])).resize((640,480)))
            frame=Image.fromarray(np.clip(base.astype(float)*.72+layer,0,255).astype(np.uint8));dr=ImageDraw.Draw(frame)
            dr.rectangle((0,0,640,43),fill=(11,17,27));dr.text((12,8),f'ACTUAL DYNAMICS REPLAY / t = {run["time"][k]:.1f} s',fill='white',font=ImageFont.truetype(FONT,17))
            dr.text((12,28),'Accumulated visible emissions | accelerated replay | fixed base',fill=(160,184,210),font=ImageFont.truetype(FONT,11))
            proc.stdin.write(np.asarray(frame).tobytes())
            if ix%20==0:print('VIDEO',ix,'/',n,flush=True)
    proc.stdin.close();raw_proc.stdin.close()
    if proc.wait()!=0 or raw_proc.wait()!=0:raise RuntimeError('ffmpeg failed')

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--assets',type=Path,default=DEFAULT_ASSETS);ap.add_argument('--output',type=Path,default=ROOT/'results');ap.add_argument('--preview-only',action='store_true');ap.add_argument('--video',action='store_true');args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    m=build_scene(args.assets,args.output);p,stats=plan(m,args.output);render_preview(m,args.output,p)
    np.savez_compressed(args.output/'plan.npz',**{f'q_{i}':s['q'] for i,s in enumerate(p)},**{f'p_{i}':s['p'] for i,s in enumerate(p)})
    if args.preview_only:return
    report,run=execute(m,p,args.output,stats);make_outputs(m,p,run,report,args.output,args.video)
if __name__=='__main__':main()
