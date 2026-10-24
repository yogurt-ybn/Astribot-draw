"""Exact serial-chain kinematics extracted from the compiled pinned MuJoCo model.

No collision, dynamics, or real hardware command is performed.
"""
from __future__ import annotations
import os,sys,xml.etree.ElementTree as ET
from pathlib import Path
os.environ.setdefault('MPLCONFIGDIR','/tmp/astribot-mpl')
os.environ.setdefault('MUJOCO_GL','egl');os.environ.setdefault('PYOPENGL_PLATFORM','egl')
os.environ.setdefault('LIBGL_ALWAYS_SOFTWARE','1');os.environ.setdefault('MESA_SHADER_CACHE_DISABLE','true')
import numpy as np
import mujoco
PIN='8c651a1abbe2821824a692aa3a3aa4cbf88e4122'
DEFAULT_ASSETS=Path(os.environ.get('ASTRIBOT_ASSETS', str(Path(__file__).resolve().parent/'descriptions')))
CAMERA=np.array([0.,-4.,.95]);PLANE_Y=-.20;FOV_Y=35.;WIDTH=960;HEIGHT=720

def expand(path):
    path=Path(path);root=ET.parse(path).getroot()
    def walk(node):
        for i,c in reversed(list(enumerate(node))):
            if c.tag=='include':
                r=expand(path.parent/c.get('file'));node.remove(c)
                for s in reversed(list(r)):node.insert(i,s)
            else:
                if c.tag in ('mesh','texture') and c.get('file'):c.set('file',str((path.parent/c.get('file')).resolve()))
                walk(c)
    walk(root);return root

def load_model(assets=DEFAULT_ASSETS, output=None):
    root=expand(Path(assets)/'mjcf/astribot_s1_mjcf/astribot_s1_chassis_fixed_with_gripper.xml')
    for s in ('left','right'):
        b=root.find(f".//body[@name='astribot_arm_{s}_link_7']")
        ET.SubElement(b,'site',name=f'{s}_light',pos='0 -.23 0')
    xml=ET.tostring(root,encoding='unicode')
    if output:Path(output).write_text(xml)
    return mujoco.MjModel.from_xml_string(xml)

def qmat(q):
    r=np.empty(9);mujoco.mju_quat2Mat(r,q);return r.reshape(3,3)

class Chain:
    def __init__(self,m,side,margin=0.):
        self.side=side;self.model=m
        self.jids=np.array([m.joint(f'astribot_arm_{side}_joint_{i}').id for i in range(1,8)])
        self.qids=m.jnt_qposadr[self.jids].copy();self.bids=m.jnt_bodyid[self.jids].copy()
        assert np.all(m.jnt_type[self.jids]==mujoco.mjtJoint.mjJNT_HINGE)
        assert np.max(abs(m.jnt_pos[self.jids]))==0
        assert np.allclose(m.jnt_axis[self.jids],[0,0,1])
        d=mujoco.MjData(m);d.qpos[:]=0;mujoco.mj_forward(m,d)
        parent=m.body_parentid[self.bids[0]]
        self.p0=d.xpos[parent].copy();self.R0=d.xmat[parent].reshape(3,3).copy()
        self.pos=m.body_pos[self.bids].copy();self.rot=np.array([qmat(q) for q in m.body_quat[self.bids]])
        self.limits=m.jnt_range[self.jids].copy();self.lo=self.limits[:,0]+margin;self.hi=self.limits[:,1]-margin
        self.reference_q=np.zeros(m.nq)
    def fk(self,q,offset=np.array([0.,-.23,0.]),jacobian=False):
        q=np.atleast_2d(np.asarray(q));n=len(q)
        p=np.broadcast_to(self.p0,(n,3)).copy();R=np.broadcast_to(self.R0,(n,3,3)).copy()
        origins=[];axes=[]
        for j in range(7):
            p=p+np.einsum('nij,j->ni',R,self.pos[j]);R=R@self.rot[j]
            if jacobian:origins.append(p.copy());axes.append(R[:,:,2].copy())
            c=np.cos(q[:,j]);s=np.sin(q[:,j]);r=R.copy()
            R[:,:,0]=r[:,:,0]*c[:,None]+r[:,:,1]*s[:,None]
            R[:,:,1]=-r[:,:,0]*s[:,None]+r[:,:,1]*c[:,None]
        tip=p+np.einsum('nij,j->ni',R,np.asarray(offset))
        if jacobian:
            jac=np.cross(np.array(axes).transpose(1,0,2),tip[:,None,:]-np.array(origins).transpose(1,0,2)).transpose(0,2,1)
            return tip,R,jac
        return tip,R
    def validate(self,q,offset=np.array([0.,-.23,0.])):
        d=mujoco.MjData(self.model);d.qpos[:]=0
        batch,R=self.fk(q,offset);errs=[];rerrs=[]
        for i,qq in enumerate(q):
            d.qpos[self.qids]=qq;mujoco.mj_forward(self.model,d)
            bid=self.bids[-1];rr=d.xmat[bid].reshape(3,3)
            p=d.xpos[bid]+rr@offset
            errs.append(np.linalg.norm(p-batch[i]));rerrs.append(np.max(abs(rr-R[i])))
        return {'configurations':len(q),'max_position_error_m':float(max(errs)),'max_rotation_entry_error':float(max(rerrs))}

def frame_metadata(m):
    d=mujoco.MjData(m);d.qpos[:]=0;mujoco.mj_forward(m,d)
    return {'source_commit':PIN,'world_axes':'world +X approx robot left; world -Y approx robot forward; +Z up; exact chassis yaw -1.57 rad',
            'world_origin':'ground under fixed chassis origin; chassis_base world position [0,0,0.097] m',
            'reference_qpos':{m.joint(i).name:0. for i in range(m.njnt)},
            'reference_description':'All non-sampled joints held at exactly 0 rad; torso joints 1-4 all 0, NOT joint-midrange neutral. Fixed chassis.',
            'chassis_position_m':d.body('chassis_base').xpos.tolist(),'chassis_rotation_world_from_body':d.body('chassis_base').xmat.reshape(3,3).tolist(),
            'collision_warning':'No collision rejection applied. Most robot mesh collision geometry is disabled. Position reachability only; not safe executable space.',
            'geom_count':int(m.ngeom),'geom_both_collision_bits_zero':int(np.sum((m.geom_contype==0)&(m.geom_conaffinity==0))),
            'endpoints':{'tool':'official arm_{left,right}_tool site; [0,-0.15,0] m in arm link_7 frame; nominal TCP, not literal finger surface',
                         'light':'prior harness left/right_light; [0,-0.23,0] m in arm link_7; 80 mm beyond official TCP'},
            'orientation':'unconstrained, existential orientation per position; not all orientations and not fixed orientation reachability'}
