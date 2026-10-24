"""Serial-chain FK extracted from compiled T1 model (all seven arm axes local Z)."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
os.environ.setdefault('PYOPENGL_PLATFORM','egl')
os.environ.setdefault('LIBGL_ALWAYS_SOFTWARE','1')
import numpy as np
import mujoco
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
