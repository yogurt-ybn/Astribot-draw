from t1_kinematics import *
def solve(c,p,q):
 q=q.copy();q[4:]=np.clip(q[4:],-.4,.4)
 for _ in range(160):
  xyz,_,J=c.fk(q,jacobian=True);err=p-xyz[0]
  if np.linalg.norm(err)<3e-5:break
  j=J[0];ji=j.T@np.linalg.inv(j@j.T+np.eye(3)*1e-5);post=np.r_[np.zeros(4),-.15*q[4:]];dq=ji@err+(np.eye(7)-ji@j)@post;q=np.clip(q+np.clip(dq,-.05,.05),c.lo,c.hi);q[4:]=np.clip(q[4:],-.4,.4)
 return q,float(np.linalg.norm(c.fk(q)[0][0]-p))


def rrt_transfer(m,c,start,goal,torso,home,sub,rng):
 d=mujoco.MjData(m)
 def valid(q):
  d.qpos[:]=home;d.qpos[:4]=torso;d.qpos[c.qids]=q;mujoco.mj_forward(m,d)
  return all(mujoco.mj_geomDistance(m,d,a,b,.05,None)>=.003 for a,b in sub)
 def edge(a,b):
  return all(valid(a+r*(b-a)) for r in np.linspace(0,1,max(2,int(np.ceil(np.max(abs(b-a))/.01))+1)))
 if edge(start,goal):return [start,goal]
 trees=[([start.copy()],[-1]),([goal.copy()],[-1])]
 def grow(tree,q):
  nodes,parents=tree;ix=int(np.argmin([np.linalg.norm(x-q) for x in nodes]));a=nodes[ix];v=q-a;dist=np.linalg.norm(v);b=q if dist<.25 else a+v*.25/dist
  if not edge(a,b):return None
  nodes.append(b.copy());parents.append(ix);return len(nodes)-1
 def back(tree,k):
  out=[]
  while k>=0:out.append(tree[0][k]);k=tree[1][k]
  return out[::-1]
 for it in range(1500):
  k=it%2; q=goal if rng.random()<.15 else np.clip(start+rng.random()*(goal-start)+rng.normal(0,.7,len(start)),c.lo,c.hi)
  ia=grow(trees[k],q)
  if ia is None:continue
  q=trees[k][0][ia]
  for _ in range(70):
   ib=grow(trees[1-k],q)
   if ib is None:break
   if np.linalg.norm(trees[1-k][0][ib]-q)<1e-8:
    a=back(trees[k],ia);b=back(trees[1-k],ib);path=a+b[-2::-1]
    if k:path=path[::-1]
    # deterministic shortcut, preserving checked segments
    slim=[path[0]];i=0
    while i<len(path)-1:
     j=len(path)-1
     while j>i+1 and not edge(path[i],path[j]):j-=1
     slim.append(path[j]);i=j
    print('RRT_PASS',c.side,it,len(slim),flush=True);return slim
 print('RRT_FAIL',c.side,flush=True);raise RuntimeError('No checked entry path')
