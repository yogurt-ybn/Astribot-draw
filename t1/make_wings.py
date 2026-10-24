from pathlib import Path
import numpy as np
from scipy.interpolate import CubicSpline
from PIL import Image,ImageDraw
ROOT=Path(__file__).resolve().parent
# single unbroken outer arch and feather-like lower edge
knots=np.array([[.16,1.08],[.20,1.32],[.37,1.52],[.62,1.57],[.60,1.43],[.49,1.34],[.58,1.30],[.52,1.23],[.40,1.18],[.49,1.13],[.40,1.04],[.25,.98],[.17,.94]])
t=np.r_[0,np.cumsum(np.linalg.norm(np.diff(knots,axis=0),axis=1))];f=CubicSpline(t,knots,bc_type='natural');p=f(np.linspace(0,t[-1],10001));arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(p,axis=0),axis=1))]
u=np.linspace(0,arc[-1],1001);p=np.c_[np.interp(u,arc,p[:,0]),np.interp(u,arc,p[:,1])]
np.save(ROOT/'wing-shape.npy',p)
im=Image.open(ROOT/'t1-canvas/drawing-regions.png').convert('RGB');d=ImageDraw.Draw(im)
for s,c in [(1,'white'),(-1,'white')]:
 uv=np.c_[(s*p[:,0]+.77)/.0025,(1.89-p[:,1])/.0025];d.line([tuple(x) for x in uv],fill=c,width=3)
im.save(ROOT/'wing-design.png');print('bbox',p.min(0),p.max(0),'length',arc[-1])
