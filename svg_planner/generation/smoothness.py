"""v1.1 design prefilter, not a physical or continuous curvature certificate."""
import math
import xml.etree.ElementTree as ET
import numpy as np
from svg_input import _tokens

def segments(d):
    ts=_tokens(d); i=0; cmd=None; p=np.zeros(2); start=None; result=[]; closed=False
    while i<len(ts):
        if ts[i].isalpha(): cmd=ts[i]; i+=1
        op=cmd.upper()
        if op=='Z':
            if np.linalg.norm(p-start)>1e-9: result.append(np.array([p,start]))
            closed=True; break
        count={'M':2,'L':2,'Q':4,'C':6}[op]
        vals=np.array([float(x) for x in ts[i:i+count]]).reshape(-1,2); i+=count
        if cmd.islower(): vals=vals+p
        if op=='M': start=vals[0].copy(); cmd='l' if cmd.islower() else 'L'
        else: result.append(np.vstack([p,vals]))
        p=vals[-1].copy()
    return result,closed

def check_smoothness(source,max_join_deg=5.,max_curvature_per_px=.1):
    root=ET.parse(source).getroot()
    report={'status':'PASS','contract':'v1.1','scope':'finite analytic-derivative geometric design prefilter, not motion feasibility',
            'max_join_angle_deg':max_join_deg,'curvature_limit_per_px':max_curvature_per_px,
            'samples_per_segment':1025,'arms':{}}
    for path in root:
        side=path.get('data-arm',path.get('id')); segs,closed=segments(path.get('d',''))
        tangents=[]; ks=[]; errors=[]
        t=np.linspace(0,1,1025)[:,None]
        for index,c in enumerate(segs):
            if len(c)==2:
                d=np.repeat((c[1]-c[0])[None,:],len(t),axis=0); dd=np.zeros_like(d)
            elif len(c)==3:
                d=2*((1-t)*(c[1]-c[0])+t*(c[2]-c[1])); dd=np.repeat((2*(c[2]-2*c[1]+c[0]))[None,:],len(t),axis=0)
            else:
                d=3*((1-t)**2*(c[1]-c[0])+2*(1-t)*t*(c[2]-c[1])+t*t*(c[3]-c[2]))
                dd=6*((1-t)*(c[2]-2*c[1]+c[0])+t*(c[3]-2*c[2]+c[1]))
            speed=np.linalg.norm(d,axis=1)
            if np.any(speed<=1e-9):
                errors.append(f'segment {index}: degenerate/zero derivative at sampled parameter'); ks.append(None)
            else:
                curvature=np.abs(d[:,0]*dd[:,1]-d[:,1]*dd[:,0])/speed**3
                k=float(np.max(curvature)); ks.append(k)
                if k>max_curvature_per_px+1e-10: errors.append(f'segment {index}: sampled curvature {k:.6g} > {max_curvature_per_px} per px')
            tangents.append((d[0],d[-1]))
        joins=[]
        pairs=[(i,i+1) for i in range(len(segs)-1)]
        if closed and segs:pairs.append((len(segs)-1,0))
        for i,j in pairs:
            a,b=tangents[i][1],tangents[j][0]; na,nb=np.linalg.norm(a),np.linalg.norm(b)
            if min(na,nb)<=1e-9: angle=None
            else:
                angle=math.degrees(math.atan2(abs(float(a[0]*b[1]-a[1]*b[0])),float(np.dot(a,b))))
                if angle>max_join_deg+1e-8: errors.append(f'join {i}->{j}: tangent angle {angle:.6g} > {max_join_deg} deg')
            joins.append({'from':i,'to':j,'angle_deg':angle})
        report['arms'][side]={'status':'FAIL' if errors else 'PASS','closed':closed,'segments':len(segs),
            'max_sampled_curvature_per_px':max((k for k in ks if k is not None),default=None),
            'joins':joins,'errors':errors}
        if errors:report['status']='FAIL'
    return report
