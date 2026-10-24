#!/usr/bin/env python3
"""Static design checks, never a robot execution certificate."""
from pathlib import Path
import sys,json
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
from drawing_contract import ROOT,SCALE,render

def intersections(p,closed):
    # Bounding boxes first, then robust segment test. Adjacent shared vertices excluded.
    a=p[:-1];b=p[1:];lo=np.minimum(a,b);hi=np.maximum(a,b);n=len(a)
    def cross(x,y):return x[...,0]*y[...,1]-x[...,1]*y[...,0]
    for i in range(n):
        js=np.arange(i+2,n)
        if closed and i==0:js=js[js!=n-1]
        if not len(js):continue
        ids=js[np.all(hi[i]>=lo[js]-1e-9,axis=1)&np.all(hi[js]>=lo[i]-1e-9,axis=1)]
        if not len(ids):continue
        c=a[ids];d=b[ids]
        x=cross(b[i]-a[i],c-a[i]);y=cross(b[i]-a[i],d-a[i]);z=cross(d-c,a[i]-c);w=cross(d-c,b[i]-c)
        hit=(x*y<=1e-12)&(z*w<=1e-12)
        if hit.any():return [int(i),int(ids[np.flatnonzero(hit)[0]])]
    return None

def validate(folder):
    folder=Path(folder);errors=[];metrics={}
    def ck(ok,msg):
        if not ok:errors.append(msg)
    data=json.loads((folder/'paths.json').read_text())
    ck(data.get('schema_version')=='astribot-drawing-v2','schema_version')
    c=data.get('canvas',{})
    ck(c.get('width')==570 and c.get('height')==471,'canvas size')
    ck(c.get('coordinate_system')=='continuous_pixel_edges','pixel coordinate convention')
    ck(c.get('crop_offset_original')==[195,0],'crop offset')
    ck(abs(c.get('meters_per_pixel',0)-SCALE)<1e-15,'metric scale')
    e=data.get('execution',{})
    expected_execution=json.loads((ROOT/'execution-contract.json').read_text())
    ck(e == expected_execution,'synchronized execution metadata')
    strokes=data.get('strokes',[])
    ck(len(strokes)==2 and [s.get('arm') for s in strokes]==['left','right'],'exactly left and right strokes in canonical storage order (not temporal order)')
    union=np.zeros((471,570),bool)
    for st in strokes:
        arm=st.get('arm');p=np.asarray(st.get('points',[]),float)
        if arm not in ['left','right']:continue
        ck(p.ndim==2 and p.shape[1:]==(2,) and 50<=len(p)<=5000,arm+' point shape/count')
        if p.ndim!=2 or p.shape[1:]!=(2,) or len(p)<3:continue
        ck(bool(np.isfinite(p).all()),arm+' finite')
        if not np.isfinite(p).all():continue
        mask=np.array(Image.open(ROOT/'inputs'/('mask-'+arm+'.png')))>0;union|=mask
        dist=distance_transform_edt(np.pad(mask,1))[1:-1,1:-1]
        delta=np.diff(p,axis=0);ds=np.linalg.norm(delta,axis=1)
        ck(bool((ds>1e-6).all()),arm+' duplicate points')
        ck(float(ds.max())<=1.000001,arm+' max spacing')
        length=float(ds.sum()*SCALE);ck(.30<=length<=4.,arm+' length 0.30–4.0m')
        closed=st.get('closed');ck(isinstance(closed,bool),arm+' closed must be boolean')
        ck(bool(np.linalg.norm(p[-1]-p[0])<1e-6)==bool(closed),arm+' closure metadata')
        dirs=delta/np.maximum(ds[:,None],1e-15)
        if closed:dirs=np.vstack([dirs,dirs[0]]);lens=np.r_[ds,ds[0]]
        else:lens=ds
        angles=np.arccos(np.clip((dirs[:-1]*dirs[1:]).sum(1),-1,1));curv=angles/np.maximum((lens[:-1]+lens[1:])/2,1e-15)
        ck(float(angles.max())<=np.deg2rad(12)+1e-8,arm+' turning angle')
        ck(float(curv.max())<=.100001,arm+' curvature')
        dense=[]
        for a,b,l in zip(p[:-1],p[1:],ds):dense.extend(a+np.linspace(0,1,max(2,int(np.ceil(l/.25))+1))[:,None]*(b-a))
        dense=np.array(dense);idx=np.floor(dense).astype(int)
        inside=(idx[:,0]>=0)&(idx[:,0]<570)&(idx[:,1]>=0)&(idx[:,1]<471);ck(bool(inside.all()),arm+' image bounds')
        md=float(dist[idx[inside,1],idx[inside,0]].min()) if inside.any() else 0.
        ck(md>=15.,arm+' margin >=15px')
        crossing=intersections(p,bool(closed));ck(crossing is None,arm+' self intersection '+str(crossing))
        metrics[arm]={'points':len(p),'length_m':length,'max_spacing_px':float(ds.max()),'min_mask_margin_px':md,'max_turn_deg':float(np.rad2deg(angles.max())),'max_curvature_rad_per_px':float(curv.max()),'self_intersection_segments':crossing}
    im=Image.open(folder/'drawing.png');ck(im.mode=='RGBA' and im.size==(570,471),'PNG mode/size')
    if im.mode=='RGBA' and im.size==(570,471):
        a=np.array(im);ck(bool((a[~union,3]==0).all()),'all nonzero alpha inside assigned union')
        ck(bool((a[a[:,:,3]>0,:3]==255).all()),'white strokes')
        expected=np.array(render(data));ck(bool(np.array_equal(a,expected)),'PNG matches canonical rendering from JSON')
    result={'design_pass':not errors,'robot_execution_status':'not_tested','errors':errors,'metrics':metrics,'scope':'Static format, ordered geometry, margin, smoothness and raster checks only. No IK, collision, dynamics or hardware certificate.'}
    (folder/'validation.json').write_text(json.dumps(result,ensure_ascii=False,indent=2));return result
if __name__=='__main__':
    try:r=validate(sys.argv[1])
    except Exception as exc:r={'design_pass':False,'robot_execution_status':'not_tested','errors':[str(exc)]}
    print(json.dumps(r,ensure_ascii=False,indent=2));sys.exit(0 if r['design_pass'] else 1)
