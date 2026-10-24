from pathlib import Path
import json,numpy as np
from PIL import Image,ImageDraw
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'t1-results/wings';OUT.mkdir(parents=True,exist_ok=True)
p=np.load(ROOT/'wing-shape.npy');N=len(p);data={'schema':'astribot-t1-path-v1','name':'wings','width':616,'height':560,'coordinate_system':'continuous_pixel_edges','arms':{}}
im=Image.new('RGBA',(616,560),(255,255,255,0));dr=ImageDraw.Draw(im);svg=['<svg xmlns="http://www.w3.org/2000/svg" width="616" height="560" viewBox="0 0 616 560">'];checks={}
for side,sign,col in [('left',1,'#00649b'),('right',-1,'#914100')]:
 uv=np.c_[(sign*p[:,0]+.77)/.0025,(1.89-p[:,1])/.0025];data['arms'][side]={'points_uv':uv.tolist(),'progress':np.linspace(0,1,N).tolist()};dr.line([tuple(x) for x in uv],fill=col,width=2)
 svg.append('<path id="'+side+'" d="M '+' L '.join(f'{u:.6f},{v:.6f}' for u,v in uv)+'" fill="none" stroke="'+col+'" stroke-width="2"/>')
 mask=np.asarray(Image.open(ROOT/'t1-canvas'/f'mask-{side}.png'))>0;ij=np.floor(uv).astype(int);dist=np.linalg.norm(np.diff(uv,axis=0),axis=1)
 checks[side]={'points':N,'mask_contained':bool(mask[ij[:,1],ij[:,0]].all()),'max_segment_px':float(dist.max()),'min_segment_px':float(dist.min()),'bbox_size_m':np.ptp(p,axis=0).tolist(),'arc_length_m':float(np.linalg.norm(np.diff(p,axis=0),axis=1).sum())}
svg.append('</svg>');(OUT/'paths.svg').write_text('\n'.join(svg));im.save(OUT/'preview.png');(OUT/'paths.json').write_text(json.dumps(data,indent=2));(OUT/'validation.json').write_text(json.dumps({'design_checks':checks,'execution_status':'UNVALIDATED','note':'Design output alone does not certify robot execution. See separate results.json for measured execution.'},indent=2))
print(checks)
