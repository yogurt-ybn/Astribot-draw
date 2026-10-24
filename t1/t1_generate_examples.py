"""Generate native vector/raster/JSON examples, never infer execution from raster."""
from pathlib import Path
import json,numpy as np
from PIL import Image,ImageDraw
from t1_sync import target_paths
ROOT=Path(__file__).resolve().parent
for name in ['waves','leaves','spiral']:
 out=ROOT/'t1-results'/name;out.mkdir(parents=True,exist_ok=True)
 record={'schema':'astribot-t1-path-v1','width':616,'height':560,'coordinate_system':'continuous_pixel_edges','arms':{}}
 preview=Image.new('RGB',(616,560),'white');draw=ImageDraw.Draw(preview);paths=[];checks={}
 for side,col,xyz in zip(['left','right'],['#00649b','#914100'],target_paths(name,2001)):
  uv=np.c_[(xyz[:,0]+.77)/.0025,(1.89-xyz[:,2])/.0025];ij=np.floor(uv).astype(int);mask=np.asarray(Image.open(ROOT/'t1-canvas'/f'mask-{side}.png').convert('L'));inside=bool(np.all(mask[ij[:,1],ij[:,0]]>0));seg=np.linalg.norm(np.diff(uv,axis=0),axis=1)
  assert inside and seg.min()>0 and seg.max()<=1
  record['arms'][side]={'points_uv':uv.tolist(),'progress':np.linspace(0,1,len(uv)).tolist()}
  paths.append(f'<path id="{side}" d="M '+ ' L '.join(f'{u:.5f},{v:.5f}' for u,v in uv)+f'" fill="none" stroke="{col}" stroke-width="2"/>')
  draw.line([tuple(p) for p in uv],fill=col,width=2)
  checks[side]={'inside_assigned_mask':inside,'max_segment_pixels':float(seg.max()),'points':len(uv),'continuous':True}
 preview.save(out/'preview.png');(out/'paths.svg').write_text('<svg xmlns="http://www.w3.org/2000/svg" width="616" height="560" viewBox="0 0 616 560">'+''.join(paths)+'</svg>')
 (out/'paths.json').write_text(json.dumps(record));(out/'validation.json').write_text(json.dumps({'checks':checks,'execution_status':'UNVALIDATED','note':'Geometry export only; see separate results.json for measured simulation of corresponding continuous curves.'},indent=2))
print('Generated and validated three native examples.')
