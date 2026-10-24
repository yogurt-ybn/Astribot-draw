from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont
R=Path(__file__).resolve().parent;out=R/'t1-results/wings';d=np.load(out/'execution.npz');draw=d['phase']=='draw_both'
im=Image.new('RGB',(1232,640),'white');font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',18)
for k,kind in enumerate(['desired','actual']):
 ca=Image.open(R/'t1-canvas/drawing-regions.png').convert('RGB');cd=ImageDraw.Draw(ca)
 for si,col in enumerate(['#00649b','#914100']):
  p=d[kind][draw,si];uv=np.c_[(p[:,0]+.77)/.0025,(1.89-p[:,2])/.0025];cd.line(list(map(tuple,uv)),fill=col,width=3)
 im.paste(ca,(616*k,50));ImageDraw.Draw(im).text((616*k+15,15),'Requested wing paths' if k==0 else 'Recorded actuator execution',font=font,fill='black')
ImageDraw.Draw(im).text((15,615),'T1 | shared torso + simultaneous arms | fixed chassis | no human',font=font,fill='black');im.save(out/'requested-vs-actual.png')
