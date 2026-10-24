#!/usr/bin/env python3
"""Canonical renderer. Points use edge coordinates; sample centers are n+0.5."""
from pathlib import Path
import json
import numpy as np
from PIL import Image,ImageDraw
SCALE=0.003328153882611492
ROOT=Path(__file__).resolve().parent

def render(data):
    ss=4
    im=Image.new('L',(570*ss,471*ss),0);d=ImageDraw.Draw(im)
    for stroke in data['strokes']:
        xy=np.asarray(stroke['points'],float)*ss-.5
        d.line([tuple(p) for p in xy],fill=255,width=12,joint='curve')
        for p in xy:
            d.ellipse((p[0]-5.5,p[1]-5.5,p[0]+5.5,p[1]+5.5),fill=255)
    alpha=im.resize((570,471),Image.Resampling.LANCZOS)
    out=Image.new('RGBA',(570,471),(255,255,255,0));out.putalpha(alpha)
    return out

def save_design(data,folder):
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
    (folder/'paths.json').write_text(json.dumps(data,ensure_ascii=False,indent=2))
    pic=render(data);pic.save(folder/'drawing.png')
    bg=Image.open(ROOT/'inputs/cropped-regions.png').convert('RGBA');bg.alpha_composite(pic)
    draw=ImageDraw.Draw(bg)
    for st in data['strokes']:
        for p,col in [(st['points'][0],(0,190,60)),(st['points'][-1],(230,40,50))]:
            u,v=p;draw.ellipse((u-4,v-4,u+4,v+4),fill=col)
    bg.convert('RGB').save(folder/'preview.png')
    return folder
