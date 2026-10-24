#!/usr/bin/env python3
import copy,json,tempfile
from pathlib import Path
import numpy as np
from PIL import Image
from drawing_contract import save_design,SCALE
from validate_drawing import validate

def baseline():
    t=np.linspace(0,2*np.pi,501)
    return {'schema_version':'astribot-drawing-v2','name':'test_circle','canvas':{'width':570,'height':471,'coordinate_system':'continuous_pixel_edges','meters_per_pixel':SCALE,'crop_offset_original':[195,0]},'execution':json.loads((Path(__file__).parent/'execution-contract.json').read_text()),'strokes':[{'arm':arm,'closed':True,'points':np.c_[cx+40*np.cos(t),235+40*np.sin(t)].tolist()} for arm,cx in [('left',430),('right',140)]]}

def main():
    outcomes={}
    with tempfile.TemporaryDirectory(prefix='drawing-validator-') as tmp:
        def case(name,mutate=None,png_mutate=None,expect=False):
            data=baseline()
            if mutate:mutate(data)
            folder=save_design(data,Path(tmp)/name)
            if png_mutate:png_mutate(folder/'drawing.png')
            r=validate(folder);outcomes[name]={'pass':r['design_pass'],'expected':expect,'errors':r['errors']}
            assert r['design_pass']==expect,(name,r)
        case('valid_circle',expect=True)
        case('sequential_metadata',lambda d:d['execution'].update(simultaneous=False))
        case('missing_sync_contract',lambda d:d['execution'].pop('progress'))
        case('wrong_png_dimensions',png_mutate=lambda p:Image.open(p).resize((571,471)).save(p))
        case('wrong_png_mode',png_mutate=lambda p:Image.open(p).convert('RGB').save(p))
        case('swapped_arms',lambda d: [s.update(arm=a) for s,a in zip(d['strokes'],['right','left'])])
        case('outside_mask',lambda d:d['strokes'][0]['points'].__setitem__(100,[10,10]))
        case('discontinuous_step',lambda d:d['strokes'][0]['points'].__setitem__(100,[450,200]))
        case('repeated_point',lambda d:d['strokes'][0]['points'].__setitem__(100,d['strokes'][0]['points'][99]))
        def sharp(d):
            p=np.array(d['strokes'][0]['points']);p[100]+=np.array([.3,-.4]);d['strokes'][0]['points']=p.tolist()
        case('sharp_corner',sharp)
        def cross(d):
            t=np.linspace(0,2*np.pi,1001);d['strokes'][0]['points']=np.c_[430+40*np.sin(t),235+25*np.sin(2*t)].tolist()
        case('self_intersection',cross)
        def pixel(p):
            im=Image.open(p);im.putpixel((430,235),(255,255,255,255));im.save(p)
        case('raster_disagrees_json',png_mutate=pixel)
    report={'all_expected_results':True,'cases':outcomes}
    (Path(__file__).parent/'validator-selftest.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
if __name__=='__main__':main()
