#!/usr/bin/env python3
"""Uniform postprocessor. Call only after benchmark workers mark inputs stable.
No planning, no geometry mutation, no dynamics execution. Robot video is measured-qpos replay.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
os.environ.setdefault('PYOPENGL_PLATFORM','egl')
os.environ.setdefault('LIBGL_ALWAYS_SOFTWARE','1')
from pathlib import Path
import argparse,hashlib,json,subprocess,io
import numpy as np
from PIL import Image,ImageDraw,ImageFont
import cairosvg
FONT='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
def font(s):return ImageFont.truetype(FONT,s)
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def label(im,xy,t,s=18,c='white'):ImageDraw.Draw(im).text(xy,t,font=font(s),fill=c)
def uv(x):return np.c_[(x[:,0]+.77)/.0025,(1.89-x[:,2])/.0025]
def panel(source,actual=None,phase=None,k=None):
    im=source.copy().convert('RGB')
    if actual is not None:im=Image.blend(im.convert('L').convert('RGB'),Image.new('RGB',im.size,'white'),0.65)
    d=ImageDraw.Draw(im)
    if actual is not None:
        end=len(actual) if k is None else k+1
        sel=phase[:end]=='draw_both'
        for si,col in enumerate([(0,115,190),(225,110,0)]):
            pts=uv(actual[:end,si][sel])
            if len(pts)>1:d.line(list(map(tuple,pts)),fill=col,width=2)
    return im

def main():
    ap=argparse.ArgumentParser();ap.add_argument('run',type=Path);ap.add_argument('--inputs',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);ap.add_argument('--robot',action='store_true',help='Requires exclusive model-memory slot.');ap.add_argument('--video',action='store_true');ap.add_argument('--cases',nargs='+',help='Only worker-confirmed stable cases');a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    freeze=json.loads((a.run/'freeze.json').read_text());meta={'run':str(a.run),'render_source_sha256':sha(Path(__file__)),'video_seconds_per_success':8,'fps':15,'canvas_projection':'u=(x+0.77)/0.0025; v=(1.89-z)/0.0025; projected actual tip xyz, not detected pixels','scope':'Measured execution only. No force control/hardware certification. Same framing and scale for all cases.','cases':[]};cards=[];videos=[]
    for filename,expected in freeze['input_sha256'].items():
        src=a.inputs/filename;name=src.stem
        if a.cases and name not in a.cases:continue
        out=a.run/name;h=sha(src);assert h==expected
        report=json.loads((out/'report.json').read_text()) if (out/'report.json').exists() else {};stages=report.get('stages',{});success=stages.get('simulation')=='passed';source=Image.open(io.BytesIO(cairosvg.svg2png(url=str(src),output_width=616,output_height=560))).convert('RGBA');white=Image.new('RGBA',source.size,'white');white.alpha_composite(source);source=white.convert('RGB')
        item={'name':name,'input_sha256':h,'stages':stages,'actual_execution':success,'video_generated':bool(success and a.video),'robot_state_replay':bool(success and a.video and a.robot),'report_sha256':sha(out/'report.json') if report else None};actual=None;phase=None
        if success:
            assert report.get('input_sha256')==h
            ex=out/'execution.npz';item['execution_sha256']=sha(ex)
            data=np.load(ex);actual=data['actual'];phase=data['phase'];q=data['qpos'];dt=float(data['dt']);assert len(actual)==len(q)==len(phase);assert np.isfinite(actual).all()
            duration=len(q)*dt;item.update(simulation_seconds=duration,video_seconds=8,playback_speed=duration/8,actual_frames=len(q),tracking=report['simulation'].get('arms'))
        card=Image.new('RGB',(1280,750),(20,27,36));label(card,(20,15),name.upper()+' | ORIGINAL SVG / ACTUAL TIP TRACE',24);label(card,(20,51),'SHA256 '+h,15)
        card.paste(source,(16,120));card.paste(panel(source,actual,phase),(648,120));label(card,(20,85),'Original SVG (unchanged)',18);label(card,(650,85),'Measured drawing overlay' if success else 'NO SUCCESSFUL EXECUTION - source only',18)
        label(card,(20,690),' / '.join(f'{s}: {stages.get(s,"not_run")}' for s in ['static','ik','full_plan','simulation']),16)
        label(card,(20,720),f'{duration:.3f} s simulation | Blue: left arm, orange: right arm' if success else str(report.get('error',{}).get('message','No completed report'))[:135],14)
        card.save(a.out/f'{name}-comparison.png');cards.append(card)
        if success and a.video:
            path=a.out/f'{name}-replay.mp4';proc=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s','1200x680','-r','15','-i','-','-an','-c:v','libx264','-crf','25','-pix_fmt','yuv420p',str(path)],stdin=subprocess.PIPE)
            renderer=None
            if a.robot:
                import mujoco
                m=mujoco.MjModel.from_binary_path(str(out/'scene.t1.mjb'));d=mujoco.MjData(m);renderer=mujoco.Renderer(m,540,600);cam=mujoco.MjvCamera();cam.lookat[:]=[0,-.06,1.05];cam.distance=2.9;cam.azimuth=115;cam.elevation=-10;opt=mujoco.MjvOption();opt.geomgroup[:]=1
            for k in np.linspace(0,len(q)-1,120).astype(int):
                im=Image.new('RGB',(1200,680),(20,27,36));label(im,(16,12),name.upper()+' / RECORDED SIMULATION',23)
                if renderer:
                    d.qpos[:]=q[k];mujoco.mj_forward(m,d);renderer.update_scene(d,camera=cam,scene_option=opt);im.paste(Image.fromarray(renderer.render()),(0,70))
                else:im.paste(source.resize((554,504)),(16,90));label(im,(16,55),'Original SVG',18)
                im.paste(panel(source,actual,phase,k).resize((554,504)),(626,90));label(im,(626,55),'Actual tip trace (draw phase only)',18)
                label(im,(626,607),f't={dt*(k+1):.2f} / {duration:.2f} s | {duration/8:.2f}x',17);label(im,(626,633),f'Phase: {phase[k]}',17);label(im,(16,651),'SHA256 '+h[:24]+'... | No hardware safety claim',13)
                proc.stdin.write(np.asarray(im).tobytes())
            proc.stdin.close();assert proc.wait()==0
            if renderer:renderer.close();del d,m,renderer
            videos.append(path)
        meta['cases'].append(item)
    sheet=Image.new('RGB',(1280,750*len(cards)),(20,27,36))
    for i,c in enumerate(cards):sheet.paste(c,(0,750*i))
    sheet.save(a.out/'three-design-comparison.jpg',quality=88)
    if videos:
        listing=a.out/'concat.txt';listing.write_text(''.join(f"file '{v.resolve()}'\n" for v in videos));subprocess.run(['ffmpeg','-y','-loglevel','error','-f','concat','-safe','0','-i',str(listing),'-c','copy',str(a.out/'combined-replay.mp4')],check=True)
    (a.out/'render-metadata.json').write_text(json.dumps(meta,indent=2));print(json.dumps(meta,indent=2))
if __name__=='__main__':main()
