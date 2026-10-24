#!/usr/bin/env python3
"""Replay measured states only; rendering does not run or alter execution."""
from pathlib import Path
import sys,os,json,subprocess
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT))
from kinematics import *
import light_painting as lp
from collision_model import render_collision_view
from PIL import Image,ImageDraw,ImageFont
FONT='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
CJK='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
NAMES={'tidal_waves':'WAVES','rounded_leaves':'ROUNDED LEAVES','twin_galaxies':'TWIN GALAXIES'}
def render(name):
 out=ROOT/'results'/name;data=np.load(out/'execution.npz');m=mujoco.MjModel.from_xml_path(str(out/'scene.collision.xml'));d=mujoco.MjData(m)
 q=data['qpos'];actual=data['actual'];phase=data['phase'];dt=float(data['dt']);desired=data['desired'];draw=phase=='draw_both'
 canvas=Image.open(ROOT/'inputs/cropped-regions.png').convert('RGB')
 font=ImageFont.truetype(FONT,20);small=ImageFont.truetype(FONT,15);n=120;fps=15;inds=np.linspace(0,len(q)-1,n).astype(int)
 cam=mujoco.MjvCamera();cam.lookat[:]=[0,-.06,1.05];cam.distance=2.9;cam.azimuth=115;cam.elevation=-10
 opt=mujoco.MjvOption();opt.geomgroup[:]=1
 d.qpos[:]=q[0];mujoco.mj_forward(m,d);z0=float(d.body('astribot_torso_link_4').xpos[2]);base0=d.body('chassis_base').xpos.copy()
 dest=out/'sync-replay.mp4';proc=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s','1200x680','-r',str(fps),'-i','-','-an','-c:v','libx264','-crf','21','-pix_fmt','yuv420p',str(dest)],stdin=subprocess.PIPE)
 with mujoco.Renderer(m,580,600) as r:
  for ni,k in enumerate(inds):
   d.qpos[:]=q[k];mujoco.mj_forward(m,d);r.update_scene(d,camera=cam,scene_option=opt);im=Image.new('RGB',(1200,680),(17,24,34));im.paste(Image.fromarray(r.render()),(0,50));dr=ImageDraw.Draw(im)
   dr.text((16,12),NAMES.get(name,name)+' | BOTH ARMS + SHARED TORSO',font=font,fill='white')
   ca=canvas.copy();cd=ImageDraw.Draw(ca)
   for si,col in enumerate([(0,100,155),(145,65,0)]):
    target=desired[draw,si];uv=lp.projection(target)-np.array([195,0]);cd.line([tuple(v) for v in uv],fill=(220,220,220),width=2)
    ix=draw[:k+1];uv=lp.projection(actual[:k+1,si][ix])-np.array([195,0])
    if len(uv)>1:cd.line([tuple(v) for v in uv],fill=col,width=3)
   im.paste(ca,(615,110));dr=ImageDraw.Draw(im);dz=(d.body('astribot_torso_link_4').xpos[2]-z0)*1000
   dr.text((615,62),f'Actual tip traces | t = {k*dt:.1f} s',font=font,fill='white')
   dr.text((615,603),f'Phase: {phase[k]}',font=font,fill='white')
   dr.text((20,630),f'Torso height change: {dz:+.1f} mm | Base fixed',font=small,fill='white')
   dr.text((615,639),'Recorded actuator dynamics; accelerated replay',font=small,fill=(195,205,215))
   proc.stdin.write(np.asarray(im).tobytes())
   if ni==60:im.save(out/'sync-preview.png');render_collision_view(m,d,out/'collision-hulls.png')
 proc.stdin.close();assert proc.wait()==0
 print(str(dest),flush=True)
 return dest

def combine(paths):
 if not paths:return
 f=ROOT/'replay-concat.txt';f.write_text(''.join("file '"+str(p)+"'\n" for p in paths));out=ROOT/'synchronized-three-designs.mp4';subprocess.run(['ffmpeg','-y','-loglevel','error','-f','concat','-safe','0','-i',str(f),'-c','copy',str(out)],check=True);print(out)
if __name__=='__main__':
 names=sys.argv[1:] or ['tidal_waves','rounded_leaves','twin_galaxies'];paths=[render(n) for n in names];combine(paths)
