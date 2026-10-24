"""Official T1 fixed-base model and explicitly enumerated collision coverage.
Assets are local only; no upstream data redistribution is implied.
"""
from pathlib import Path
import os,json,re,copy,xml.etree.ElementTree as ET
os.environ.setdefault('MUJOCO_GL','egl')
import mujoco
import numpy as np
ROOT=Path(__file__).resolve().parent
ASSETS=Path(os.environ.get('ASTRIBOT_T1_ASSETS',str(ROOT/'descriptions')))

def expand(path):
 root=ET.parse(path).getroot()
 def visit(node):
  for i,ch in reversed(list(enumerate(list(node)))):
   if ch.tag=='include':
    inc=expand((path.parent/ch.get('file')).resolve());node.remove(ch)
    for sub in reversed(list(inc)):node.insert(i,sub)
   else:
    if ch.tag in ('mesh','texture') and ch.get('file'):ch.set('file',str((path.parent/ch.get('file')).resolve()))
    visit(ch)
 visit(root);return root

def label(name):
 s='left' if 'left' in name else 'right' if 'right' in name else None
 if not s:return None,None
 mat=re.search(r'arm_'+s+r'_link_(\d+)$',name)
 return s,int(mat.group(1)) if mat else (9 if 'wand' in name else 8)

def classify(m,a,b,exclude_compound_elbow=False):
 if m.geom(a).name.startswith('visual_') or m.geom(b).name.startswith('visual_'):return None,'visual-only geometry; explicit collision proxy used'
 ba,bb=int(m.geom_bodyid[a]),int(m.geom_bodyid[b]);na,nb=m.body(ba).name,m.body(bb).name
 sa,ia=label(na);sb,ib=label(nb)
 if ba==bb:return None,'same body'
 if sa and sb:
  if sa!=sb:return 'inter_arm',None
  if {ia,ib}=={5,8}:
   other=b if ia==5 else a;ob=bb if ia==5 else ba
   if m.body(ob).name==f'astribot_gripper_{sa}_base' and m.geom_type[other]==mujoco.mjtGeom.mjGEOM_MESH and m.mesh(int(m.geom_dataid[other])).name=='gripper_base_link_collision':
    return None,'T1 compound wrist mechanical interface: forearm5 vs official gripper base collision mesh only; pads/fingers/wand remain checked'
  if abs(ia-ib)<=1:return None,'adjacent arm/tool assembly'
  if ia>=7 and ib>=7:return None,'wrist/gripper/light rigid attachment and internal gripper assembly'
  if {ia,ib}=={5,7}:return None,'T1 wrist6/7 zero-offset compound articulation'
  if exclude_compound_elbow and {ia,ib}=={3,5}:return None,'OPTIONAL T1 adjacent upperarm/forearm across compound elbow4/5; local folding collision NOT covered'
  return 'same_arm_nonadjacent',None
 if not(sa or sb):
  if ba==0 or bb==0:return None,'non-arm ground support'
  if m.body_weldid[ba]==m.body_weldid[bb]:return None,'same fixed assembly'
  if m.body_parentid[ba]==bb or m.body_parentid[bb]==ba:return None,'direct parent-child mechanical attachment'
  if {na,nb}=={'astribot_torso_link_2','astribot_torso_link_4'}:return None,'T1 torso3/4 share zero-offset compound attachment'
  if {na,nb}=={'astribot_torso_link_4','astribot_head_link_2'}:return None,'T1 head1/2 share zero-offset compound mount'
  return 'torso_head_base',None
 side,ai,ob,og=(sa,ia,bb,b) if sa else (sb,ib,ba,a)
 if ai in (1,2) and m.body(ob).name=='astribot_torso_link_4':
  mesh=m.mesh(int(m.geom_dataid[og])).name.removesuffix('_collision') if m.geom_type[og]==mujoco.mjtGeom.mjGEOM_MESH else ''
  if mesh in ('astribot_torso_link_4',f'astribot_arm_{side}_base_link'):return None,'T1 shoulder1/2 common pivot and ipsilateral mount'
 return ('arm_environment' if ob==0 else 'arm_body'),None

def collision_pairs(m):
 return [(int(m.pair_geom1[i]),int(m.pair_geom2[i]),classify(m,int(m.pair_geom1[i]),int(m.pair_geom2[i]))[0]) for i in range(m.npair)]
def check_distances(m,d,pairs=None,max_distance=.05):
 pairs=collision_pairs(m) if pairs is None else pairs;best={'distance_m':max_distance,'minimum_distance_m':max_distance,'geom1':None,'geom2':None,'category':None};cats={}
 for a,b,cat in pairs:
  dist=float(mujoco.mj_geomDistance(m,d,a,b,max_distance,None))
  rec={'distance_m':dist,'geom1':m.geom(a).name,'geom2':m.geom(b).name}
  if cat not in cats or dist<cats[cat]['distance_m']:cats[cat]=rec
  if dist<best['distance_m']:best.update(rec,minimum_distance_m=dist,category=cat)
 best['category_minima']=cats;return best
class CollisionChecker:
 def __init__(self,m):self.m=m;self.pairs=collision_pairs(m)
 def check(self,d):return check_distances(self.m,d,self.pairs)

def build_t1_scene(out,assets=None,collision=True,exclude_compound_elbow=True):
 out=Path(out);out.mkdir(parents=True,exist_ok=True);assets=Path(assets or ASSETS)
 root=expand(assets/'astribot_t1/model/mjcf/robot/t1.xml');root.set('model','astribot_t1_fixed_base_drawing')
 compiler=root.find('compiler');compiler.attrib.pop('meshdir',None)
 for parent in root.iter():
  for ch in list(parent):
   if ch.tag=='freejoint' or (ch.tag=='joint' and ch.get('name','').startswith('wheel_')):parent.remove(ch)
   elif ch.tag in ('velocity','general') and ch.get('name','').startswith('astribot_chassis_motor_'):parent.remove(ch)
 # Existing wheel geometry becomes static with fixed base. No floating dynamics.
 chassis=root.find(".//body[@name='chassis_base']");chassis.set('euler','0 0 -1.5707963267948966')
 world=root.find('worldbody')
 ET.SubElement(world,'geom',name='floor',type='plane',size='4 4 .1',rgba='.07 .09 .12 1',contype='0',conaffinity='0')
 ET.SubElement(world,'light',pos='1 -2 3',dir='-.2 .3 -1',diffuse='.8 .8 .8',ambient='.2 .2 .2')
 ET.SubElement(world,'camera',name='photography',pos='0 -3 1',xyaxes='1 0 0 0 0 1',fovy='40')
 vis=ET.SubElement(root,'visual');ET.SubElement(vis,'global',offwidth='960',offheight='720');ET.SubElement(vis,'headlight',ambient='.35 .35 .35',diffuse='.55 .55 .55')
 for side,color in [('left','.15 .85 1 1'),('right','1 .4 .1 1')]:
  arm=root.find(f".//body[@name='astribot_arm_{side}_link_7']")
  site=arm.find(f"site[@name='arm_{side}_tool']");assert np.allclose(np.fromstring(site.get('pos'),sep=' '),[0,-.15,0])
  wand=ET.SubElement(arm,'body',name=side+'_wand',pos=site.get('pos'))
  ET.SubElement(wand,'inertial',pos='0 -.04 0',mass='.025',diaginertia='.00002 .000005 .00002')
  ET.SubElement(wand,'geom',name=side+'_wand_geom',type='capsule',fromto='0 0 0 0 -.08 0',size='.007',rgba=color,group='5',contype='0',conaffinity='0')
  ET.SubElement(wand,'site',name=side+'_light',pos='0 -.08 0',size='.008',rgba=color)
 for i,g in enumerate(root.findall('.//worldbody//geom')):
  if not g.get('name'):g.set('name',f't1_geom_{i:03d}')
  g.set('contype','0');g.set('conaffinity','0')
 # Use official dedicated collision meshes where supplied; retain visual geometry separately.
 meshmap={e.get('name'):e for e in root.findall('.//asset/mesh')};asset=root.find('asset');proxy_sources=[]
 for body in root.findall('.//worldbody//body'):
  for g in list(body.findall('geom')):
   if g.get('type')!='mesh':continue
   original=g.get('mesh');me=meshmap[original];visualpath=Path(me.get('file'));candidate=visualpath.with_name(visualpath.stem+'_collision_mesh.obj')
   if visualpath.stem=='astribot_torso_base_link':candidate=visualpath.with_name('astribot_torso_base_collision_mesh.obj')
   if candidate.exists() and candidate.read_bytes().startswith(b'version https://git-lfs'):
    raise RuntimeError(f'Official collision mesh remains an unresolved LFS pointer: {candidate}')
   valid=candidate.exists()
   if valid:
    newname=original+'_collision'
    if newname not in meshmap:
     added=ET.SubElement(asset,'mesh',name=newname,file=str(candidate));meshmap[newname]=added
   else:newname=original
   proxy=copy.deepcopy(g);proxy.set('name','collision_'+g.get('name'));proxy.set('mesh',newname);proxy.set('group','3');proxy.set('rgba','.5 .7 .9 .18');proxy.attrib.pop('material',None);proxy.set('density','0');body.append(proxy)
   g.set('name','visual_'+g.get('name'));g.set('group','1')
   proxy_sources.append({'body':body.get('name'),'visual':str(visualpath),'collision':str(candidate if valid else visualpath),'official_dedicated_proxy':valid})
 # Preserve gripper equality but rebuild exact contact coverage.
 for c in root.findall('contact'):root.remove(c)
 opt=root.find('option');ET.SubElement(opt,'flag',nativeccd='disable')
 xml=ET.tostring(root,encoding='unicode');temp=mujoco.MjModel.from_xml_string(xml)
 pairs=[];excluded=[];counts={}
 if collision:
  ct=ET.SubElement(root,'contact')
  for a in range(temp.ngeom):
   for b in range(a+1,temp.ngeom):
    cat,why=classify(temp,a,b,exclude_compound_elbow);rec={'geom1':temp.geom(a).name,'geom2':temp.geom(b).name,'body1':temp.body(int(temp.geom_bodyid[a])).name,'body2':temp.body(int(temp.geom_bodyid[b])).name}
    if cat:
     ET.SubElement(ct,'pair',name=f't1_pair_{a}_{b}',geom1=rec['geom1'],geom2=rec['geom2'],condim='3',margin='.001',solref='.02 1',solimp='.95 .99 .001',friction='.7 .7 .001 .0001 .0001')
     pairs.append(dict(rec,category=cat));counts[cat]=counts.get(cat,0)+1
    else:excluded.append(dict(rec,reason=why))
 xml=ET.tostring(root,encoding='unicode');(out/'scene.t1.xml').write_text(xml)
 del temp
 m=mujoco.MjModel.from_xml_string(xml);mujoco.mj_saveModel(m,str(out/'scene.t1.mjb'),None)
 rep={'upstream_commit':'cab872dc00081d2c5b096c7a19e563050eb7a2ff','model':'Astribot T1, not S1','fixed_base':'Removed chassis freejoint, all wheel joints and four wheel velocity actuators. Wheels/rollers retained as welded geometry. Chassis yaw -pi/2 for +X image-right,-Y forward frame.','tool':'Official arm_*_tool site [0,-.15,0] in link7; additional 80mm local -Y light stick,7mm capsule radius,25g mass.','compound_wrist_evidence':'Forearm5 and gripperbase dedicated collision mesh overlap near wrist6 pivot (witness radii9-40mm), pads are separated38mm; only base mesh pair excluded, not contact pads/fingers/wand.', 'compound_elbow_evidence':'Original supplied dedicated collision meshes link3/link5 overlap35.50mm atneutral; independent originaltriangle ray audit finds overlapping solids around joint. Explicit adjacent mechanism filter adopted, not a global collision-free claim. Further samepair fold safety not tested.', 'exclude_compound_elbow':exclude_compound_elbow,'proxy_sources':proxy_sources,'nq':m.nq,'nu':m.nu,'pair_count':len(pairs),'counts':counts,'pairs':pairs,'excluded_pairs':excluded,'limits':['MuJoCo convex hulls fill concavities, not calibrated hardware collision models.','Explicit mechanical attachment exclusions are enumerated and cannot certify contact safety in those regions.','No human model; base/wheel movement excluded by fixed-base scope.','T1 joint limits/inertias retained; historical S1 reachability not applicable.']}
 (out/'t1-collision-model.json').write_text(json.dumps(rep,indent=2));return m

if __name__=='__main__':
 out=ROOT/'t1_model_check';m=build_t1_scene(out);d=mujoco.MjData(m);d.qpos[:4]=[.05,-.4,.35,0];mujoco.mj_forward(m,d)
 print('nq',m.nq,'nu',m.nu,'pairs',m.npair,flush=True);print(json.dumps(check_distances(m,d),indent=2),flush=True)
