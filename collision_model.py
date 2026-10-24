"""Explicit conservative convex-mesh self-collision model; no human geometry."""
from pathlib import Path
import os,sys,json,re,xml.etree.ElementTree as ET
import numpy as np
import mujoco
import light_painting as lp

def label(body):
    s='left' if ('left' in body) else ('right' if 'right' in body else None)
    if s:
        m=re.search(r'arm_'+s+r'_link_(\d+)$',body)
        return s,int(m.group(1)) if m else (9 if 'wand' in body else 8)
    return None,None

def pair_category(m,a,b):
    ba=int(m.geom_bodyid[a]);bb=int(m.geom_bodyid[b]);na=m.body(ba).name;nb=m.body(bb).name
    sa,ia=label(na);sb,ib=label(nb)
    if ba==bb:return None
    if sa and sb:
        if sa!=sb:return 'inter_arm'
        if abs(ia-ib)<=1 or (ia>=7 and ib>=7) or ({ia,ib}=={5,7}):return None
        return 'same_arm_nonadjacent'
    if not(sa or sb):
        if ba==0 or bb==0:return None # fixed chassis-floor support, arms checked separately
        if m.body_parentid[ba]==bb or m.body_parentid[bb]==ba:return None
        if m.body_weldid[ba]==m.body_weldid[bb]:return None
        if {na,nb}=={'astribot_torso_link_2','astribot_torso_link_4'}:return None
        if {na,nb}=={'astribot_torso_link_4','astribot_head_link_2'}:return None
        # Consecutive torso hinges and head mounting assembly are mechanically adjacent.
        return 'torso_head_base'
    arm,ai,ob=(ba,ia,bb) if sa else (bb,ib,ba)
    # A shoulder link is intentionally attached to the upper torso/mount assembly.
    if ai in (1,2) and m.body(ob).name=='astribot_torso_link_4':
        og=b if sa else a
        mesh=m.mesh(int(m.geom_dataid[og])).name if m.geom_type[og]==mujoco.mjtGeom.mjGEOM_MESH else ''
        side=sa or sb
        if mesh in ('astribot_torso_link_4',f'astribot_arm_{side}_base_link'):return None
    return 'arm_environment' if ob==0 else 'arm_body'

def collision_pairs(m):
    # Explicit MJCF pairs are authoritative; no dynamic contype broad phase.
    out=[]
    for k in range(m.npair):
        a,b=int(m.pair_geom1[k]),int(m.pair_geom2[k]);out.append((a,b,pair_category(m,a,b) or 'attachment_excluded_error'))
    return out

def check_distances(m,d,pairs=None,max_distance=.05):
    if pairs is None:pairs=collision_pairs(m)
    minima={};counts={};best=(float(max_distance),None,None,None)
    for a,b,cat in pairs:
        dist=float(mujoco.mj_geomDistance(m,d,a,b,max_distance,None))
        counts[cat]=counts.get(cat,0)+1
        if cat not in minima or dist<minima[cat]['distance_m']:minima[cat]={'distance_m':dist,'geom1':m.geom(a).name,'geom2':m.geom(b).name}
        if dist<best[0]:best=(dist,m.geom(a).name,m.geom(b).name,cat)
    return {'minimum_distance_m':best[0],'geom1':best[1],'geom2':best[2],'category':best[3],'category_minima':minima,'pair_counts':counts}

def build_collision_scene(assets,out):
    assets=Path(assets);out=Path(out);out.mkdir(parents=True,exist_ok=True)
    lp.build_scene(assets,out)
    root=ET.parse(out/'scene.generated.xml').getroot();world=root.find('worldbody')
    human=world.find("body[@name='mannequin']")
    if human is not None:world.remove(human)
    for node in root.findall('contact'):root.remove(node)
    # Preserve original meshes and inertias; explicit pairs activate response despite masks zero.
    for i,g in enumerate(root.findall('.//worldbody//geom')):
        if not g.get('name'):g.set('name',f'collision_mesh_{i:03d}')
        g.set('contype','0');g.set('conaffinity','0')
    opt=root.find('option');ET.SubElement(opt,'flag',nativeccd='disable')
    temp=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
    ct=ET.SubElement(root,'contact');pairs=[];excluded=[]
    for a in range(temp.ngeom):
        for b in range(a+1,temp.ngeom):
            cat=pair_category(temp,a,b)
            if cat:
                ET.SubElement(ct,'pair',name=f'check_{a}_{b}',geom1=temp.geom(a).name,geom2=temp.geom(b).name,condim='3',margin='0.001',gap='0',solref='.02 1',solimp='.95 .99 .001',friction='.7 .7 .001 .0001 .0001')
                pairs.append({'geom1':temp.geom(a).name,'geom2':temp.geom(b).name,'body1':temp.body(int(temp.geom_bodyid[a])).name,'body2':temp.body(int(temp.geom_bodyid[b])).name,'category':cat})
            else:
                excluded.append({'geom1':temp.geom(a).name,'geom2':temp.geom(b).name,'body1':temp.body(int(temp.geom_bodyid[a])).name,'body2':temp.body(int(temp.geom_bodyid[b])).name,'reason':'Mechanical adjacent/welded/same assembly or non-arm floor support; exact filtering in pair_category'})
    xml=ET.tostring(root,encoding='unicode');(out/'scene.collision.xml').write_text(xml)
    m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    report={'distance_engine':'MuJoCo 3.4 libccd via nativeccd=disable; native convex distance produced false zero for a distant pair and is not used.','representation':'Supplied full per-link triangle meshes used as MuJoCo convex collision hulls; floor primitive, original gripper pads and physical 7 mm radius wand capsules retained. No manual mesh shrinkage. Concave cavities are conservatively filled by hulls.','pair_count':len(pairs),'pairs':pairs,'excluded_pairs':excluded,'response':'Explicit MJCF contact pairs with 1 mm contact margin; all other dynamic collision masks disabled to avoid unlisted internal assembly contacts. These explicit pairs are active in mj_step.','exclusions':['Same rigid body','Same-arm adjacent numbered links; link5-link7 compound wrist joint; wrist link7/gripper/wand internal attachment assembly','Arm link1/link2 versus main torso convex shell and ipsilateral shoulder mount only; neck and contralateral shoulder are checked','Rigid/welded and parent-child non-arm mechanical assembly pairs excluded; nonadjacent torso/head/base checked','Gripper internal linkage geometry overlaps excluded as same assembly','Torso2-torso4 and torso4-head2 compound joints have zero-length intervening links; excluded as adjacent assembly'], 'limits':['Convex hull geometry is conservative but not independently metrologically calibrated to hardware.','No human geometry.','Moving torso nonadjacent hulls checked; adjacent mechanical interfaces excluded.','Excluded mechanical attachments cannot certify arbitrary articulation or deformation.'], 'neutral':check_distances(m,d)}
    (out/'collision_model_report.json').write_text(json.dumps(report,indent=2))
    return m

if __name__=='__main__':
    m=build_collision_scene(Path(os.environ.get('ASTRIBOT_ASSETS', str(Path(__file__).parent/'descriptions'))),Path(__file__).parent)
    d=mujoco.MjData(m);mujoco.mj_forward(m,d);print(json.dumps(check_distances(m,d),indent=2));print('ncon',d.ncon)
    vals=[]
    for a,b,cat in collision_pairs(m):
        dis=mujoco.mj_geomDistance(m,d,a,b,.05,None)
        if dis<.01:vals.append((dis,m.geom(a).name,m.geom(b).name,m.body(int(m.geom_bodyid[a])).name,m.body(int(m.geom_bodyid[b])).name,cat))
    print(sorted(vals))

class CollisionChecker:
    def __init__(self,m):self.m=m;self.pairs=collision_pairs(m)
    def check(self,d):
        r=check_distances(self.m,d,self.pairs);r['distance_m']=r['minimum_distance_m'];return r

def render_collision_view(m,d,path):
    from PIL import Image
    old=m.geom_rgba.copy()
    for g in range(m.ngeom):
        side,_=label(m.body(int(m.geom_bodyid[g])).name)
        m.geom_rgba[g]=([.1,.7,1,.55] if side=='left' else [1,.45,.1,.55] if side=='right' else [.6,.6,.6,.35])
    cam=mujoco.MjvCamera();cam.lookat[:]=[0,0,1.1];cam.distance=2.7;cam.azimuth=135;cam.elevation=-12
    opt=mujoco.MjvOption();opt.geomgroup[:]=1;opt.flags[mujoco.mjtVisFlag.mjVIS_CONVEXHULL]=1
    with mujoco.Renderer(m,720,960) as r:
        r.update_scene(d,camera=cam,scene_option=opt);Image.fromarray(r.render()).save(path)
    m.geom_rgba[:]=old
