"""Strict, non-rendering SVG centerline input. No remote resources or XML entities.

Only a deliberately bounded SVG subset is a robot program: root svg, two direct
path children with id=left/right (or data-arm), M L C Q Z, no transforms/styles.
Stroke styling is ignored; stroke centreline is executed, not its painted edges.
"""
from pathlib import Path
import hashlib
import math
import re
import xml.etree.ElementTree as ET
import numpy as np

class SVGInputError(ValueError):
    pass

NUMBER = r'[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?'
TOKEN = re.compile(r'[A-Za-z]|' + NUMBER)
WIDTH, HEIGHT = 616, 560


def _tokens(d):
    out=[]; end=0
    for m in TOKEN.finditer(d):
        gap=d[end:m.start()]
        if gap.strip(' \t\r\n,') or (',' in gap and (gap.count(',')!=1 or not out or out[-1].isalpha() or m.group().isalpha())):
            raise SVGInputError('Invalid path separator syntax')
        out.append(m.group());end=m.end()
    if d[end:].strip(' \t\r\n'): raise SVGInputError('Invalid path syntax')
    return out


def _distance(p,a,b):
    v=b-a
    if np.dot(v,v)==0:return float(np.linalg.norm(p-a))
    return float(np.linalg.norm(p-(a+np.clip(np.dot(p-a,v)/np.dot(v,v),0,1)*v)))


def _flatten(ctrl,tol,out,depth=0):
    if len(out)>200000:raise SVGInputError('Curve exceeds point budget')
    # Convex-hull distance to finite chord bounds curve-to-polyline error.
    if max(_distance(p,ctrl[0],ctrl[-1]) for p in ctrl[1:-1])<=tol:
        out.append(ctrl[-1]);return
    if depth>=24:raise SVGInputError('Curve subdivision limit exceeded')
    levels=[ctrl]
    while len(levels[-1])>1:levels.append((levels[-1][:-1]+levels[-1][1:])/2)
    _flatten(np.array([x[0] for x in levels]),tol,out,depth+1)
    _flatten(np.array([x[-1] for x in levels[::-1]]),tol,out,depth+1)


def _path(d,tol,max_step):
    ts=_tokens(d);i=0;cmd=None;current=np.zeros(2);start=None;points=[];curved=False;closed=False
    while i<len(ts):
        if ts[i].isalpha():cmd=ts[i];i+=1
        elif cmd is None:raise SVGInputError('Path must start with M')
        op=cmd.upper();relative=cmd.islower()
        if op not in 'MLCQZ':raise SVGInputError('Unsupported path command: '+cmd)
        if closed:raise SVGInputError('Commands after Z are not supported')
        if op=='Z':
            if start is None:raise SVGInputError('Z before M')
            points.append(start.copy());current=start.copy();closed=True;cmd=None;continue
        count={'M':2,'L':2,'C':6,'Q':4}[op]
        if i+count>len(ts) or any(x.isalpha() for x in ts[i:i+count]):raise SVGInputError('Incomplete '+cmd+' command')
        vals=np.array([float(x) for x in ts[i:i+count]],dtype=float).reshape(-1,2);i+=count
        if not np.isfinite(vals).all() or np.max(np.abs(vals))>1e7:raise SVGInputError('Non-finite or excessive coordinate')
        if relative:vals+=current
        if op=='M':
            if start is not None:raise SVGInputError('Multiple subpaths/M commands are forbidden')
            start=vals[0].copy();points.append(start);cmd='l' if relative else 'L'
        else:
            if start is None:raise SVGInputError('Path must start with M')
            if op=='L':points.append(vals[0])
            else:_flatten(np.vstack([current,vals]),tol,points);curved=True
        current=vals[-1].copy()
        if len(points)>200000:raise SVGInputError('Path exceeds subdivision budget')
    if not points or len(points)<2:raise SVGInputError('Path needs nonzero length')
    dense=[points[0]];duplicates=0
    for a,b in zip(points[:-1],points[1:]):
        length=float(np.linalg.norm(b-a))
        if length==0:duplicates+=1;continue
        n=max(1,int(math.ceil(length/max_step)))
        if len(dense)+n>200000:raise SVGInputError('Path exceeds point budget')
        dense.extend(a+(b-a)*(j/n) for j in range(1,n+1))
    p=np.array(dense)
    if len(p)<2:raise SVGInputError('Path needs nonzero length')
    if np.any(p<0) or np.any(p[:,0]>=WIDTH) or np.any(p[:,1]>=HEIGHT):raise SVGInputError('Path outside canvas (0 <= u < 616, 0 <= v < 560)')
    lengths=np.linalg.norm(np.diff(p,axis=0),axis=1);arc=np.r_[0,np.cumsum(lengths)]
    return {'points_uv':p.tolist(),'progress':(arc/arc[-1]).tolist(),
            'points_world':np.c_[-.77+.0025*p[:,0],np.full(len(p),-.42),1.89-.0025*p[:,1]].tolist(),
            'length_m':float(arc[-1]*.0025),'closed':closed,
            'curve_to_polyline_error_bound_px':tol if curved else 0.,
            'removed_zero_length_segments':duplicates}


def parse_svg(path,tolerance_px=.1,max_step_px=1):
    if not (0<tolerance_px<=.1 and 0<max_step_px<=1):raise SVGInputError('Require 0 < tolerance <= .1 and 0 < step <= 1 pixels')
    raw=Path(path).read_bytes()
    if len(raw)>8_000_000:raise SVGInputError('SVG exceeds 8 MB budget')
    # Only UTF-8; do not let UTF-16/NUL conceal declaration tokens.
    try:source=raw.decode('utf-8-sig')
    except UnicodeDecodeError as e:raise SVGInputError('SVG must be UTF-8') from e
    if '\x00' in source:raise SVGInputError('NUL bytes forbidden')
    # Reject declarations before XML parsing, including entity definitions.
    if '<!' in source or '<?' in source:raise SVGInputError('XML declarations, DTDs, entities and processing instructions forbidden')
    try:r=ET.fromstring(source)
    except ET.ParseError as e:raise SVGInputError('Invalid SVG XML: '+str(e)) from e
    ns='{http://www.w3.org/2000/svg}'
    if r.tag not in ('svg',ns+'svg'):raise SVGInputError('Expected SVG root')
    allowed_root={'width','height','viewBox','version'}
    if set(r.attrib)-allowed_root:raise SVGInputError('Unsupported root attributes: '+str(set(r.attrib)-allowed_root))
    for attr,want in [('width',WIDTH),('height',HEIGHT)]:
        val=r.get(attr,'')
        if not re.fullmatch(NUMBER+r'(?:px)?',val) or float(val.removesuffix('px'))!=want:raise SVGInputError('Require '+attr+'='+str(want)+' in px or unitless')
    try:vb=[float(x) for x in re.split(r'[\s,]+',r.get('viewBox','').strip())]
    except ValueError:vb=[]
    if vb!=[0.,0.,float(WIDTH),float(HEIGHT)]:raise SVGInputError('Require viewBox="0 0 616 560"; no implicit scaling')
    if r.text and r.text.strip():raise SVGInputError('Unexpected root text')
    arms={}
    allowed={'id','data-arm','d','fill','stroke','stroke-width','stroke-linecap','stroke-linejoin'}
    for child in r:
        if child.tag not in ('path',ns+'path') or len(child):raise SVGInputError('Only two direct path elements supported; no embedded content')
        if (child.text and child.text.strip()) or (child.tail and child.tail.strip()):raise SVGInputError('Unexpected text')
        if set(child.attrib)-allowed:raise SVGInputError('Unsupported path attributes: '+str(set(child.attrib)-allowed))
        if any('url(' in v.lower() or 'javascript:' in v.lower() for v in child.attrib.values()):raise SVGInputError('External/reference content forbidden')
        side=child.get('data-arm',child.get('id'))
        if side not in ('left','right') or side in arms:raise SVGInputError('Exactly one explicitly labelled left and right path required')
        if child.get('id') in ('left','right') and child.get('id')!=side:raise SVGInputError('Contradictory arm labels')
        if child.get('fill','none')!='none':raise SVGInputError('Paths must have fill="none"')
        arms[side]=_path(child.get('d',''),tolerance_px,max_step_px)
    if set(arms)!={'left','right'}:raise SVGInputError('Exactly one explicitly labelled left and right path required')
    return {'schema':'astribot-t1-path-v1','width':WIDTH,'height':HEIGHT,
            'coordinate_system':'continuous_pixel_edges','arms':arms,
            'metadata':{'source_sha256':hashlib.sha256(raw).hexdigest(),'svg_subset':'MLCQZ-v1',
                        'max_step_px':max_step_px,'flatten_tolerance_px':tolerance_px,
                        'synchronization':'equal normalized per-arm arc progress',
                        'shape_rescaled':False,'paths_reordered':False,
                        'mask_validated':False,'mechanical_feasibility':'NOT_CHECKED'}}


def _touches_box(a,b,x,y,pad):
    lo=np.array([x-pad,y-pad]);hi=np.array([x+1+pad,y+1+pad]);delta=b-a;t0=0.;t1=1.
    for k in (0,1):
        if delta[k]==0:
            if a[k]<lo[k] or a[k]>hi[k]:return False
        else:
            u=(lo[k]-a[k])/delta[k];v=(hi[k]-a[k])/delta[k]
            t0=max(t0,min(u,v));t1=min(t1,max(u,v))
            if t0>t1:return False
    return True


def validate_masks(data,canvas_dir):
    """Conservative supercover of segments plus flattening error envelope.

    Masks are sampled marginal support only, NOT a joint feasibility certificate.
    """
    from PIL import Image
    data['metadata']['mask_validated']=False
    report={'status':'PASS','scope':'sampled marginal mask only','arms':{}}
    for side,arm in data['arms'].items():
        mask=np.asarray(Image.open(Path(canvas_dir)/('mask-'+side+'.png')).convert('L'))>127
        if mask.shape!=(HEIGHT,WIDTH):raise SVGInputError('Wrong mask dimensions')
        p=np.array(arm['points_uv']);pad=arm.get('curve_to_polyline_error_bound_px',0.);visited=set()
        for idx,(a,b) in enumerate(zip(p[:-1],p[1:])):
            # Inclusive integer borders: all touched cells are checked.
            low=np.ceil(np.minimum(a,b)-pad).astype(int)-1;high=np.floor(np.maximum(a,b)+pad).astype(int)
            for y in range(low[1],high[1]+1):
                for x in range(low[0],high[0]+1):
                    if _touches_box(a,b,x,y,pad):
                        if x<0 or y<0 or x>=WIDTH or y>=HEIGHT or not mask[y,x]:raise SVGInputError(f'{side}: path segment {idx} outside sampled mask at cell ({x},{y})')
                        visited.add((x,y))
        report['arms'][side]={'checked_cells':len(visited),'envelope_px':pad}
    data['metadata']['mask_validated']=True
    return report
