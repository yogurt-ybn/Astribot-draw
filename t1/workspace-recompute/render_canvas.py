from pathlib import Path
import json,numpy as np
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'results-final';SCALE=.0025
FONT='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
def font(n):return ImageFont.truetype(FONT,n)
g=np.load(OUT/'workspace-witnesses.npz');meta=json.loads((OUT/'workspace-summary.json').read_text());xs=g['xs'];zs=g['zs'];ti=g['torso_index'];grid=ti.reshape(2,len(zs),len(xs))>=0;union=grid.any(0);vv,uu=np.where(union);u0,u1=uu.min(),uu.max()+1;v0,v1=vv.min(),vv.max()+1
step=meta['grid_spacing_m'];n=int(round(step/SCALE));xlo=xs[u0]-step/2;xhi=xs[u1-1]+step/2;zlo=zs[v0]-step/2;zhi=zs[v1-1]+step/2
w=int((u1-u0)*n);h=int((v1-v0)*n);small=grid[:,v0:v1,u0:u1][:,::-1,:];masks=np.repeat(np.repeat(small,n,axis=1),n,axis=2)
for side,mask in zip(['left','right'],masks):Image.fromarray(mask.astype('uint8')*255).save(OUT/f'marginal-mask-{side}.png')
# Conventional ownership split only reduces marginal regions, does not certify synchronized combinations.
xcent=xlo+(np.arange(w)+.5)*SCALE
assigned=np.stack([masks[0]&(xcent[None,:]>=0),masks[1]&(xcent[None,:]<0)])
for side,mask in zip(['left','right'],assigned):Image.fromarray(mask.astype('uint8')*255).save(OUT/f'mask-{side}.png')
Image.new('RGBA',(w,h),(255,255,255,0)).save(OUT/'blank-transparent.png');Image.new('RGB',(w,h),'white').save(OUT/'blank-white.png')
colors=np.full((h,w,3),235,dtype=np.uint8);colors[masks[0]]=[30,174,220];colors[masks[1]]=[245,143,62];colors[masks.all(0)]=[151,95,191]
Image.fromarray(colors).save(OUT/'marginal-regions.png')
a=np.full((h,w,3),235,dtype=np.uint8);a[assigned[0]]=[30,174,220];a[assigned[1]]=[245,143,62];Image.fromarray(a).save(OUT/'drawing-regions.png')
cal={'schema':'astribot-t1-canvas-v1','robot':'Astribot T1','width':w,'height':h,'meters_per_pixel':SCALE,'coordinate_system':'continuous_pixel_edges; +u right, +v down, pixel centers at integer+0.5','drawing_plane_world_y_m':meta['plane_y_m'],'world_bounds_m':{'x':[float(xlo),float(xhi)],'z':[float(zlo),float(zhi)]},'physical_size_m':[w*SCALE,h*SCALE],'pixel_to_world':{'x':f'{xlo:.10f} + u * {SCALE}', 'y':str(meta['plane_y_m']),'z':f'{zhi:.10f} - v * {SCALE}'},'sampling_grid_m':step,'pixels_per_sampling_cell':n,'masks':'marginal-mask-* contains all found arm witnesses; mask-* further assigns x>=0 to left arm and x<0 to right for design. Filled cells visualize sampling support only, not verified every pixel.','torso_samples':meta['torso_samples'],'joint_margin_rad':meta['joint_margin_rad'],'model_clearance_m':meta['collision_clearance_threshold_m'],'tool':'80mm local -Y beyond official T1 arm tool site; tip at link7 [0,-.23,0]m','source_commit':meta['upstream_commit'],'model_pair_count':meta['full_model_pair_count'],'validity':'Sampled marginal union over torso; arbitrary two-arm target combinations and continuous paths NOT guaranteed. No S1 camera/crop mapping retained.'}
(OUT/'calibration.json').write_text(json.dumps(cal,indent=2));meta['canvas']=cal;(OUT/'workspace-summary.json').write_text(json.dumps(meta,indent=2))
# Main explanatory figure: real masks at exact scale plus comparison of torso0 versus union.
pad=32;maxw=max(1000,w+2*pad);pw=maxw;ph=h+360;im=Image.new('RGB',(pw,ph),'#f7f9fc');dr=ImageDraw.Draw(im);dr.text((pad,15),'Astribot T1：重新计算的绘制画布',font=font(29),fill='#132c43');dr.text((pad,60),f'{w} × {h} 像素  |  {w*SCALE:.3f} × {h*SCALE:.3f} 米  |  绘制平面 Y = {meta["plane_y_m"]:.2f} m',font=font(22),fill='#354a61')
im.paste(Image.fromarray(colors),((pw-w)//2,110));y=h+125
dr.text((pad,y),'蓝：机器人左臂   橙：机器人右臂   紫：两臂分别可达（不代表同时可达）',font=font(21),fill='#263c54');y+=38
baseline=[int((row==0).sum()) for row in ti];total=[int((row>=0).sum()) for row in ti]
dr.text((pad,y),f'姿态 0 采样点：左 {baseline[0]} / 右 {baseline[1]}；11 个躯干姿态并集：左 {total[0]} / 右 {total[1]}',font=font(20),fill='#263c54');y+=35
dr.text((pad,y),'每像素 2.5 mm；可达验证网格 20 mm。彩色方格不是逐像素安全保证。',font=font(20),fill='#263c54');y+=34
dr.text((pad,y),'每点有完整关节姿态见证；另一臂停放。轨迹仍需共享躯干、同步及碰撞验证。',font=font(20),fill='#263c54');y+=34
dr.text((pad,y),'固定底盘、工具朝向自由；模型机械连接排除项仍适用。灰色：未找到合格逆解。',font=font(20),fill='#263c54');im.save(OUT/'t1-canvas-overview-zh.png')
# Optional torso scope comparison on same new coordinate canvas.
bas=(ti==0).reshape(2,len(zs),len(xs))[:,v0:v1,u0:u1][:,::-1,:].any(0);base=np.repeat(np.repeat(bas,n,0),n,1);new=masks.any(0)&~base
c=np.full((h,w,3),235,dtype=np.uint8);c[base]=[75,133,175];c[new]=[72,180,133];Image.fromarray(c).save(OUT/'torso-sampling-extension.png')
print(json.dumps(cal,indent=2))
