from pathlib import Path
import argparse,json,time,traceback,subprocess
p=argparse.ArgumentParser();p.add_argument('--steps',type=int,default=140);p.add_argument('--out',required=True);a=p.parse_args();OUT=Path(a.out);OUT.mkdir(parents=True,exist_ok=True)
from isaaclab.app import AppLauncher
app=AppLauncher(headless=True,enable_cameras=True,device='cuda:0').app
try:
 import numpy as np,torch
 from PIL import Image
 from scipy.spatial.transform import Rotation
 import hardware
 hardware.install()
 from yam_arena import station,ik
 import yam_arena.scene as sm
 from yam_arena.constants import READY
 from yam_arena.embodiment import BimanualYamEmbodiment
 station.ITEMS={};orig=sm.station_assets
 def assets(cam):
  d=orig(cam);d.pop('bottle',None);return d
 sm.station_assets=assets
 emb=BimanualYamEmbodiment(enable_cameras=True,overview_cam=False,contact_sensors=False,robot_contact_offset=.0005,rig=station.rig(),cam_width=320,cam_height=176)
 env=sm.make_env(emb,scene='station',control_hz=20,device='cuda:0');b=env.unwrapped;env.reset()
 pv=b.sim.physics_manager.get_physics_sim_view();cv=pv.create_rigid_body_view('/World/envs/env_0/service_cable/segment_*');fv=pv.create_rigid_body_view('/World/envs/env_0/service_flap/segment_*')

 def poses(v):
  x=v.get_transforms();return x.numpy() if hasattr(x,'numpy') else x.cpu().numpy()
 # Reset unmanaged task bodies once alongside robot reset, BEFORE any action.
 import warp as wp
 for v,key in [(cv,'cable_poses'),(fv,'flap_poses')]:
  desired=np.asarray(hardware.CREATED[key],dtype=np.float32);indices=wp.array(np.arange(v.count,dtype=np.int32),dtype=wp.int32,device=b.device)
  v.set_transforms(wp.array(desired,dtype=wp.float32,device=b.device),indices)
  v.set_velocities(wp.zeros((v.count,6),dtype=wp.float32,device=b.device),indices)
 (OUT/'initialization_check.json').write_text(json.dumps({'reset_once_before_action':True,'max_cable_position_error_m':float(np.linalg.norm(poses(cv)[:,:3]-np.array(hardware.CREATED['cable_poses'])[:,:3],axis=1).max())}))
 qr=np.array(json.loads(Path(__file__).with_name('initial_robot_state.json').read_text())['right_actual_q'])

 qr=np.array(json.loads(Path(__file__).with_name('initial_robot_state.json').read_text())['right_actual_q']);grip=1.;rows=[];k=0;failure=None;latched=False
 from prepare_geometry import fk
 import importlib.util
 adhesive_spec=importlib.util.spec_from_file_location('cable_adhesive',Path(__file__).with_name('velcro_contact.py'));adhesive_module=importlib.util.module_from_spec(adhesive_spec);adhesive_spec.loader.exec_module(adhesive_module);VelcroContact=adhesive_module.VelcroContact
 adhesive=VelcroContact(pv,hardware);adhesive.bind()
 P=Path(__file__).parent;seg=9
 def jointq(side):
  d=b.scene[side+'_robot'].data.joint_pos;d=d.torch if hasattr(d,'torch') else d;return d[0].cpu().numpy()
 def actual_mount_T():
  robot=b.scene['left_robot'];i=robot.body_names.index('link3')
  pos=robot.data.body_link_pos_w[0,i].cpu().numpy();q=robot.data.body_link_quat_w[0,i].cpu().numpy()
  T=np.eye(4);T[:3,3]=pos;T[:3,:3]=Rotation.from_quat(q).as_matrix();return T
 def move(target,g,n,label):
  global qr,grip,k,latched
  q0=qr.copy();g0=grip
  for j in range(1,n+1):
   t=ik.smoothstep(j/n);qr=q0+(target-q0)*t;grip=g0+(g-g0)*t
   if latched:adhesive.maintain(actual_mount_T())
   env.step(torch.tensor([[*READY,1.,*qr,grip]],device=b.device,dtype=torch.float32));k+=1
   aq=jointq('right');lq=jointq('left');tcp,RR=ik.tcp_pose(aq[:6],station.rig().bases['right']);fp=poses(fv);distance=float(np.linalg.norm(fp[seg,:3]-tcp-RR[:,2]*.006))
   T=actual_mount_T()
   if k==1:pass
   if label in ['lower strap','press strap'] and not latched:
    if adhesive.update(fp,T,.05):latched=True;print('CONTACT BOND',k,flush=True)
   if k%5==0:
    rows.append(dict(step=k,phase=label,right_actual_q=aq[:6].tolist(),right_command_q=qr.tolist(),right_actual_fingers=aq[6:].tolist(),left_actual_q=lq[:6].tolist(),flap=fp.tolist(),cable=poses(cv).tolist(),grasp_distance=distance,latched=latched))
    for name in ['top_cam','left_cam','right_cam']:
     rgb=b.scene[name].data.output['rgb'];rgb=rgb.torch if hasattr(rgb,'torch') else rgb;Image.fromarray(rgb[0,:,:,:3].cpu().numpy().astype('uint8')).save(OUT/f'{name}_{k:04d}.png')
   if k%20==0:pass
   if k%20==0:print(k,label,'distance',round(distance,4),'qerr',round(float(max(abs(aq[:6]-qr))),4),flush=True);(OUT/'trace.json').write_text(json.dumps(rows))
  return distance,float(max(abs(aq[:6]-qr)))
 previous_phase=None
 for row in json.loads((P/'replay_actions.json').read_text()):
  if previous_phase=='lower strap' and row['phase']!='lower strap':
   from scipy.optimize import least_squares
   for attempt in range(3):
    if latched:break
    aq=jointq('right')[:6];pp,RR=ik.tcp_pose(aq,station.rig().bases['right']);T=actual_mount_T();normal=T[:3,:3]@hardware.B[:,0];goal=pp-normal*.001
    def press_loss(v):
     pt,rt=ik.tcp_pose(v,station.rig().bases['right']);return np.r_[pt-goal,.08*Rotation.from_matrix(RR@rt.T).as_rotvec()]
    fit=least_squares(press_loss,aq,bounds=(ik.LOWER+1e-7,ik.UPPER-1e-7),max_nfev=120)
    move(qr+(fit.x-aq),0.,20,'press strap')
  move(np.array(row['q']),row['grip'],5,row['phase']);previous_phase=row['phase']
 move(qr.copy(),1.,60,'final unsupported hold')
 grasp=True
 T=actual_mount_T();remaining=adhesive.matches(poses(fv),T);held=latched and len(remaining)>=1
 failure=None if held else 'adhesive did not remain attached after release'
 (OUT/'trace.json').write_text(json.dumps(rows));(OUT/'result.json').write_text(json.dumps(dict(grasp=bool(grasp),latched=latched,retained=held,remaining_contact_regions=remaining,failure=failure,scope='continuous scripted demonstration'),indent=2));env.close()
except Exception:
 (OUT/'error.txt').write_text(traceback.format_exc());print(traceback.format_exc(),flush=True)
finally:app.close()
