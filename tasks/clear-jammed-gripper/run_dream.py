"""Headless native-observation probe. No GPU launched unless explicitly run.
Modes: hold, open_only (trivial escape control), demo. No model run.
"""
import argparse,json,time,shutil
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--out',required=True);p.add_argument('--max-steps',type=int,default=1800);a=p.parse_args()
out=Path(a.out);out.mkdir(parents=True,exist_ok=True);(out/'source').mkdir(exist_ok=True)
for f in Path(__file__).parent.iterdir():
 if f.suffix in ['.py','.json']:shutil.copy2(f,out/'source'/f.name)
from isaaclab.app import AppLauncher
app=AppLauncher(headless=True,enable_cameras=True,device='cuda:0').app
try:
 import numpy as np,torch
 from scipy.spatial.transform import Rotation
 from dataclasses import replace
 from PIL import Image
 from yam_arena import ik,station,task_file
 from yam_arena.constants import READY,TCP_OFFSET
 import scene,adapter
 adapter.install()
 from yam_arena.embodiment import BimanualYamEmbodiment
 from yam_arena.scene import make_env,object_states
 from scorer import Scorer
 from geometry import distal_clearance as measure_clearance
 task=task_file.load(str(Path(__file__).with_name('scene.py')))
 emb=BimanualYamEmbodiment(enable_cameras=True,overview_cam=True,contact_sensors=False,rig=station.rig(),cam_width=320,cam_height=176)
 eye=np.array([.60,-.40,1.40])
 forward=np.array([.32,-.04,1.08])-eye;forward/=np.linalg.norm(forward);right=np.cross(forward,[0,0,1]);right/=np.linalg.norm(right);down=np.cross(forward,right)
 emb.camera_config.overview_cam.offset.pos=tuple(map(float,eye))
 emb.camera_config.overview_cam.offset.rot=tuple(Rotation.from_matrix(np.column_stack([right,down,forward])).as_quat())
 emb.camera_config.overview_cam.offset.convention='ros'
 env=make_env(emb,scene='task',task=task,control_hz=20);base=env.unwrapped;env.reset()
 rows=[];scorer=Scorer();start=time.monotonic();held_confirmed=False;hold_evidence=0.;grasp_offset=None
 def body_state():
  d=object_states(env,task.names)['debris'];rb=base.scene.rigid_objects['debris'].data
  def arr(x):
   x=x.torch if hasattr(x,'torch') else x
   return x[0].cpu().numpy()
  return replace(d,pos=arr(rb.root_pos_w),quat_xyzw=arr(rb.root_quat_w))
 # Settle the initial lodged object before opening a paid model session.
 reset_rows=[]
 for k in range(40):
  env.step(torch.tensor([[*scene.Q,0.,*READY,1.]],device=base.device,dtype=torch.float32))
  d=body_state();left=base.scene['left_robot'];ql=left.data.joint_pos.torch[0].cpu().numpy()
  names=list(left.joint_names);gap=-float(ql[names.index('joint7')]+ql[names.index('joint8')])
  tcp,R=ik.tcp_pose(ql[:6],station.rig().bases['left']);root=tcp-R@np.array(TCP_OFFSET)
  clearance=measure_clearance(scene,d.pos,d.quat_xyzw,root,R)
  removed=bool(clearance>.010 or np.linalg.norm(d.pos-tcp)>.15)
  assessment=scorer.update(dt=.05,phase='reset close test',gap=gap,closing_command=True,removed=removed,hand_clear=False,fragment_on_table_or_held=bool(d.pos[2]>scene.TOP-.015))
  reset_rows.append(dict(step=k,left_grip_command=0.,distal_clearance_m=clearance,assessment=assessment))
 (out/'initial_validation.json').write_text(json.dumps(reset_rows,indent=2))
 if not scorer.jam_observed:raise RuntimeError('Initial blocked closure not established; no paid session started.')
 from dream_driver import DreamDriver
 driver=DreamDriver(base,json.loads(Path(__file__).with_name('task.json').read_text())['instruction'],out)
 for k in range(a.max_steps):
  action=driver.action()
  env.step(torch.tensor(action[None],device=base.device,dtype=torch.float32))
  d=body_state();left=base.scene['left_robot'];ql=left.data.joint_pos.torch[0].cpu().numpy()
  names=list(left.joint_names);gap=-float(ql[names.index('joint7')]+ql[names.index('joint8')])
  tcp,R=ik.tcp_pose(ql[:6],station.rig().bases['left']);root=tcp-R@np.array(TCP_OFFSET)
  initial=root+R@scene.ROOT_LOCAL
  # Conservative separation from left jaw; geometry alone is not a success guarantee.
  # Verify the whole fragment has cleared the native fingertip, not an arbitrary
  # displacement of its root. Root pose is the actual free rigid-body frame.
  distal_clearance=measure_clearance(scene,d.pos,d.quat_xyzw,root,R)
  removed=bool(distal_clearance>.010 or np.linalg.norm(d.pos-tcp)>.15)
  rp,rR=ik.tcp_pose(base.scene['right_robot'].data.joint_pos.torch[0,:6].cpu().numpy(),station.rig().bases['right'])
  tab_local=rR.T@(d.pos-rp)
  phase='Dream-1 control'
  if not removed and gap>.010:phase='verify initial obstruction'
  elif removed and gap<.002:phase='verify recovered close'
  elif removed and gap>.085:phase='verify recovered open'
  if removed and np.linalg.norm(tab_local)<.025 and d.pos[2]>scene.TOP+.10 and action[13]<.5:
   if grasp_offset is None:grasp_offset=tab_local.copy()
   hold_evidence=hold_evidence+.05 if np.linalg.norm(tab_local-grasp_offset)<.006 else 0.
   if hold_evidence>=2.:held_confirmed=True
  elif not held_confirmed:hold_evidence=0.;grasp_offset=None
  scoring_phase=('verify initial obstruction' if not removed and gap>.010 else 'verify recovered close' if removed and gap<.002 else 'verify recovered open' if removed and gap>.085 else 'observe')
  assessment=scorer.update(dt=.05,phase=scoring_phase,gap=gap,closing_command=bool(action[6]<=.05),removed=removed,hand_clear=np.linalg.norm(rp-tcp)>.12,fragment_on_table_or_held=bool(d.pos[2]>scene.TOP-.015))
  assessment['grasp_hold_confirmed']=held_confirmed
  assessment['policy_control_only']=True
  with (out/'measurements.jsonl').open('a') as log:
   log.write(json.dumps(dict(step=k, left_grip_command=float(action[6]), gap_m=gap, removed=removed, distal_clearance_m=distal_clearance, hand_clear=bool(np.linalg.norm(rp-tcp)>.12), debris_pos=d.pos.tolist(), debris_quat_xyzw=d.quat_xyzw.tolist(), assessment=assessment))+'\n')
  if k%5==0:
   for name in ['top_cam','left_cam','right_cam','overview_cam']:
    rgb=base.scene[name].data.output['rgb'];rgb=rgb.torch if hasattr(rgb,'torch') else rgb
    Image.fromarray(rgb[0,...,:3].cpu().numpy().astype(np.uint8)).save(out/f'{name}_{k:04d}.png')
   rows.append(dict(step=k,phase=phase,left_joints=ql[:6].tolist(),brick_in_left=(R.T@(d.pos-root)).tolist(),debris_pos=d.pos.tolist(),debris_quat_xyzw=d.quat_xyzw.tolist(),left_grip_command=float(action[6]),right_tcp=rp.tolist(),right_gap_m=-float(base.scene["right_robot"].data.joint_pos.torch[0,6:8].sum()),tab_in_right_tcp=tab_local.tolist(),hold_evidence_s=hold_evidence,distal_clearance_m=distal_clearance,assessment=assessment));(out/'trace.json').write_text(json.dumps(rows,indent=2))
 (out/'result.json').write_text(json.dumps(dict(completed=True,mode='Dream-1',wall_seconds=time.monotonic()-start,score=assessment['score'],final=rows[-1],status='Recorded local trial'),indent=2));driver.close();env.close()
except Exception:
 import traceback
 (out/'error.txt').write_text(traceback.format_exc());raise
finally:app.close()
