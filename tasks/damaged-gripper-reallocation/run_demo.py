"""Physical feasibility demo. No Dream-1 run is claimed."""
import argparse,json,time
from pathlib import Path
import shutil
p=argparse.ArgumentParser();p.add_argument('--out',required=True);a=p.parse_args();out=Path(a.out);out.mkdir(parents=True,exist_ok=True)
snapshot=out/'source';snapshot.mkdir(exist_ok=True)
for source in Path(__file__).parent.iterdir():
 if source.suffix in ('.py','.json'):
  shutil.copy2(source,snapshot/source.name)
from isaaclab.app import AppLauncher
app=AppLauncher(headless=True,enable_cameras=True,device='cuda:0').app
try:
 import numpy as np, torch
 from PIL import Image
 from scipy.spatial.transform import Rotation
 import damage_adapter
 damage_adapter.install()
 import asset_mirror
 (out/"asset_mirror.json").write_text(json.dumps(asset_mirror.install(),indent=2))
 import scene as spec
 from yam_arena import station,task_file,ik
 from scorer import FruitScorer
 from yam_arena.embodiment import BimanualYamEmbodiment
 from yam_arena.scene import make_env,object_states
 from yam_arena.demo import ArmPlan
 from yam_arena.constants import READY
 task=task_file.load(str(Path(__file__).with_name('scene.py')))
 emb=BimanualYamEmbodiment(enable_cameras=True,overview_cam=False,contact_sensors=False,rig=station.rig(),cam_width=640,cam_height=360)
 # Present the exposed flange to the native top camera from the first observation.
 # Fixed initial posture, not a scripted damage signal or policy override.
 damaged_ready=np.array(json.loads(Path(__file__).with_name("initial_pose.json").read_text())["left_joints"])
 env=make_env(emb,scene='task',task=task,control_hz=20);base=env.unwrapped;env.reset()
 (out/'damage_audit.json').write_text(json.dumps(damage_adapter.AUDIT,indent=2))
 if len(damage_adapter.AUDIT)!=2:raise RuntimeError('Damage was not applied to both left tips')
 plan=ArmPlan(spec.demo()['right'],station.rig().bases['right'])
 rows=[];started=time.monotonic();scorers={c:FruitScorer() for c in spec.GOALS}
 for k in range(20+int((plan.duration+3)*20)):
  right,label=([*READY,1.],'settle') if k<20 else plan.action7((k-19)/20)
  action=torch.tensor([[*damaged_ready,1.,*right]],device=base.device,dtype=torch.float32)
  env.step(action);states=object_states(env,task.names)
  tcps=[]
  for side in ('left','right'):
   robot=base.scene[side+'_robot'];q=robot.data.joint_pos.torch[0,:6].cpu().numpy()
   tcps.append(ik.tcp_pose(q,station.rig().bases[side])[0])
  components={}
  for color,goal in spec.GOALS.items():
   block=states[color]
   components[color]=scorers[color].update(fruit=block,bowl=states[spec.BOWLS[color]],tcp_positions=tcps,dt=.05,table_z=spec.TOP)
  assessment={'score':sum(x['score'] for x in components.values())/2,'components':components}
  if k%5==0:
   for name in ('top_cam','left_cam','right_cam'):
    rgb=base.scene[name].data.output['rgb'];rgb=rgb.torch if hasattr(rgb,'torch') else rgb
    Image.fromarray(rgb[0,...,:3].cpu().numpy().astype(np.uint8)).save(out/f'{name}_{k:04d}.png')
   rows.append(dict(step=k,phase=label,block_positions={c:states[c].pos.tolist() for c in spec.GOALS},geometric_success=spec.success(states),assessment=assessment))
   (out/'trace.json').write_text(json.dumps(rows,indent=2));print(k,label,rows[-1],flush=True)
 (out/'result.json').write_text(json.dumps(dict(completed=True,wall_seconds=time.monotonic()-started,final=rows[-1],worst_ik_mm=plan.worst_ik_mm,score=assessment['score'],scope='Two color-matched transfers, each lifted/released/stable. Scripted intact-hand feasibility, not policy reallocation evidence. Dream-1 deferred.'),indent=2))
 env.close()
except Exception:
 import traceback
 (out/'error.txt').write_text(traceback.format_exc());raise
finally:app.close()
