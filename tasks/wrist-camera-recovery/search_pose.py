import json,numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation
from yam_arena import ik,station
from yam_arena.constants import READY,TCP_OFFSET
from yam_arena.demo import grip_rotation
native=station.wrist_camera('left');N=Rotation.from_quat(native.rot_xyzw).as_matrix()
O=Rotation.from_euler('x',90,degrees=True).as_matrix();M=N@O.T;lo=np.array([0,.019,0]);mount=np.array(native.pos)-M@lo
cs=json.loads((Path(__file__).parent.parent/'native_mount_and_reallocation/analysis/presentation_candidates.json').read_text())
for i,c in enumerate(cs):
 R=np.array(c['gripper_rotation']);br=R@M;goal=np.array(c['camera'])-br@lo
 for yaw in [0,np.pi/2,-np.pi/2,np.pi]:
  G=grip_rotation(yaw);tcp=goal+br@np.array([0,0,.013]);gr=br@G
  q,pe,re=ik.solve(tcp,ik.matrix_to_quat_xyzw(gr),np.array(READY),base_pos=station.rig().bases['right'],iters=250)
  print(i,round(yaw,2),'pos',goal.round(3).tolist(),'err',round(pe*1000,2),round(np.degrees(re),2))
