from pathlib import Path
import json,numpy as np
from scipy.spatial.transform import Rotation
from yam_arena import station
r=Path(__file__).parent;c=json.loads((r.parent/'native_mount_and_reallocation/analysis/presentation_candidates.json').read_text())[3]
N=Rotation.from_quat(station.wrist_camera('left').rot_xyzw).as_matrix();O=Rotation.from_euler('x',90,degrees=True).as_matrix();M=N@O.T;L=np.array([0,.019,0]);R=np.array(c['gripper_rotation']);B=R@M
j=json.loads((r/'layout_prototype.json').read_text());j.update(source='Native optical target; camera front normal aligned with optical axis',mount_rotation_in_gripper=M.tolist(),mount_position_in_gripper=(np.array(station.wrist_camera('left').pos)-M@L).tolist(),optical_rotation_body=O.tolist(),lens_offset_body=L.tolist(),camera_center_world=(np.array(c['camera'])-B@L).tolist(),note='Box front +Y is lens face; front optical axis +Y. No diagonal lens. Native target unchanged.')
(r/'layout_prototype.json').write_text(json.dumps(j,indent=2));print('body rotation',B)
