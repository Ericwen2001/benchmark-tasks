"""Saved-sample diagnostic; never a substitute for a full-rate simulator replay."""
import argparse,json
import numpy as np
from yam_arena import ik,station
from scorer import Scorer
p=argparse.ArgumentParser();p.add_argument('trace');p.add_argument('--historical-demo',action='store_true');a=p.parse_args()
rows=json.load(open(a.trace));s=Scorer();previous=None
for row in rows:
 tcp,_=ik.tcp_pose(np.array(row['left_joints']),station.rig().bases['left'])
 body=np.array(row['debris_pos']);other=np.array(row['right_tcp'])
 dt=.05 if previous is None else (row['step']-previous)/20;previous=row['step']
 # Legacy traces omit body orientation. A 40 mm radius encloses the entire brick;
 # this conservative clearance test does not depend on missing orientation.
 conservative_clearance=float(row['brick_in_left'][2])-.040-.146843
 command=row.get('left_grip_command')
 # Retained demo controller explicitly commanded close for its first four seconds.
 # This is reconstructed provenance, not a contemporaneously logged command.
 closing=(command<=.05) if command is not None else (a.historical_demo and row['step']<80)
 result=s.update(dt=dt,phase='measured',gap=row['assessment']['gap_m'],closing_command=closing,removed=bool(conservative_clearance>.010 or np.linalg.norm(body-tcp)>.15),hand_clear=bool(np.linalg.norm(other-tcp)>.12),fragment_on_table_or_held=bool(body[2]>.7475-.015))
print(json.dumps({'score':result['score'],'samples':len(rows),'sample_period_seconds':.25,'clearance':'Conservative40 mm bounding sphere; no legacy distal-clearance values used.','command_provenance':'Retained demo controller, first4 seconds only' if a.historical_demo else 'Recorded command if available; otherwise no initial-fault credit assumed','note':'4 Hz saved-sample diagnostic, not full20 Hz replay or a new simulator run.'},indent=2))
