"""Extract a hollow toy building brick lodged over the left gripper finger.
Retention and removal evidence is described in README.md.
"""
import json
from pathlib import Path
import numpy as np
from yam_arena.task_file import Box,TABLE_TOP
from yam_arena import ik,station
from yam_arena.constants import TCP_OFFSET
SCENE='station'; TOP=TABLE_TOP[SCENE]
LAYOUT=json.loads(Path(__file__).with_name('layout.json').read_text())
Q=np.array(LAYOUT['left_joints']); TCP,R=ik.tcp_pose(Q,station.rig().bases['left'])
ORIGIN=TCP-R@np.array(TCP_OFFSET)
# Dimensions derived from native left pad bounds at q7=q8=-25 mm:
# x [-31.431,-25.031], y [-13.695,13.695], z [129.192,146.843] mm.
# Hollow toy brick: collision geometry, no adhesive or fixed joint.
ROOT_LOCAL=np.array([-.019,0,.1484])
ROOT_WORLD=ORIGIN+R@ROOT_LOCAL
COLOR=(.65,.025,.018)
# Generic 2x2 hollow toy brick, 32x32 mm footprint. No brand or logo.
# Root collider is the top plate; walls and studs are explicit contact geometry.
PARTS=[
 ('outer_wall',(-.034,0,.1384),(.002,.032,.018)),
 ('inner_wall',(-.004,0,.1384),(.002,.032,.018)),
 ('side_a',(-.019,-.0144,.1384),(.028,.002,.018)),
 ('side_b',(-.019,.0144,.1384),(.028,.002,.018)),
]
# Hollow underside clutch tube, standard toy-brick structural feature.
for j in range(12):
 angle=2*np.pi*j/12
 PARTS.append((f'tube_{j}',(-.019+.005*np.cos(angle),.005*np.sin(angle),.1394),(.002,.0027,.016)))
STUDS=[(-.019+x,y,.15115) for x in (-.008,.008) for y in (-.008,.008)]
# Start at the seated obstructed gap, rather than dropping onto an open finger.
SHIFT=np.array([.015,0,0]);ROOT_LOCAL=ROOT_LOCAL+SHIFT;ROOT_WORLD=ORIGIN+R@ROOT_LOCAL
PARTS=[(n,tuple(np.array(p)+SHIFT),sz) for n,p,sz in PARTS]
STUDS=[tuple(np.array(p)+SHIFT) for p in STUDS]
OBJECTS=[Box('debris',pos=tuple(map(float,ROOT_WORLD)),size=(.032,.032,.002),mass=.012,color=COLOR)]
# Runtime scorer needs robot state; no object-only success shortcut here.
def success(objects):return False
