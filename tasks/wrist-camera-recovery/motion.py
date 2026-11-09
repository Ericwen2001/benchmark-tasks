import numpy as np
from types import SimpleNamespace
from yam_arena import ik,station
from yam_arena.demo import ArmPlan,grip_rotation
from yam_arena.constants import FINGER_OPEN,FINGER_CLOSED,PAD_TIP_PAST_TCP
import scene
class PosePlan(ArmPlan):
 def target(self,t):
  p,R,g=self.start_pos,self.start_rot,self.finger_start
  for w in self.waypoints:
   if t<w.duration:
    s=ik.smoothstep(t/w.duration)
    return p+s*(w.pos-p),ik.slerp_matrix(R,w.rot,s),g+s*(w.finger-g),w.label
   t-=w.duration;p,R,g=w.pos,w.rot,w.finger
  return p,R,g,'done'
def make_plan():
 B=np.array(scene.LAYOUT['present_rotation'])@np.array(scene.LAYOUT['mount_rotation_in_gripper'])
 G=grip_rotation(-np.pi/2);off=np.array([0,0,PAD_TIP_PAST_TCP]);pick=np.array(scene.CAMERA.pos);goal=scene.GOAL
 ws=[]
 def add(s,body,R,grip,label):ws.append(SimpleNamespace(duration=s,pos=np.array(body)+R@off,rot=R@G,finger=grip,label=label))
 I=np.eye(3)
 add(2,pick+[0,0,.09],I,FINGER_OPEN,'approach camera')
 add(2,pick,I,FINGER_OPEN,'grasp camera')
 add(1.3,pick,I,FINGER_CLOSED,'close')
 add(2,pick+[0,0,.09],I,FINGER_CLOSED,'lift')
 add(3,goal+B@np.array([0,0,.035]),B,FINGER_CLOSED,'rotate camera toward original mount')
 add(2,goal+[0,0,0.001],B,FINGER_CLOSED,'seat at original mount')
 add(1.2,goal+[0,0,.001],B,FINGER_OPEN,'release camera')
 add(2,goal+B@np.array([0,0,.07]),B,FINGER_OPEN,'clear mount')
 add(3,[.35,-.20,scene.TOP+.15],I,FINGER_OPEN,'withdraw')
 return PosePlan(ws,station.rig().bases['right'])
if __name__=='__main__':
 p=make_plan()
 for t in np.arange(0,p.duration+.05,.05):
  a,l=p.action7(t)
 print(p.worst_ik_mm,p.worst_ik_deg)
