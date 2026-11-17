"""Observable routing, strap placement and unsupported-release rubric."""
import numpy as np
from scipy.spatial.transform import Rotation
MOUNT=np.array([-.05,.007,-.16]);B=np.array([[0,0,-1],[1,0,0],[0,-1,0.]])
class Scorer:
 def __init__(self):self.route_time=0.;self.strap_time=0.;self.release_time=0.;self.best=0.
 def update(self,cable,strap,T,right_tcp,gap,dt):
  c=(T@np.r_[MOUNT,1])[:3];basis=T[:3,:3]@B
  wire=(np.asarray(cable)[:,:3]-c)@basis
  cable=np.asarray(cable);axes=Rotation.from_quat(cable[:,3:]).as_matrix()[:,:,0];half=.7217926530318888/40/2
  gap_error=float(np.linalg.norm((cable[:-1,:3]+axes[:-1]*half)-(cable[1:,:3]-axes[1:]*half),axis=1).max())
  intact=gap_error<.020
  # Cable must actually rest on the upper shaft, behind the cap.
  sampled=[]
  for pos,axis in zip(cable[:,:3],axes):
   sampled.extend((np.array([pos+axis*t for t in np.linspace(-half,half,5)])-c)@basis)
  sampled=np.array(sampled);radial=np.linalg.norm(sampled[:,1:],axis=1)
  contact=(sampled[:,0]>-.006)&(sampled[:,0]<.028)&(sampled[:,2]>.005)&(abs(radial-.013)<.002)
  crossing=bool(np.any(sampled[:,1]<-.008) and np.any(sampled[:,1]>.008))
  routed=bool(intact and crossing and contact.any())
  overlap=False
  for row in strap:
   R=Rotation.from_quat(row[3:]).as_matrix();p=(np.array(row[:3])+R[:,2]*.00115-c)@basis
   if abs(p[0]-.0355)<=.0015 and np.linalg.norm(p[1:])<.017 and R[:,2]@basis[:,0]<-.866:overlap=True
  distance=float(np.linalg.norm(np.asarray(right_tcp)-c));clear=bool(distance>.12 and gap>.080)
  self.route_time=self.route_time+dt if routed else 0.
  self.strap_time=self.strap_time+dt if routed and overlap else 0.
  self.release_time=self.release_time+dt if routed and overlap and clear else 0.
  score=1. if self.release_time>=2. else .75 if self.strap_time>=1. else .5 if self.route_time>=1. else 0.
  self.best=max(self.best,score)
  return dict(score=score,best_score=self.best,routed=routed,strap_overlap=overlap,right_hand_clear=clear,right_tcp_distance_m=distance,release_hold_s=self.release_time)
