"""Contact-and-compression-triggered local ideal adhesion.
Only engaged rigid strip patches are constrained; adjoining elastic joints
retain dynamics. Constraint projection is applied at physics frequency.
"""
import numpy as np
from scipy.spatial.transform import Rotation
class VelcroContact:
 def __init__(self,physics,hardware):
  self.h=hardware;self.bond_index=None;self.bonded_indices=[];self.local={};self.hold={};self.T=None;self.subscription=None
  self.view=physics.create_rigid_body_view('/World/envs/env_0/service_flap/segment_*')
  self.contacts=physics.create_rigid_contact_view('/World/envs/env_0/service_flap/segment_*',filter_patterns=[hardware.CREATED['links']['link3']])
 def matches(self,poses,T):
  B=T[:3,:3]@self.h.B;c=(T@np.r_[self.h.MOUNT,1])[:3];out=[]
  for i,row in enumerate(poses):
   R=Rotation.from_quat(row[3:]).as_matrix();p=(row[:3]+R[:,2]*.00115-c)@B
   # <=1.5mm numerical contact envelope, true pad radius17mm.
   if abs(p[0]-.0355)<=.0015 and np.linalg.norm(p[1:])<.017 and R[:,2]@B[:,0]<-.866:out.append(i)
  return out
 def bind(self):
  import omni.physx
  self.subscription=omni.physx.get_physx_interface().subscribe_physics_step_events(lambda *args:self._project())
 def update(self,poses,T,dt):
  self.T=T.copy()
  if self.bond_index is not None:return False
  f=self.contacts.get_contact_force_matrix(dt);f=f.numpy() if hasattr(f,'numpy') else f.cpu().numpy()
  forces=np.linalg.norm(f.reshape(len(poses),-1,3),axis=2).sum(axis=1);self.last_forces=forces.tolist()
  ids=self.matches(poses,T)
  for i in range(len(poses)):self.hold[i]=self.hold.get(i,0)+dt if i in ids and forces[i]>.05 else 0.
  active=[i for i in ids if self.hold[i]>=.2-1e-8]
  if not active:return False
  # Pick the strongest actually pressed patch, not the entire strip.
  i=max(active,key=lambda i:forces[i]);row=poses[i]
  self.local[i]=((row[:3]-T[:3,3])@T[:3,:3],T[:3,:3].T@Rotation.from_quat(row[3:]).as_matrix())
  self.bonded_indices=[i];self.bond_index=i;return True
 def maintain(self,T):self.T=T.copy()
 def _project(self):
  if not self.local or self.T is None:return
  import warp as wp
  raw=self.view.get_transforms();rows=(raw.numpy() if hasattr(raw,'numpy') else raw.cpu().numpy()).copy()
  raw=self.view.get_velocities();vel=(raw.numpy() if hasattr(raw,'numpy') else raw.cpu().numpy()).copy()
  for i,(p,R) in self.local.items():
   rows[i]=np.r_[self.T[:3,3]+self.T[:3,:3]@p,Rotation.from_matrix(self.T[:3,:3]@R).as_quat()];vel[i]=0.
  ids=wp.array(np.array(self.bonded_indices,dtype=np.int32),dtype=wp.int32,device='cuda:0')
  self.view.set_transforms(wp.array(rows.astype(np.float32),dtype=wp.float32,device='cuda:0'),ids)
  self.view.set_velocities(wp.array(vel.astype(np.float32),dtype=wp.float32,device='cuda:0'),ids)
