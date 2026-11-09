"""Physical bracket plus keyed snap-latch approximation. Never writes camera body poses.
Latch models a reusable clip, not arbitrary attachment: tight seating and low speed required.
"""
import numpy as np
BRACKET_AUDIT=[]
def install():
 import yam_arena.embodiment as emb
 import isaaclab.sim as sim
 from isaaclab.sim.utils import clone
 from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg
 from yam_arena import station
 from yam_arena.demo import grip_rotation
 from yam_arena.robot import asset_link_paths
 import scene as spec
 original=emb.yam_arm_cfg;R=np.array(spec.LAYOUT['mount_rotation_in_gripper']).T;center=np.array(spec.LAYOUT['mount_position_in_gripper'])
 @clone
 def spawn(path,cfg,*args,**kwargs):
  from yam_arena.robot import spawn_in_dream_look
  prim=spawn_in_dream_look.__wrapped__(path,cfg,*args,**kwargs)
  parent=path+asset_link_paths()['gripper']
  # Shelf is integral with wrist body; no kinematic teleport of the bracket.
  for name,delta,size in [('rail_a',(-.017,0,-.017),(.006,.026,.006)),('rail_b',(.017,0,-.017),(.006,.026,.006))]:
   c=sim.CuboidCfg(size=size,collision_props=PhysxCollisionPropertiesCfg(),visual_material=sim.PreviewSurfaceCfg(diffuse_color=(.1,.35,.75)))
   from yam_arena import ik
   c.func(parent+'/recovery_'+name,c,translation=tuple(center+R.T@np.array(delta)),orientation=tuple(ik.matrix_to_quat_xyzw(R.T)))
   BRACKET_AUDIT.append(parent+'/recovery_'+name)
  # Two solid struts join the seat to the wrist housing. Optical target is unchanged.
  from scipy.spatial.transform import Rotation
  for side in (-1,1):
   start=np.array([side*.017,-.020,.025])
   end=center+R.T@np.array([side*.017,0,-.020])
   delta=end-start; length=np.linalg.norm(delta); z=delta/length
   x=np.cross([0,0,1],z);x/=np.linalg.norm(x);y=np.cross(z,x)
   rotation=Rotation.from_matrix(np.column_stack([x,y,z])).as_quat()
   c=sim.CuboidCfg(size=(.009,.009,float(length+.009)),collision_props=PhysxCollisionPropertiesCfg(),visual_material=sim.PreviewSurfaceCfg(diffuse_color=(.32,.36,.40),metallic=.6,roughness=.35))
   c.func(parent+'/recovery_strut_'+str(side).replace('-','m'),c,translation=tuple((start+end)/2),orientation=tuple(rotation))
   BRACKET_AUDIT.append(dict(part='strut',start=start.tolist(),end=end.tolist()))
  c=sim.CuboidCfg(size=(.045,.030,.006),collision_props=PhysxCollisionPropertiesCfg(),visual_material=sim.PreviewSurfaceCfg(diffuse_color=(.32,.36,.40),metallic=.6,roughness=.35))
  c.func(parent+'/recovery_seat',c,translation=tuple(center+R.T@np.array([0,0,-.023])),orientation=tuple(ik.matrix_to_quat_xyzw(R.T)))
  return prim
 def configured(*args,**kwargs):
  cfg=original(*args,**kwargs)
  if 'LeftArm' in cfg.prim_path:cfg.spawn.func=spawn;cfg.spawn.make_uninstanceable=True
  return cfg
 emb.yam_arm_cfg=configured

def latch(stage,parent_path,camera_path,gripper_pos,gripper_rot,camera_pos,camera_rot):
 from pxr import UsdPhysics,Gf,Sdf
 from scipy.spatial.transform import Rotation
 # Joint frames coincide at current camera pose, so latching cannot snap/teleport it.
 joint=UsdPhysics.FixedJoint.Define(stage,'/World/RecoveryLatch')
 joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_path)])
 joint.CreateBody1Rel().SetTargets([Sdf.Path(camera_path)])
 local=gripper_rot.T@(camera_pos-gripper_pos);q=Rotation.from_matrix(gripper_rot.T@camera_rot).as_quat()
 joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*map(float,local)))
 joint.CreateLocalRot0Attr().Set(Gf.Quatf(float(q[3]),Gf.Vec3f(*map(float,q[:3]))))
 joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0,0,0));joint.CreateLocalRot1Attr().Set(Gf.Quatf(1))
 joint.CreateExcludeFromArticulationAttr().Set(True)
 return joint
