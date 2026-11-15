"""Initialize a compound free rigid fragment; never attach it to the robot."""
def install():
 import numpy as np
 import isaaclab.sim as sim
 from isaaclab.sim.utils import clone
 from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg
 from scipy.spatial.transform import Rotation
 import yam_arena.embodiment as emb
 import yam_arena.scene as scenes
 import scene
 original_arm=emb.yam_arm_cfg
 def arm(*args,**kw):
  cfg=original_arm(*args,**kw)
  if 'LeftArm' in cfg.prim_path:
   cfg.init_state.joint_pos.update({f'joint{i+1}':float(v) for i,v in enumerate(scene.Q)})
   cfg.init_state.joint_pos.pop('joint[7-8]',None)
   cfg.init_state.joint_pos.update(joint7=-.0108,joint8=-.0108)
  if 'RightArm' in cfg.prim_path:
   cfg.init_state.joint_pos.update({f'joint{i+1}':float(v) for i,v in enumerate(__import__('yam_arena.constants',fromlist=['READY']).READY)})
  return cfg
 emb.yam_arm_cfg=arm
 original_object=scenes.task_object
 def obj(spec):
  result=original_object(spec)
  if spec.name!='debris':return result
  @clone
  def spawn(path,cfg,*args,**kw):
   from isaaclab.sim.spawners.shapes import spawn_cuboid
   prim=spawn_cuboid.__wrapped__(path,cfg,*args,**kw)
   from pxr import UsdGeom,Gf
   # Rotate root plate geometry only, not the rigid-body frame.
   mesh=prim.GetStage().GetPrimAtPath(path+'/geometry/mesh')
   if not mesh.IsValid():raise RuntimeError('Missing root plate mesh')
   q=Rotation.from_matrix(scene.R).as_quat()
   mesh.GetAttribute("xformOp:orient").Set(Gf.Quatd(float(q[3]),Gf.Vec3d(*map(float,q[:3]))))
   for name,pos,size in scene.PARTS:
    c=sim.CuboidCfg(size=size,collision_props=PhysxCollisionPropertiesCfg(contact_offset=.0001,rest_offset=0),visual_material=sim.PreviewSurfaceCfg(diffuse_color=scene.COLOR,roughness=.25))
    partR=scene.R@Rotation.from_euler('z',int(name.split('_')[1])*2*np.pi/12).as_matrix() if name.startswith('tube_') else scene.R
    c.func(path+'/'+name,c,translation=tuple(scene.R@(np.array(pos)-scene.ROOT_LOCAL)),orientation=tuple(Rotation.from_matrix(partR).as_quat()))
   for j,pos in enumerate(scene.STUDS):
    stud=sim.CylinderCfg(radius=.0045,height=.0035,collision_props=PhysxCollisionPropertiesCfg(contact_offset=.0001,rest_offset=0),visual_material=sim.PreviewSurfaceCfg(diffuse_color=scene.COLOR,roughness=.25))
    stud.func(path+'/stud_'+str(j),stud,translation=tuple(scene.R@(np.array(pos)-scene.ROOT_LOCAL)),orientation=tuple(Rotation.from_matrix(scene.R).as_quat()))
   return prim
  result.spawner_cfg.func=spawn;result.object_cfg.spawn.func=spawn
  result.object_cfg.spawn.collision_props=PhysxCollisionPropertiesCfg(contact_offset=.0001,rest_offset=0)
  return result
 scenes.task_object=obj
