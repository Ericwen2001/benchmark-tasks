"""Remove left fingertip visual and collision contributions at spawn time.
Preserves 14-D action API and hidden carrier joint dynamics. This simplification must be disclosed.
No runtime teleporting or selective policy-action overriding.
"""
AUDIT=[]
def install():
 import yam_arena.embodiment as emb
 from isaaclab.sim.utils import clone
 from pxr import Usd, UsdGeom, UsdPhysics
 original_cfg=emb.yam_arm_cfg
 @clone
 def spawn_damaged(path,cfg,*args,**kwargs):
  from yam_arena.robot import spawn_in_dream_look, asset_link_paths
  prim=spawn_in_dream_look.__wrapped__(path,cfg,*args,**kwargs)
  stage=prim.GetStage();links=asset_link_paths()
  for name in ('tip_left','tip_right'):
   link=stage.GetPrimAtPath(path+links[name]);UsdGeom.Imageable(link).MakeInvisible()
   n=0
   for child in Usd.PrimRange(link):
    if child.HasAPI(UsdPhysics.CollisionAPI):
     UsdPhysics.CollisionAPI(child).GetCollisionEnabledAttr().Set(False);n+=1
   if n==0:raise RuntimeError('No colliders disabled for '+str(link.GetPath()))
   AUDIT.append(dict(link=str(link.GetPath()),disabled_colliders=n,visibility='invisible'))
  # Complete distal gripper mechanism missing; retain hidden inertial/joint carriers
  # for the native action API. Replace its appearance with an exposed mounting flange.
  import isaaclab.sim as sim
  from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg
  body=stage.GetPrimAtPath(path+links['gripper'])
  for child in list(body.GetChildren()):
   if child.IsA(UsdGeom.Imageable): UsdGeom.Imageable(child).MakeInvisible()
   for node in Usd.PrimRange(child):
    if node.HasAPI(UsdPhysics.CollisionAPI):UsdPhysics.CollisionAPI(node).GetCollisionEnabledAttr().Set(False)
  flange=sim.CylinderCfg(radius=.031,height=.012,collision_props=PhysxCollisionPropertiesCfg(),visual_material=sim.PreviewSurfaceCfg(diffuse_color=(.50,.53,.56),metallic=.7,roughness=.35))
  flange.func(str(body.GetPath())+'/exposed_mount',flange,translation=(0,0,.006))
  socket=sim.CylinderCfg(radius=.011,height=.001,visual_material=sim.PreviewSurfaceCfg(diffuse_color=(.025,.025,.028)))
  socket.func(str(body.GetPath())+'/drive_socket',socket,translation=(0,0,.0126))
  for i,(x,y) in enumerate([(-.019,-.019),(-.019,.019),(.019,-.019),(.019,.019)]):
   hole=sim.CylinderCfg(radius=.003,height=.001,visual_material=sim.PreviewSurfaceCfg(diffuse_color=(.025,.025,.028)))
   hole.func(str(body.GetPath())+'/mount_hole_'+str(i),hole,translation=(x,y,.0127))
  return prim
 def configured(*args,**kwargs):
  cfg=original_cfg(*args,**kwargs)
  if 'LeftArm' in cfg.prim_path:
   cfg.spawn.func=spawn_damaged;cfg.spawn.make_uninstanceable=True
   import json
   from pathlib import Path
   q=json.loads(Path(__file__).with_name("initial_pose.json").read_text())["left_joints"]
   cfg.init_state.joint_pos.update({f"joint{i+1}":v for i,v in enumerate(q)})
  return cfg
 emb.yam_arm_cfg=configured
