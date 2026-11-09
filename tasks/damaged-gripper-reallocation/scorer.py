"""Fruit-in-bowl: measured containment, prior lift, release and stable dwell."""
from dataclasses import dataclass
import numpy as np
def contained_fraction(fruit,bowl):
 overlap=np.maximum(0.,np.minimum(fruit.high,bowl.high)-np.maximum(fruit.low,bowl.low))
 return float(np.prod(overlap)/max(float(np.prod(fruit.high-fruit.low)),1e-12))
@dataclass
class FruitScorer:
 lifted:bool=False
 stable_seconds:float=0.
 def update(self,fruit,bowl,tcp_positions,dt,table_z):
  self.lifted |= bool(fruit.low[2]>table_z+.02)
  fraction=contained_fraction(fruit,bowl)
  inside=fraction>=.5 and np.linalg.norm(fruit.pos[:2]-bowl.pos[:2])<.045
  released=all(np.linalg.norm(fruit.pos-np.asarray(t))>.10 for t in tcp_positions)
  stationary=fruit.resting(max_speed=.01,max_spin=.5)
  self.stable_seconds=self.stable_seconds+dt if self.lifted and inside and released and stationary else 0.
  return dict(score=float(self.stable_seconds>=2),lifted=self.lifted,inside=bool(inside),contained_fraction=fraction,released=bool(released),stationary=bool(stationary),stable_seconds=self.stable_seconds)
