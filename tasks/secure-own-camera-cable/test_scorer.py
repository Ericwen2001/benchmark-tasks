import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from scorer import Scorer,MOUNT,B
class ScoringTests(unittest.TestCase):
 def scene(self):
  c=MOUNT.copy();wire=[]
  for y in np.arange(40)*(.7217926530318888/40)-.35:wire.append(np.r_[c+B@[.015,y,.013],Rotation.from_matrix(np.column_stack([B[:,1],B[:,2],B[:,0]])).as_quat()])
  R=Rotation.from_matrix(np.column_stack([B[:,2],B[:,1],-B[:,0]]));normal=R.as_matrix()[:,2]
  strap=[np.r_[c+B@[.0355,0,0]-normal*.00115,R.as_quat()]]
  return np.array(wire),np.array(strap),np.eye(4),c+np.array([.3,0,0])
 def test_partial_and_support_free(self):
  w,f,T,tcp=self.scene();s=Scorer()
  for _ in range(8):r=s.update(w,f,T,tcp,.09,.25)
  self.assertEqual(r['score'],1.)
  r=s.update(w,f,T,MOUNT,.09,.25);self.assertEqual(r['score'],.75)
 def test_unsecured_and_disconnected(self):
  w,f,T,tcp=self.scene();s=Scorer();f[:,:3]+=.2
  for _ in range(8):r=s.update(w,f,T,tcp,.09,.25)
  self.assertEqual(r['score'],.5)
  w[1,:3]+=.1;r=s.update(w,f,T,tcp,.09,.25);self.assertEqual(r['score'],0.)
 def test_release_dwell_restarts(self):
  w,f,T,tcp=self.scene();s=Scorer()
  for _ in range(7):r=s.update(w,f,T,tcp,.09,.25)
  self.assertLess(r['score'],1.)
  s.update(w,f,T,tcp,.01,.25)
  r=s.update(w,f,T,tcp,.09,.25);self.assertEqual(r['release_hold_s'],.25)
 def test_air_gap_does_not_fastening_score(self):
  w,f,T,tcp=self.scene();f[:,:3]+=B[:,0]*.009;s=Scorer()
  for _ in range(10):r=s.update(w,f,T,tcp,.09,.25)
  self.assertEqual(r['score'],.5)
 def test_hovering_cable_has_no_route_credit(self):
  w,f,T,tcp=self.scene();w[:,:3]+=B[:,2]*.018;s=Scorer()
  for _ in range(10):r=s.update(w,f,T,tcp,.09,.25)
  self.assertEqual(r['score'],0.)
if __name__=='__main__':unittest.main()
