import numpy as np
from types import SimpleNamespace
from scorer import FruitScorer

def obj(x,y,z,h=(.03,.03,.03)):
 p=np.array([x,y,z]);h=np.array(h)
 return SimpleNamespace(pos=p,low=p-h,high=p+h,resting=lambda **kw:True)
bowl=obj(0,0,.029,(.083,.083,.029));rest=obj(0,0,.035);lift=obj(0,0,.12)
far=[[1,1,1],[1,-1,1]]
s=FruitScorer()
for _ in range(45):a=s.update(rest,bowl,far,.05,0)
assert a['score']==0
s.update(lift,bowl,far,.05,0)
for _ in range(45):a=s.update(rest,bowl,[[0,0,.04],[1,1,1]],.05,0)
assert a['score']==0
for _ in range(45):a=s.update(rest,bowl,far,.05,0)
assert a['score']==1
wrong=obj(.2,0,.035)
assert s.update(wrong,bowl,far,.05,0)['score']==0
rim=obj(0,0,.09)
assert s.update(rim,bowl,far,.05,0)['score']==0
print('PASS: no-lift, held, correctly-contained stable, wrong bowl, above rim')
