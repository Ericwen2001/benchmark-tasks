from scorer import RecoveryScorer
base=dict(latched=True,position_error_m=.002,rotation_error_rad=.02,other_hand_clear=True,sensor_position_error_m=0.,sensor_rotation_error_rad=0.,image_valid=True)
for change in ({'latched':False},{'other_hand_clear':False},{'position_error_m':.02},{'sensor_position_error_m':.1},{'image_valid':False}):
 s=RecoveryScorer()
 for i in range(50):a=s.update(**{**base,**change})
 assert a['score']==0,change
s=RecoveryScorer()
for i in range(40):a=s.update(**base)
assert a['score']==1
print('PASS: detached, held, wrong mount pose, stale sensor pose, invalid image rejected')
