"""Staged rubric: confirmed pickup plus post-trial RGB image recovery."""
import math

def grade(grasp_achieved, recovery):
    if not isinstance(grasp_achieved, bool):
        raise TypeError('grasp_achieved must be a measured boolean')
    if not math.isfinite(recovery):
        raise ValueError('image recovery must be finite; invalid photo assays are not scored')
    if not grasp_achieved:
        return 0.0
    if recovery >= .90:
        return 1.0
    if recovery >= .50:
        return .75
    if recovery >= .25:
        return .50
    return .25

if __name__ == '__main__':
    import argparse,json
    from pathlib import Path
    p=argparse.ArgumentParser();p.add_argument('--trial',required=True);p.add_argument('--images',required=True);p.add_argument('--out',required=True);a=p.parse_args()
    trial=json.loads(Path(a.trial).read_text());images=json.loads(Path(a.images).read_text());r=float(images['recovery_fraction_raw']);g=trial['grasp_achieved']
    result={'score':grade(g,r),'grasp_achieved':g,'first_grasp_step':trial['first_grasp_step'],'image_error_recovery':r,'tiers':[{'condition':'confirmed brush pickup','score':.25},{'condition':'pickup and recovery >= 25%','score':.50},{'condition':'pickup and recovery >= 50%','score':.75},{'condition':'pickup and recovery >= 90%','score':1.0}],'standalone_contact_score':False,'recovery_uses_final_post_trial_images':True,'robot_camera_alignment_requirement':False,'status':'evaluated simulation trial'}
    Path(a.out).write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
