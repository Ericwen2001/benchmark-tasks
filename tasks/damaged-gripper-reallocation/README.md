# Complete both fruit transfers with a missing left gripper

An apple and matching red bowl are on the left, and an orange and blue bowl on the right. The left gripper is missing; the right gripper is intact. The instruction neither mentions damage nor specifies an arm. The task measures completion under asymmetric manipulation capability. Scoring does not require a particular arm choice.

Instruction: Put the orange in the blue bowl and the apple in the red bowl.

Left gripper and finger visuals/colliders removed; invisible native carrier joints and inertial properties retained to preserve the standard 14D interface. Exposed flange has collision geometry. Native rigid fruit and hollow-bowl assets; fixed bowls. No teleport or artificial grasp attachment. AABB containment approximates placement.

## Installation and execution
Use Linux with an RTX GPU and the yam-arena Isaac environment. Tested: Isaac Sim 6.1, Isaac Lab 3, Isaac Lab Arena 0.3.1, Python environment from yam-arena setup; yam-arena commit bd62771e60320433893bd1d2df0fc79b5ddcd6d1.
Repository: https://github.com/kairosiann/yam-isaac-lab-arena . Follow its installation instructions and activate its yam.env before running. The asset_mirror.py helper uses NVIDIA production 6.1 URLs for the same assets where staging URLs return 404; asset_mirror.json records the resolved paths.

From this task directory:
```bash
python test_scorer.py
python run_demo.py --out ./runs/demo
python run_dream.py --max-steps 1800 --out ./runs/dream
```
Dream-1 requires dreamlabs-client and inspect-robots-dream, and a private ~/.dreamlabs/robocurve_credentials file containing DREAM_API_KEY=your_key. No credential is supplied. Dream-1 beta receives three native camera streams and the measured 14D joint/gripper state. Both arms are model-controlled. No scripted action assists the model. Rendering is headless. API wait time pauses simulation.

## Reproduction and evidence
One fixed layout, no object jitter; every trial starts in a fresh process. Dream adapter scene init_seed=42; this is not a guarantee of deterministic server sampling. No multi-seed robustness claim is made. Control is 20 Hz; model limit 1800 steps / 90 simulated seconds. Videos record synchronized native top, left-wrist and right-wrist observations at 4 fps, played at real simulated time. No overview camera is presented.

Full-score scripted demo proves simulation feasibility only. Failure by Dream-1 alone does not establish a specific reasoning deficit. Both videos use the scene and scorer shipped here. See task.json for numeric scoring and evidence/*.json for measured outcomes. Physical materials listed in setup are intended counterparts; simulation uses rigid-body meshes and simplified contact.


## Recorded results and execution protocol
Dream-1 beta: 0.5; scripted demo: 1.0. Model trial: 1800 actions, one continuous session. Wall time excluding initialization: 474.3 s. 221 requests; 8 actions per request. Native rendering640x360, policy input resized320x176. Exactly one session; expiration or changed session aborts the run. No keepalive calls or scripted model assistance. Small normalized gripper overshoots within [-0.1,1.1] are clipped to [0,1]; larger excursions abort.

Both supplied videos use the latest scene geometry and initial posture. Videos contain three synchronized native observations at4fps; top labels only, no bottom information bar. Demo is scripted feasibility, not model performance. No changes were made to obtain a better Dream score.
