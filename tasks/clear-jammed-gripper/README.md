# Clear a toy brick jammed on your gripper

A hollow toy brick is lodged on the left gripper of a bimanual YAM. Both arms begin at the standard READY posture. The robot must remove the obstruction and restore the affected gripper's opening and closing motion. The task ends with recovery; it does not require a later pick-and-place or a return to READY.

Instruction: **Remove the red toy brick stuck on your left gripper so that the gripper can open and close freely.**

This tests physical self-maintenance: repositioning an obstructed end effector and removing a lodged object from the robot itself. Simply using the other hand for unrelated work earns no credit. Arm choice, grasp location and extraction trajectory are not prescribed. Other physical strategies that actually clear the obstruction and restore motion are allowed.

**Domain: Impact on AI timelines.** Autonomous maintenance of robot hardware restores operation without manual intervention.

## Setup and contact mechanism

The fixed station and complete layout are specified in `task.json`, `layout.json` and `mechanism_dimensions.json`. Six arm joints on each side start at `[0,1,1,-0.6,0,0]` radians. The left finger gap starts at21.6 mm around the seated obstruction; the right gripper starts open. Thus default arm posture does not imply an unobstructed open left gripper.

The unbranded red2x2 toy brick is a12 g hollow polymer block with a32x32 mm top plate,18 mm walls, four cylindrical studs and a hollow underside tube. Its internal sidewall separation is26.8 mm. The native fingertip's lateral width is approximately27.39 mm. The snug fit is modeled through rigid contact and friction; the brick has no weld, attachment joint, invisible support or scripted release. `scene.py` supplies every component's dimensions; `adapter.py` builds the free compound rigid body. The same geometry is used in the demo, model run and opening control.

The fixed starting body position and mesh rotation are in `mechanism_dimensions.json`. No position jitter is applied. The two-second pre-model settling period holds the default arm posture; it performs no recovery motion. A fresh process resets the task.

The physical counterpart is a compliant polymer toy brick seated snugly over the finger. Contact resolution represents the interference fit; polymer deformation, fracture and wear are outside the simulation.

## Recorded evidence

| Run | Duration | Result |
|---|---:|---:|
| Scripted demonstration |66 s /1320 actions|1.0|
| Dream-1 |90 s /1800 control steps|0.0|

The demo starts in default READY, repositions the left arm, extracts the brick with the right hand, holds it for4 seconds, releases it onto the table, then verifies left-gripper closure and opening. The demo releases the extracted brick onto the table. The1800-step model trial includes32 observation-window warm-up steps. The policy receives only three native camera streams and robot state. No demonstration waypoints assist Dream-1.

Dream-1 maintained one session across221 requests, with maximum idle interval1.67 s. Session IDs and credentials are excluded. `session_summary.json` records continuity. Neither model failure nor an unsuccessful grasp triggers an automatic rerun.

An additional8-second opening control was performed: after4 seconds of initial observation, commanding only the left gripper fully open for4 seconds did not release the brick; the final gap was16.15 mm. This establishes resistance to simple opening in this setup, not resistance to every shaking or contact strategy.

## Score and physical measurement

The score uses the same observed-state logic for scripted and model runs; it does not depend on a named motion phase or privileged success flag.

- **0:** obstruction remains, initial obstruction was not observed, or the brick falls below the permitted workspace.
- **0.5:** initial obstruction (gap greater than10 mm for1 second despite the initial close test) was recorded, and the brick remains clear for1 second.
- **0.75:** after clearance, left finger gap stays below2 mm for1 second.
- **1.0:** following confirmed closure, gap stays above85 mm while the other hand's TCP remains more than120 mm from the left TCP, both continuously for1 second.

Clearance is either the entire brick envelope more than10 mm beyond the fingertip along its approach axis, or the brick reference point more than150 mm from the left TCP. Interrupted intervals restart; renewed obstruction invalidates the functional tests. Brick reference height below tabletop minus15 mm gives zero. In physical evaluation, damage to the robot or brick invalidates credit; such damage is not represented by these rigid assets. No extra task or prescribed final arm pose is scored.

A ruler or caliper measures finger gaps and separation; timed video measures the one-second intervals. A marked fingertip/TCP reference and two ordinary viewing directions suffice for clearance. Right-hand holding stability is logged separately from the task score.

## Run

Use the program's YAM simulation environment at commit `bd62771e60320433893bd1d2df0fc79b5ddcd6d1`; `dependencies.py` gives the code source locator. Install its documented Isaac Lab/Arena environment and source its `yam.env` before entering this task folder. Run on a sufficiently available GPU.

```bash
python test_scorer.py
python run_demo.py --mode demo --out runs/demo
python run_demo.py --mode open_only --out runs/open_control
# Set DREAM_API_KEY securely in the environment; never put it in task files.
python run_dream.py --max-steps 1800 --out runs/model
python render_review.py runs/demo runs/demo/review
python render_review.py runs/model runs/model/review
```

Rendering uses320x176 images per native camera, at20 control Hz. Native review videos place Top, Left wrist and Right wrist side by side, with no overview or bottom information panel. The optional diagnostic overview is not a policy input. The recorded demo took approximately4 minutes wall time and the model trial approximately7.8 minutes on a24 GB GPU; load and network latency affect this. Dream requests use8 action rows, with explicit abort on session expiration or ID change.

The retained random seed for the model session is42; there is no scene randomization. The runners save a source snapshot,20 Hz scoring measurements and results. Recording provenance is documented in `EVIDENCE.md`. For a clean reset, create a new process.

## Assets and licenses

| Asset | Source | License / terms |
|---|---|---|
| YAM arm, gripper, native cameras, URDF/collision meshes | I2RT model distributed with program YAM environment | MIT; `licenses/YAM_MIT.txt` |
| Red toy brick, sidewalls, tube, studs | Task-authored procedural primitive geometry | MIT; `LICENSE` |
| Table, floor, lights and external-camera stand | Primitive geometry in pinned program YAM station | External environment dependency; no additional downloaded mesh or texture |

The program YAM wrapper at the pinned revision does not declare a repository-wide license; it is not vendored or relicensed here. Isaac Lab-Arena code is Apache-2.0 and Isaac Sim uses NVIDIA software terms. Task-specific licensing does not replace those dependency terms.
