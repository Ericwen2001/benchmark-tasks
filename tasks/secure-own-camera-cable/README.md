# Secure your loose wrist-camera cable

The robot routes its own loose wrist-camera cable over a retaining peg and closes a hook-and-loop strap. The final state requires the working hand to release and move away while the cable remains secured.

This tests robot self-maintenance: restoring a cable-management fixture on the robot itself. The working hand must leave the cable and strap secured on the arm.

## Setup and mechanism

`task.json` describes the fixed scene; `layout.json` summarizes dimensions and initial robot joints. `hardware.py` constructs the camera housing, cable, side-mounted peg and articulated strap. The left wrist camera retains its calibrated optical pose. The policy receives the standard top, left-wrist and right-wrist RGB images and14D joint/gripper state.

The strap is a semi-rigid polymer-backed hook-and-loop textile: the backing holds a shallow outward bow when open and bends during closure. For physical reset, lift its free end away from the pad, restore the outward bow and let the cable hang freely between its two connectors; use the setup image and dimensions to reproduce the span.

The cable remains dynamically articulated and attached at its two connectors. The open strap bends through compliant revolute joints. After the inner strap face contacts the receiving pad and sustains compression for0.2 seconds, `velcro_contact.py` applies an ideal local attachment to the engaged segment; the remaining strip segments retain their elastic joints and dynamics.

## Run and score

Use the official YAM simulation environment from the program walkthrough. `dependencies.py` identifies the pinned YAM source. Dependencies are its Isaac Lab/Isaac Sim runtime, NumPy and SciPy; Dream evaluation also uses the official Dream and Inspect Robots adapters. Video export uses Pillow and `ffmpeg`.

Run these commands from this folder after activating the YAM environment:

```bash
python test_scorer.py
python run_setup.py --out runs/setup --steps 20
python run_complete.py --out runs/demo --steps 1800
python score_trace.py runs/demo
python render_native.py runs/demo

# Set DREAM_API_KEY privately in the shell.
python run_dream.py --out runs/model --steps 1800
python score_trace.py runs/model
python render_native.py runs/model
```

Use a new output directory and a fresh process for every trial. The supplied demo follows a fixed joint plan. Scene initialization is deterministic; the model scene seed is42. Physics runs at240 Hz and policy control at20 Hz. The recorded script stays within the shared90-second limit; exact action counts are in `recording_summary.json`. Wall time depends on GPU load, rendering and API latency; the model run records its wall time in `result.json`.

Three native camera views appear left to right as Top, Left wrist and Right wrist, with no overview or information bar. Both videos are continuous recordings. Dream uses 8 action rows per request.

## Scoring

The rubric in `task.json` and `scorer.py` awards0,0.5,0.75 or1.0 for the final held state. `score_trace.py` replays measured scene geometry and robot state using the same evaluator for scripted and model trials.  Physical evaluation uses the corresponding ruler measurements and timed video.

## Files

The videos and setup image are attached on the task board through the media ID in `task.json`.

- `task.json`, `setup.png`: task and fixed initial observations.
- `demo.mp4`, `model_run.mp4`, `scores.txt`: evidence and evaluated scores.
- `run_complete.py`, `replay_actions.json`: continuous scripted feasibility run.
- `run_dream.py`, `dream_driver.py`: model-only control with native observations.
- `hardware.py`, `cable_usd.py`, `prepare_geometry.py`, `camera_mount.py`: scene construction and geometry.
- `scorer.py`, `score_trace.py`, `render_native.py`: scoring and native-view export.
- `EVIDENCE.md`, `recording_summary.json`: recording details and results.

## Assets and licenses

Task fixtures, cable and strap are procedural primitive geometry, under the bundled MIT license. YAM arm/gripper/camera assets come from the pinned program environment; its MIT notice is in `licenses/YAM_MIT.txt`. Table, floor and lights are station primitives.  Isaac Lab-Arena and Isaac Sim retain their upstream software terms.

## Implementation and recording notes

**Fastening model.** Hook-and-loop engagement is represented by a local contact bond after 0.2 s of compression. The engaged segment is attached; the rest of the strap remains flexible. Scoring checks the resulting placement and hand release rather than the bond flag.


**Recording details.** Both videos are uninterrupted trials. The demo uses a fixed joint plan without intermediate state restoration. Dream-1 used one session and 221 requests, with no restarts. `recording_summary.json` records the evaluated outcomes.
