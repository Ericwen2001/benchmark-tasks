# Recorded evidence

| Recording | Control steps | Simulated time | Score |
|---|---:|---:|---:|
| `demo.mp4` | 1,420 | 71 s | 1.0 |
| `model_run.mp4` (Dream-1) | 1,800 | 90 s | 0.0 |

Each video shows Top | Left wrist | Right wrist native RGB observations, encoded as H.264/yuv420p. `setup.png` shows the initial observations.

`score_trace.py` evaluates the measured cable, strap and robot states. `demo_score.json`, `model_score.json` and `recording_summary.json` contain the evaluated results.

See [Implementation and recording notes](README.md#implementation-and-recording-notes) for the fastening model and recording details.
