# HoLoArm Drone

MuJoCo model and collision test for **HoLoArm**, a quadrotor with deformable
("soft") arms for collision-tolerant flight.

This implementation is based on the paper below — please cite it if you use this
model or build on it.

## Contents

```
holoarm/
├── models/
│   ├── holoarm_physics.xml         # internal mechanics template, not a demo
│   └── soft_holoarm_visual.xml     # current HoLoArm appearance assets
├── holoarm_narrow_passage_viewer.py # current scene + stable folding-aware flight
└── wall_collision_comparison.py    # render soft-vs-rigid comparison video
```

## Run

From the project root (with the virtual environment activated — see the
[top-level README](../README.md)):

```bash
python3 holoarm/holoarm_narrow_passage_viewer.py
```

Only the current reference-style HoLoArm narrow-passage demo is supported.
The previous demo and its standalone scene XML have been removed. ROS uses
the same `Flight` class, model and 46 cm open-top passage, not the old PID demo.
`models/holoarm_physics.xml` is an internal mechanics template; the builder
attaches the current artwork before running. Launch the viewer above rather
than loading that template as a standalone visual model.

To render a reproducible side-by-side wall impact comparison:

```bash
python holoarm/wall_collision_comparison.py
```

This writes `outputs/wall_collision_recovery.mp4`. Both vehicles have the same
mass, approach speed and closed-loop PD controller. Soft arms passively bend;
rigid arms detach only when contact loads exceed the assumed fracture limits.
The video includes a close-up and slow-motion impact replay. To survey 12 speeds
(2–7 m/s in 0.5 m/s increments, plus 7.6 m/s):

```bash
python holoarm/wall_collision_comparison.py --sweep
```

Results, videos and plots are saved in `outputs/wall_collision_study/`. Use
`--no-fracture --no-video` for an unbreakable rigid control case. See
[the methodology](WALL_COLLISION_STUDY.md) for assumptions, recovery criteria,
reference-video interpretation and the distinction from the paper's drop tests.
FFmpeg is bundled by `imageio-ffmpeg`, installed from `requirements.txt`.

The rigid drone now uses original X2-inspired visual geometry from
`models/rigid_x2_visual.xml`: a dark shell, dual cameras, rear antennas, carbon
booms and silver motors. Its matched collision envelope and inertia are retained
to preserve the comparison; it is not an identified Skydio X2 dynamics model.
Updated survey clips are available in `outputs/wall_collision_x2_style/` when
rendered with `--sweep --results-dir outputs/wall_collision_x2_style`.

The soft visual now follows HoLoArm's reference appearance: white frame and
guards, black three-blade props, exposed electronics, tracking markers and
curved orange TPU bands following the actual moving joint sites. The artwork
is in `models/soft_holoarm_visual.xml` and `soft_visual.py`; it adds no mass or
contact forces. To render both updated appearances:

```bash
python holoarm/wall_collision_comparison.py --sweep \
  --results-dir outputs/wall_collision_holoarm_style
```

## Interactive reference-style HoLoArm: narrow passage

From the repository root, on a desktop with MuJoCo installed:

```bash
python3 holoarm/holoarm_narrow_passage_viewer.py
```

This uses the new white HoLoArm frame, black props and orange ligament curves
attached to the moving spring/damper joints. A tapered entry leads to a 46 cm
clear slot at X=1.85–2.35 m; the nominal guard-to-guard width is about 55 cm.
The top of the slot is open for observation. There are no floor route markers,
coloured start/goal pads or target sphere.

Click the MuJoCo window to focus it. Keyboard controls:

| Keys | Action |
| --- | --- |
| I / K or Up / Down | Forward / backward along world X |
| J / L or Left / Right | Left / right along world Y |
| U / O | Up / down |
| Q / E | Yaw left / right |
| H | Brake and hold current position/yaw |
| R | Reset drone and controller |
| Space | Pause / resume |
| F | Cycle overview, close follow and top-down cameras |
| Esc | Close viewer |

Hold movement keys for continuous velocity control; release them to brake and
hold the current position. Physical key states are read every frame, without
OS repeat delays or fixed position increments. Losing window focus also
brakes. Holding Q/E turns continuously at 60 degrees/s. Default horizontal
speed limit is 0.6 m/s. To start, hold I to fly toward the opening, and press F
for a close/top view of folding. Keep I held while pushing through the slot;
there is no scripted folding, teleportation or automatic retreat. Braking is
physical, so it takes a short time rather than instantly zeroing velocity.
H brakes even if a movement key remains held; release/repress that movement
key to resume. Reset and pause also clear movement commands.
You can orbit/zoom with the standard viewer mouse controls in overview mode.

```bash
python3 holoarm/holoarm_narrow_passage_viewer.py --gap 0.48 --speed 0.5
python3 holoarm/holoarm_narrow_passage_viewer.py --smoke-test
python3 -m unittest discover -s holoarm -p 'test_*.py'
```

The headless smoke test checks hover, physical contact, arm bending, passage
and recovery to the goal. This is the project's reduced-order mechanics with
position/yaw feedback and pose-dependent thrust allocation, not the paper's
trained RL controller or a validated hardware digital twin. This interactive
input uses the Linux/X11 desktop (`DISPLAY`, `libX11`). Control keys are grabbed
only on this MuJoCo window to prevent native debug-shortcut flicker; no global
keyboard hook is installed, and other apps are unaffected. Existing wall-impact
videos and the old demo are unchanged.

## Citation

> Q. N. Pham, J. Eschmann, Y. Zhou, A. Ojeda Olarte, G. Loianno, and V. A. Ho,
> "HoLoArm: Deformable Arms for Collision-Tolerant Quadrotor Flight,"
> *IEEE Robotics and Automation Letters*, 2026.

```bibtex
@article{pham2026holoarm,
  title={HoLoArm: Deformable Arms for Collision-Tolerant Quadrotor Flight},
  author={Pham, Quang Ngoc and Eschmann, Jonas and Zhou, Yang and Olarte, Alejandro Ojeda and Loianno, Giuseppe and Ho, Van Anh},
  journal={IEEE Robotics and Automation Letters},
  year={2026},
  publisher={IEEE}
}
```
