# HoLoArm CAD simulation

The soft vehicle uses the supplied full-drone CAD, nonlinear loading stiffness
and one-arm v2 viscoelastic parameters. The narrow-passage viewer, soft wall
trial and ROS adapter share the model builder and material integration. The
wall comparison defaults to the previous hard-arm model for the rigid vehicle.
See [CAD parameters and assumptions](CAD_MODEL.md) for the parameter files, units,
motor numbering and remaining modeling assumptions.

## Contents

```
holoarm/
├── models/holoarm_cad/             # CAD XML, meshes, skins and frozen parameters
├── models/scene_config.json        # wall/passage dimensions
├── models/wall_collision_config.json
├── cad_model.py                    # CAD builder and material forces
├── hard_arm_model.py               # previous rigid geometry with X2-inspired artwork
├── holoarm_narrow_passage_viewer.py # interactive flight and headless passage check
├── cad_wall_viewer.py              # interactive wall-impact experiment
└── wall_collision_comparison.py    # soft CAD vs hard-arm videos and metrics
```

## Run

From the repository root with its virtual environment activated:

```bash
python holoarm/holoarm_narrow_passage_viewer.py --gap .46 --speed .6
python holoarm/cad_wall_viewer.py --speed .6
python holoarm/wall_collision_comparison.py --impact-speed .6 --duration 1.5
python holoarm/wall_collision_comparison.py --sweep --results-dir outputs/wall_collision_cad
```

The default sweep uses 0.3, 0.6 and 0.9 m/s. Use `--speeds` for custom speeds,
`--no-video` for metrics only and `--overwrite` to replace existing target
videos. FFmpeg is bundled by `imageio-ffmpeg` in `requirements.txt`.
The default `--rigid-model hard-arm` uses the previous rigid model with its
X2-inspired artwork. Total mass, controller and test timing match the soft
trial, while rigid geometry, rotor layout and contact envelope remain distinct.
The artwork does not represent identified Skydio X2 dynamics. Use
`--rigid-model locked-cad` for the optional reference with the same HoLoArm CAD
and locked arms. Fracture is disabled by default; `--assumed-fracture` explicitly
enables legacy, uncalibrated damage thresholds. See
[wall-test methodology](WALL_COLLISION_STUDY.md).

Wall and passage dimensions are full dimensions in metres, configured in
`models/scene_config.json`. `--wall-thickness` overrides impact-wall thickness
while preserving its front face. `--passage-thickness` changes obstacle depth
along world X while preserving the clear gap and guide connections.

Windows users can open `RUN_HOLOARM_WINDOWS.bat` in the repository root; see
[Windows setup](../README.md#run). Linux interactive demos
use an X11 desktop. Both platforms need a working OpenGL driver for rendering.

## Narrow-passage controls

Click the MuJoCo window to focus it. Hold movement keys to fly and release them
to brake and hold position. The default clear gap is 46 cm and speed limit is
0.6 m/s. Press F for close/top views of arm folding.

| Keys | Action |
| --- | --- |
| I / K or Up / Down | Forward / backward along world X |
| J / L or Left / Right | Left / right along world Y |
| U / O | Up / down |
| Q / E | Yaw left / right |
| H | Brake and hold current position/yaw |
| R | Reset drone, controller and material memory |
| Space | Pause / resume |
| F | Cycle overview, close-follow and top-down cameras |
| Esc | Close viewer |

Losing focus releases movement commands. Control keys are captured only for
the focused MuJoCo window. The wall viewer uses Space to pause, R to restart,
S to save metrics and F to change view.

## Verification

```bash
python -m unittest discover -s holoarm -p 'test_*.py'
python holoarm/holoarm_narrow_passage_viewer.py --smoke-test
```

CAD tests check frozen hashes, geometry, mass, rotor coordinates, damping,
contact masks, wall/passage geometry and material response. Viewer and adapter
tests check shared dynamics and input behavior without requiring ROS. The
headless smoke test checks hover, contact, passage and recovery.

Historical `behavior`, `restored`, `compression_then_lean` and `rigid_offset`
scripts use `legacy_collision_study.py` and the original illustrative model.
`holoarm_physics.xml`, `soft_holoarm_visual.xml` and `rigid_x2_visual.xml` are
retained for those scenarios and the current hard-arm reference. Their old
videos and test results do not describe the updated soft CAD. See
[legacy behavior notes](RIGID_WALL_BEHAVIOR.md).

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
