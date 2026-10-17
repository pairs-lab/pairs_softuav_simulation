# PAIRS Soft UAV Simulation

> **Branch: `ros1`** — this branch adds a ROS 1 (catkin / rospy) wrapper around
> the simulation. See [**ROS 1 wrapper**](#ros-1-wrapper) below. The plain Python
> demos still work standalone; the `main` branch has no ROS dependency.

MuJoCo drone simulations with two independent demos:

- **HoLoArm CAD** — the supplied CAD geometry, mass/rotor layout, nonlinear
  stiffness and v2 viscoelastic parameters, shared by the narrow-passage viewer,
  wall-impact study and ROS adapter. Based on Pham et al., *HoLoArm: Deformable
  Arms for Collision-Tolerant Quadrotor Flight* (RA-L 2026) — see
  [holoarm/README.md](holoarm/README.md) for the citation.
- **Skydio X2** — keyboard teleop of a Skydio X2 in a furnished indoor house.
  Needs the external [`mujoco_menagerie`](https://github.com/google-deepmind/mujoco_menagerie)
  model (one extra download — see below).

The wall study compares soft HoLoArm with the same CAD and mass with locked
arms. See [CAD parameters and assumptions](holoarm/CAD_MODEL.md).

## Branches

This repo is organized by runtime — clone the branch that matches your stack:

| Branch | Contents |
| --- | --- |
| `main` | Standalone Python + MuJoCo demos, no ROS |
| `ros1` | ROS 1 (catkin / rospy) wrapper node — **you are here** |
| `ros2` | ROS 2 (ament_python / rclpy) wrapper node |

```bash
git clone -b ros1 https://github.com/pairs-lab/pairs_softuav_simulation.git   # ROS 1
```

## Structure

```
pairs_softuav_simulation/
├── holoarm/
│   ├── models/holoarm_cad/           # CAD meshes, skins, frozen material/flight data
│   ├── models/scene_config.json      # wall and passage dimensions
│   ├── cad_model.py                 # CAD builder + nonlinear viscoelastic forces
│   ├── holoarm_narrow_passage_viewer.py # current interactive narrow-gap demo
│   └── wall_collision_comparison.py  # soft-vs-locked-CAD MP4 renderer
├── skydio_x2/
│   ├── models/
│   │   ├── indoor_env.xml            # house environment; includes the X2 model
│   │   └── mujoco_menagerie-main/    # downloaded (see Run)
│   └── teleop.py                     # run this
└── requirements.txt
```

## Install

You need Python 3.9+ (tested on 3.12) and a display for the viewer.

Using [uv](https://github.com/astral-sh/uv) (recommended, no sudo):

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
source $HOME/.local/bin/env
uv --version
uv venv --python 3.12 .venv
uv pip install --python .venv -r requirements.txt
source .venv/bin/activate
```

Or plain venv (needs `python3-venv`, e.g. `sudo apt install python3.12-venv`):

```bash
sudo apt install python3.12-venv
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
```

## Run

HoLoArm CAD narrow-passage viewer:

```bash
python3 holoarm/holoarm_narrow_passage_viewer.py
```

Interactive wall impact and headless passage check:

```bash
python3 holoarm/cad_wall_viewer.py --speed .6
python3 holoarm/holoarm_narrow_passage_viewer.py --smoke-test
```

The CAD comparison renderer uses FFmpeg bundled in `requirements.txt`:

```bash
# Render a single impact
python holoarm/wall_collision_comparison.py --impact-speed .6 --duration 1.5

# Render CAD comparisons at 0.3, 0.6 and 0.9 m/s
python holoarm/wall_collision_comparison.py --sweep \
  --results-dir outputs/wall_collision_cad
```

Use `--output /path/to/result.mp4` to set the single-video output path.
Add `--overwrite` to rerun in the same directory and replace existing target videos.
The script automatically uses EGL when no desktop display is available.
The speed sweep produces MP4 videos, CSV/JSON metrics, and plots.

Edit `holoarm/models/scene_config.json` for wall/passage dimensions, or use
`--wall-thickness` for wall impacts and `--passage-thickness` for the passage.
Rigid-arm fracture is disabled by default; `--assumed-fracture` explicitly
enables the old, uncalibrated thresholds. See the
[methodology and limitations](holoarm/WALL_COLLISION_STUDY.md).

On Windows, install 64-bit Python with the `py` launcher (Python 3.12 is
recommended), then double-click `RUN_HOLOARM_WINDOWS.bat` in the repository
root. It creates `.venv`, installs `requirements-windows.txt` on the first run
(Internet access required), and opens the CAD launcher for the narrow-passage
viewer, wall-impact viewer and video export. These demos run without ROS or
WSL. Rendering needs a working OpenGL graphics driver; FFmpeg is bundled with
the Python dependencies.

Historical behavior scripts use `holoarm/legacy_collision_study.py` and their
original illustrative mechanics. Their 2–7.6 m/s results and X2-style artwork
are separate from the current CAD model; see [legacy behavior notes](holoarm/RIGID_WALL_BEHAVIOR.md).

Skydio X2 indoor teleop — first fetch the drone model into `skydio_x2/models/`:

```bash
cd skydio_x2/models
curl -L -o menagerie.zip https://github.com/google-deepmind/mujoco_menagerie/archive/refs/heads/main.zip
unzip -q menagerie.zip && rm menagerie.zip   # extracts to mujoco_menagerie-main/
cd ../.. && python skydio_x2/teleop.py
```

Run interactive MuJoCo viewers from a real desktop session.

## Controls

| Key | Action |
| --- | --- |
| `↑`/`↓` or `i`/`k` | forward / backward |
| `←`/`→` or `j`/`l` | left / right |
| `u` / `o` | up / down |
| `ESC` | quit |

## ROS 1 wrapper

This branch wraps the simulation in a catkin package (`pairs_softuav_simulation`).
The node `drone_sim_node.py` uses the same CAD model and `Flight` controller as
the HoLoArm viewer, with a ROS setpoint topic. The separate Skydio backend uses
`src/pairs_softuav_sim/controller.py`.

| Interface | Topic | Type |
| --- | --- | --- |
| Subscribe | `~target` | `geometry_msgs/Point` (world-frame setpoint, m) |
| Publish | `~odom` | `nav_msgs/Odometry` (drone state at sim rate) |
| Broadcast | TF | `odom` → `base_link` |

Place this repo in a catkin workspace `src/`, then build and run:

```bash
cd ~/catkin_ws && catkin_make        # or catkin build
source devel/setup.bash

# MuJoCo + deps must be on the Python that ROS uses:
pip install mujoco simple-pid numpy

roslaunch pairs_softuav_simulation holoarm_sim.launch
# fly it:
rostopic pub -1 /drone_sim/target geometry_msgs/Point "{x: 1.5, y: 0.0, z: 1.5}"
```

The `skydio_x2_sim.launch` target additionally needs the `mujoco_menagerie`
download (see [skydio_x2/README.md](skydio_x2/README.md)). This node runs the
physics headlessly and publishes state; it does not open the MuJoCo GUI viewer.

## Credits

[MuJoCo](https://github.com/google-deepmind/mujoco) ·
[MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) ·
[simple-pid](https://github.com/m-lundberg/simple-pid)
