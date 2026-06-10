# PAIRS Soft UAV Simulation

> **Branch: `ros1`** — this branch adds a ROS 1 (catkin / rospy) wrapper around
> the simulation. See [**ROS 1 wrapper**](#ros-1-wrapper) below. The plain Python
> demos still work standalone; the `main` branch has no ROS dependency.

MuJoCo drone simulations with cascaded PID control. Two independent demos:

- **HoloArm** — a soft-arm quadrotor flown through a narrow V-shaped gap to test
  collision behavior. Self-contained. Based on Pham et al., *HoLoArm: Deformable
  Arms for Collision-Tolerant Quadrotor Flight* (RA-L 2026) — see
  [holoarm/README.md](holoarm/README.md) for the citation.
- **Skydio X2** — keyboard teleop of a Skydio X2 in a furnished indoor house.
  Needs the external [`mujoco_menagerie`](https://github.com/google-deepmind/mujoco_menagerie)
  model (one extra download — see below).

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
│   ├── models/                       # holoarm.xml + holoarm_narrow_passage.xml
│   └── narrow_passage_test.py        # run this
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

HoloArm narrow-passage test — no extra setup:

```bash
python holoarm/narrow_passage_test.py
```

Skydio X2 indoor teleop — first fetch the drone model into `skydio_x2/models/`:

```bash
cd skydio_x2/models
curl -L -o menagerie.zip https://github.com/google-deepmind/mujoco_menagerie/archive/refs/heads/main.zip
unzip -q menagerie.zip && rm menagerie.zip   # extracts to mujoco_menagerie-main/
cd ../.. && python skydio_x2/teleop.py
```

Each opens an interactive MuJoCo viewer, so run it from a real desktop session.

## Controls

| Key | Action |
| --- | --- |
| `↑`/`↓` or `i`/`k` | forward / backward |
| `←`/`→` or `j`/`l` | left / right |
| `u` / `o` | up / down |
| `ESC` | quit |

## ROS 1 wrapper

This branch wraps the simulation in a catkin package (`pairs_softuav_simulation`).
The node `drone_sim_node.py` runs the cascaded-PID flight controller from
`src/pairs_softuav_sim/controller.py`, replacing the keyboard teleop with a ROS
setpoint topic.

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
