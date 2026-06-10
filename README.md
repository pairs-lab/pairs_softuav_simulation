# PAIRS Soft UAV Simulation

> **Branch: `ros2`** — this branch adds a ROS 2 (ament_python / rclpy) wrapper
> around the simulation. See [**ROS 2 wrapper**](#ros-2-wrapper) below. The plain
> Python demos still work standalone; the `main` branch has no ROS dependency.

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
| `ros1` | ROS 1 (catkin / rospy) wrapper node |
| `ros2` | ROS 2 (ament_python / rclpy) wrapper node — **you are here** |

```bash
git clone -b ros2 https://github.com/pairs-lab/pairs_softuav_simulation.git   # ROS 2
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

## ROS 2 wrapper

This branch is an ament_python package (`pairs_softuav_simulation`). The node
`drone_sim_node` runs the cascaded-PID flight controller from
`pairs_softuav_simulation/controller.py`, replacing the keyboard teleop with a
ROS setpoint topic.

| Interface | Topic | Type |
| --- | --- | --- |
| Subscribe | `target` | `geometry_msgs/Point` (world-frame setpoint, m) |
| Publish | `odom` | `nav_msgs/Odometry` (drone state at sim rate) |
| Broadcast | TF | `odom` → `base_link` |

Place this repo in a colcon workspace `src/`, then build and run:

```bash
cd ~/ros2_ws && colcon build --packages-select pairs_softuav_simulation
source install/setup.bash

# MuJoCo + deps must be on the Python that ROS uses:
pip install mujoco simple-pid numpy

ros2 launch pairs_softuav_simulation holoarm_sim.launch.py
# fly it:
ros2 topic pub --once /target geometry_msgs/msg/Point "{x: 1.5, y: 0.0, z: 1.5}"
```

The `skydio_x2_sim.launch.py` target additionally needs the `mujoco_menagerie`
download (see [skydio_x2/README.md](skydio_x2/README.md)). This node runs the
physics headlessly and publishes state; it does not open the MuJoCo GUI viewer.

## Credits

[MuJoCo](https://github.com/google-deepmind/mujoco) ·
[MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) ·
[simple-pid](https://github.com/m-lundberg/simple-pid)
