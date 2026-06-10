# Skydio X2 Drone

Keyboard teleop of a [Skydio X2](https://github.com/google-deepmind/mujoco_menagerie/tree/main/skydio_x2)
quadrotor flying inside a furnished indoor house, driven by a cascaded
(velocity → attitude → rate) PID controller.

## Contents

```
skydio_x2/
├── models/
│   ├── indoor_env.xml          # house environment; includes the X2 model
│   └── mujoco_menagerie-main/  # downloaded (see Setup) — the Skydio X2 model
├── teleop.py                   # run this
└── LICENSE                     # MIT — upstream keyboard-teleop project this is based on
```

> The Skydio X2 *model* itself is licensed separately (Apache-2.0) and ships with
> the `mujoco_menagerie` download, not in this repo.

## Setup

The Skydio X2 model is not bundled. Fetch it once into `models/`:

```bash
cd models
curl -L -o menagerie.zip https://github.com/google-deepmind/mujoco_menagerie/archive/refs/heads/main.zip
unzip -q menagerie.zip && rm menagerie.zip   # extracts to mujoco_menagerie-main/
cd ..
```

## Run

From the project root (with the virtual environment activated — see the
[top-level README](../README.md)):

```bash
python skydio_x2/teleop.py
```

Controls are listed in the [top-level README](../README.md#controls).

## Acknowledgements

- [MuJoCo](https://github.com/google-deepmind/mujoco)
- [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) (Skydio X2 model)
- [simple-pid](https://github.com/m-lundberg/simple-pid)
