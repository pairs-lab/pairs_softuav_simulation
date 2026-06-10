# HoLoArm Drone

MuJoCo model and collision test for **HoLoArm**, a quadrotor with deformable
("soft") arms for collision-tolerant flight.

This implementation is based on the paper below — please cite it if you use this
model or build on it.

## Contents

```
holoarm/
├── models/
│   ├── holoarm.xml                 # HoLoArm core model
│   └── holoarm_narrow_passage.xml  # HoLoArm in a narrow V-shaped passage
└── narrow_passage_test.py          # fly the HoLoArm through the narrow gap
```

## Run

From the project root (with the virtual environment activated — see the
[top-level README](../README.md)):

```bash
python holoarm/narrow_passage_test.py
```

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
