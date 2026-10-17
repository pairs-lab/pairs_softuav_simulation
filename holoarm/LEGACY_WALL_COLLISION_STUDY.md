# Controlled wall impact survey

Run the 12-case 2–7.6 m/s survey (2–7 m/s in 0.5 m/s increments, plus 7.6 m/s):

```bash
python3 holoarm/wall_collision_comparison.py --sweep
```

There are 15 speeds and two independent trials per speed. Output is placed in
`outputs/wall_collision_study/`: individual MP4 comparisons, `summary.csv`,
`results.json` including all parameters, 200 Hz trajectory/contact traces, and
`speed_sweep.png`. Rendering requires `ffmpeg` and Pillow; plotting requires
Matplotlib. Physics and metrics are independent of video FPS and resolution.
The clips are also joined into `speed_sweep_comparison.mp4`, ordered by survey
speed, without re-encoding. Original wall-demo videos remain available as
historical outputs; use these new clips to assess the corrected controller.

```bash
# Run only the numerical survey, without a renderer or desktop.
python3 holoarm/wall_collision_comparison.py --sweep --no-video
# A single detailed replay; existing videos are never overwritten.
python3 holoarm/wall_collision_comparison.py --impact-speed 7.6 \
  --output outputs/wall_7_6_recovery.mp4
# Control experiment: an unbreakable rigid frame with identical control.
python3 holoarm/wall_collision_comparison.py --speeds 2 4 7.6 \
  --no-fracture --no-video --results-dir outputs/unbreakable_control
```

## Soft drone appearance

The soft replay now has original **HoLoArm-inspired artwork** based on the
[reference video](https://www.youtube.com/watch?v=hmfgFP9XoHQ): an ivory frame and
guard rings, white arm tubes, black three-blade propellers, exposed controller
electronics, reflective tracking-marker balls and orange bowed TPU ligaments.
Artwork is authored in `models/soft_holoarm_visual.xml` and `soft_visual.py`.
It is an approximate visual reconstruction, not the researchers' original CAD.

Added geometry has zero mass and no contact participation. Physical guard
geometry is retained and coloured ivory. Existing spring/damper and contact
settings are retained. Orange curves are drawn between the existing fixed and
moving attachment sites at each render frame, so they deform with actual
arm bending and axial motion. Their shape is illustrative; force computation
continues to use the existing reduced-order joint springs, not the curves.

```bash
python3 holoarm/wall_collision_comparison.py --sweep \
  --results-dir outputs/wall_collision_holoarm_style
```

This produces updated clips and a combined video with the HoLoArm appearance
and the previously added X2-style rigid drone, while retaining earlier videos.

## Rigid drone appearance

The rigid baseline now uses original **Skydio X2-inspired artwork**: a dark,
faceted fuselage, dual front camera lenses, two rear antennas, carbon-coloured
booms, silver motor housings and separate propeller blades. It is labelled
`RIGID / X2-style` in the replay. The visual kit is authored in
`models/rigid_x2_visual.xml`; it does not require Menagerie or external meshes.
The appearance was informed by the
[Menagerie X2 reference](https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/main/skydio_x2/README.md).

This is a change in appearance, **not an actual Skydio X2 dynamics model**.
New visual geometries have zero mass and no collision participation. The old
rigid physical geometry is retained but hidden, including the matched swept
rotor contact envelope. Body mass/inertia, thrust sites and breakable welds
remain the same. All visible arm/motor parts follow their physical detached
assemblies when a weld fails. Tests compare the old and new models' inertial,
joint, actuator and weld arrays to ensure that the visual change preserves the
comparison setup. There is no visible soft-joint hardware or coloured guard
ring on the rigid vehicle.

Updated videos can be rendered to a separate directory to retain earlier clips:

```bash
python3 holoarm/wall_collision_comparison.py --sweep \
  --results-dir outputs/wall_collision_x2_style
```

## What was wrong with the original video

The original wall demo continuously applied the same four motor forces, with
no attitude or altitude feedback. Once compliant arms redirected thrust, the
vehicle could tumble or accelerate back into the wall. Its rotational springs
did not model the real mechanism's directional stiffness. Its sliding joints
also allowed excessive travel under high-speed impact because the default
joint-limit constraints were too soft. Conversely, removing all rigid joints
created an indefinitely strong rigid frame, with no possible arm failure.

The updated demo settles each vehicle in closed-loop hover before injecting
the requested velocity. It logs the **measured** velocity immediately before
first wall contact, and does not brake before that contact. Both vehicles use
the same attitude/position PD controller and a 40 ms first-order motor response.
After contact, the target is a hover point clear of the wall. Motor allocation
uses the current arm positions and thrust axes, including their deformation.
The initial velocity is an imposed test condition, not a simulated flight
acceleration manoeuvre.

Contact first occurs on physical propeller guards. Soft arms have a much stiffer
pitch spring than yaw spring, so nominal rotor thrust does not fold the arms
vertically. Their bending and axial travel respond passively to MuJoCo forces;
no animation, position reset, or recovery teleport is used. Guard contact force
and its moment about each arm root are measured at each physics step.

Rigid arms are separate free bodies held by equality welds. When arm contact
load exceeds either configured limit, its weld is disabled and its motor stops.
The detached arm retains its mass, pose and velocity and falls under physics.
The limits are **assumptions**, not identified PLA strength. `--no-fracture`
provides the essential control case: an intact rigid vehicle may also recover.
Soft-material rupture is not implemented, so an intact soft model does not
establish that real TPU/PLA would remain undamaged at that speed.

## Metrics and success criteria

- First wall-contact time and the immediately preceding velocity.
- Peak total wall force, root contact moment, and unfiltered body X acceleration
  during the first 250 ms. Later floor strikes are excluded from these peaks.
- Peak 10 ms mean body deceleration, calculated from the 2 kHz velocity history.
  This is a stated measurement bandwidth, not a replacement for raw peaks.
- Passive arm bending relative to the pre-impact loaded hover pose, compression
  in mm, and maximum numerical overtravel beyond the nominal 4 mm slide limit.
- Body tilt as the angle between its vertical axis and world vertical, correctly
  identifying an upside-down vehicle.
- Detached arms and floor contact, including event times.
- Hover recovery: within 12 cm of the retreat target, speed below 0.25 m/s,
  tilt below 8 degrees, no wall contact, no detached arms, and no prior ground
  contact, maintained for 0.3 s. The reported time begins at first impact and
  ends at the beginning of that uninterrupted stable interval.

Each video has a top-view close-up and **0.15x slow motion** around impact.
The two panels always use the same physics time. Recovery labels use the
numerical criteria above, rather than a manually specified outcome.

## Reference and scope

[Pham et al., HoLoArm (2026)](https://arxiv.org/html/2605.25790v1) reports:

- Mid-flight disturbances with an RL policy: soft arms facilitate continued
  hovering; rigid failure depends on disturbance direction.
- Drop tests, rather than a powered 7.6 m/s wall flight: equal 670 g test masses,
  heights of 1, 1.5 and 3 m; the rigid frame breaks in the 3 m drop while HoLoArm
  remains intact. These experiments must not be used directly as a wall-impact
  failure-speed calibration.

The [reference video](https://www.youtube.com/watch?v=hmfgFP9XoHQ) similarly shows
hover disturbances, narrow-gap traversal and a separate 3 m drop test.

This repository's survey is a **reduced-order wall-impact study**, not a
reproduction of those experiments. The mass is normalized to 970 g, and rotor
positions, 40 ms motor-response scale and nominal 4 mm axial stroke are informed
by the paper. Guard geometry, joint spring/damping constants, contact solver
parameters, fracture limits and controller gains are approximate. The controller
is PD, not the paper's trained RL policy. Parameters are exposed in
[wall_collision_config.json](models/wall_collision_config.json); result JSON
stores the configuration used.

In particular, stiff axial stops can produce larger **unfiltered** force spikes
in this approximate soft model even when it reduces 10 ms mean deceleration and
recovers successfully. That is a limitation to report, not evidence that the
experimental HoLoArm is worse or a reason to hide the peaks. Quantitative force,
fracture-speed and material-safety conclusions require identified constitutive
laws, measured contact loads and convergence checks. Tests at 8 m/s extend beyond
the paper's reported top speed and are simulation results only.

## Verification

```bash
python3 -m unittest discover -s holoarm -p 'test_collision_study.py' -v
python3 holoarm/validate_wall_collision.py
```

The tests verify matched mass/guard geometry, passive restoring forces,
load-triggered detachment without pose/velocity resets, low-speed recovery and
the unbreakable control case. The audit compares 0.5 ms and 0.25 ms physics steps
at 7.6 m/s, saving both outputs rather than just a pass/fail label. Consistent
hover/fracture outcomes do not validate the remaining axial-stop overtravel or
the exact trajectory of detached bodies.
