# HoLoArm CAD wall-impact study

This study uses the supplied CAD, nonlinear loading laws and v2 material
memory through `cad_model.py`. Both vehicles share CAD geometry, mass and rotor
layout. The soft vehicle has articulated arms; the reference locks those arms
with four welds. Historical X2-style demonstrations and linear-spring results
are described in [legacy methodology](LEGACY_WALL_COLLISION_STUDY.md).

## Run

```bash
python holoarm/wall_collision_comparison.py --impact-speed .6 --duration 1.5
python holoarm/wall_collision_comparison.py --sweep --results-dir outputs/wall_collision_cad
python holoarm/wall_collision_comparison.py --speeds .3 .6 --no-video --results-dir outputs/cad_metrics
python holoarm/validate_wall_collision.py --help
```

The default sweep is 0.3, 0.6 and 0.9 m/s. Use `--overwrite` to replace existing
target videos. Existing reports, CSV/JSON metrics and plots are regenerated in
the chosen results directory; unrelated files are retained. Use a new directory
to keep a prior run. Videos use the FFmpeg bundled in the Python requirements.

## Parameters and protocol

Material and flight data live in `models/holoarm_cad/`; geometry dimensions are
in `models/scene_config.json`. `models/wall_collision_config.json` contains test
timing and recovery criteria. `load_config()` adds the effective CAD parameters
to the configuration recorded in results. See [CAD model](CAD_MODEL.md) for
loading-law coefficients, rheology, mass and retained assumptions.

The default step is 0.0004 s. Each vehicle settles for 0.8 s before approach.
`Trial.prepare_approach()` positions the settled assembly relative to the front
of the wall and sets its incoming velocity before beginning the test. This is
an imposed initial test condition, rather than a simulated acceleration flight.
The study records the actual velocity immediately before first contact.
No pose or velocity correction creates the post-impact response.

The controller uses current rotor poses for force allocation, with a 40 ms
motor response. Nonlinear restoring and Maxwell-memory forces are applied at
each physics step; viscous damping is integrated implicitly once. The neutral
CAD hover geometry assumes a fixed internal preload.

Guard contact is normal-only with ideal tangential slip. `--wall-thickness`
changes the wall's full thickness while preserving its front contact face.
Changing only the back face should not change the initial impact response.

## Metrics and interpretation

Results include actual impact speed, first-contact time, first-impact wall
force and impulse, arm-root moment, raw body acceleration and a 10 ms mean
body deceleration. Peaks for force/acceleration are restricted to the first
250 ms after contact to separate them from later floor strikes.

Arm metrics distinguish bend, positive axial compression, absolute slide
travel and numerical guard overtravel. Nominal slide bounds are [-1, 8] mm and
hinge bounds are +/-0.70 rad. These are numerical guards, not measured stops.

Recovery requires position within 12 cm of the retreat target, speed below
0.25 m/s, tilt below 8 degrees, no wall contact, no detached arms and no prior
floor contact, continuously for 0.3 s. This is a simulation test criterion.

Arm fracture is disabled by default. `--assumed-fracture` enables the original
uncalibrated force/moment thresholds on the locked reference. Soft-material
rupture is absent. Neither recovery nor lack of solver warnings establishes
hardware survival, general soft-frame superiority or flight-model validation.

Supplied bench rheology is conditional on the identified release protocol;
pitch uses a fixed beta_up memory/damping preset. Body/arm inertia estimates,
neutral-hover preload, contact and controller choices remain assumptions.
K/C values are imported unchanged and are not tuned to force a recovery result.

## Historical outputs

Existing `outputs/wall_collision_baseline/` files from the old model are
historical. Importing CAD does not replace them. The supplied reference CAD
checks are retained in `outputs/cad_checks/`.

Historical behavior demos explicitly import `legacy_collision_study.py` and
legacy JSON configurations. They do not silently reuse CAD parameters.
