# Rigid wall behavior: evidence and isolated scenarios

These historical scenarios use `legacy_collision_study.py` and the original
illustrative model/configuration. They do not use the current HoLoArm CAD or
its supplied material parameters. See [CAD model](CAD_MODEL.md) for the current
viewer, wall-impact study and ROS adapter.

## Latest illustrative wall-recovery profile

`outputs/wall_collision_comparison/` keeps the approved
soft compression/bank trajectory, visible rigid guards, level frontal approach,
and load-triggered arm fracture. It uses the selected 12 speeds (2–7 m/s in
0.5 m/s increments plus 7.6 m/s). Narrow-passage control is unchanged.

The HoLoArm video segment from 1:50 to approximately 3:12 and paper section IV-C
show hover disturbances applied to arms: stronger rigid roll/pitch oscillations
with recovery, versus more stable soft response. They do NOT provide a measured
2–5 m/s wall-impact curve. The new scenario deliberately selects temporary
post-contact observation delay of 50 ms for 2.8 s, returning to the existing
12 ms delay. This same assumption applies at every speed; no initial angle,
state kicks, forced torque, or speed-conditioned recovery animation is added.

These delay values/durations are NOT identified hardware parameters or a claim
that the reference IMU behaves this way. The controllers are not matched to
soft, and the result does not isolate a structural-compliance advantage.
The criteria checked at 2–5 m/s are larger rigid peak tilt, recovery at least
0.5 s later than soft, and successful recovery without fracture/ground contact.
This is a qualitative video scenario, not a validated hardware prediction.

```bash
MUJOCO_GL=egl python3 holoarm/rigid_offset_behavior_sweep.py --rigid-profile transient --output-dir outputs/new_rigid_recovery
```

This is a qualitative rigid-quadrotor illustration with X2-inspired artwork.
It is **not** an identified Skydio X2 model, crash certification or prediction
of Skydio's proprietary recovery/obstacle-avoidance behavior. No verified X2
wall-impact dataset with known speed, damage and controller settings was found.
Avoidance is not simulated: these are deliberate physical-contact scenarios.

## References

- Patnaik et al., *Collision Recovery Control of a Foldable Quadrotor*:
  https://arxiv.org/html/2105.12273v1 — Figures 8/9 and section V-B compare
  1–2.5 m/s wall impacts. The rigid platform exhibits rebound, overshoot,
  oscillatory/unpredictable recovery and sometimes a second wall strike;
  the authors discuss IMU sample loss during brief impacts.
  Experiment video: https://www.youtube.com/watch?v=pGe2hkaTKSE .
- HoLoArm, section IV-C: https://arxiv.org/html/2605.25790v1 — an intact rigid
  drone can recover from roll/pitch disturbances; yaw disturbances caused a
  crash in their tests. These are not X2 tests. Video:
  https://www.youtube.com/watch?v=hmfgFP9XoHQ . Drop-test arm fracture is a
  separate experiment and is not evidence of a wall-fracture threshold.
- Morphy project and video, context for compliant impact recovery, NOT X2
  evidence: https://www.autonomousrobotslab.com/collision-tolerant-aerial-robots.html
  and https://www.youtube.com/watch?v=C6l7Vklbc9k .
- Rigid propeller damage is a separate failure mechanism documented in
  *Collision Detection and Reaction for Quadrotors Using Encoder-Integrated
  Tombo Propellers*, https://doi.org/10.1109/ACCESS.2025.3572243 . Experiment
  video: https://www.youtube.com/watch?v=czliIvss5_I . No thresholds below
  are fitted to those experiments. The linked video was unavailable when
  checked on 2026-10-14; it was not visually inspected. The ASU foldable/rigid
  comparison and Morphy videos were downloaded and inspected frame by frame;
  Morphy is compliant-only context, not an additional rigid/X2 validation.

## What was overly idealized in the old video?

`rigid_visual.py` hides the original HoLoArm physical guards but retains their
contacts. Thus the displayed X2-style unguarded rotors were actually protected
by invisible rings. The old controller also reads exact MuJoCo ground-truth
states and immediately selects a retreat point after detecting contact.
It has no sensor clipping/dropout or propeller failure model. A mild guarded
impact really can recover rapidly; rapid recovery alone is not proof of error.

## New cases

1. **Intact guarded rigid frame**: the protected comparison envelope remains.
   Current guarded previews display the four physical rings in grey, with
   massless support-spoke artwork. Ring contact geometry and inertia are
   unchanged; these are rigid rings, not compliant HoLoArm joints.
   The preview's attitude/position law runs on sampled/delayed feedback
   (nominal 150 Hz sampling, 12 ms delay). After acceleration above an assumed
   16 g, the last valid observation is held for an assumed 40 ms. These are
   illustrative finite-feedback assumptions, NOT measured X2 specifications.
   Rotor thrust is continuous and the rigid body can recover normally.
2. **Exposed X2-style propeller strike**: all 80 hidden ring contacts are disabled
   on this trial only. The four 104 mm-radius discs at the actual visual rotor
   height become swept-envelope contact proxies (not blade-resolved rotor
   dynamics). At a per-rotor accumulated normal wall-contact impulse of an
   assumed **0.3 N s**, that rotor loses all thrust, its blade is shown shortened,
   and its swept-disc collision is removed. This is an explicit damage scenario,
   NOT a validated material failure law or assertion that every graze breaks a
   rotor. Healthy motors still respond through the same four-motor control law;
   the attitude/height loss is produced by physics. After a damaged vehicle
   contacts the ground, a scenario-specific safety disarm prevents relaunch.
   No pose, velocity or forced downward impulse is injected after contact.

Frame fracture is disabled in these preview cases to isolate rotor-contact
failure; the old survey's assumed arm-fracture model is left unchanged.
Mass and inertias are preserved and matched for the illustration, NOT X2 mass.
Sensor and damage constants, scenario name, measured impact speed, rotor
impulses, damage times and recovery metrics are exported to results JSON/CSV.

## Videos and interpretation

- `rigid_intact_vs_old.mp4`: left sampled-feedback intact rigid preview;
  right the original ideal-state guarded rigid model.
- `soft_vs_rigid_prop_strike.mp4`: left the user-approved soft behavior,
  right the exposed-rotor damage scenario with X2-inspired artwork.

Contact times are aligned for viewing; actual impact velocities are printed.
This is NOT a controlled same-speed controller-performance comparison because
the old approach injects initial velocity and uses a different hover target.
The soft/rigid contact geometries differ deliberately in the exposed case.
These previews do not validate the full 1.5–8 m/s range. Undamaged rigid recovery
and exposed-rotor crash are both physically possible outcomes, not universal rules.

## Run

```bash
python3 holoarm/rigid_wall_behavior_preview.py
python3 holoarm/rigid_wall_behavior_preview.py --no-video --output-dir /tmp/new_rigid_check
python3 -m unittest discover -s holoarm -p 'test_*.py'
```

Existing videos, global mechanics, the approved soft trajectory and narrow-
passage keyboard controller are not changed. All new physics/sensor/damage
parameters are applied only to instances in `rigid_wall_behavior_preview.py`.
