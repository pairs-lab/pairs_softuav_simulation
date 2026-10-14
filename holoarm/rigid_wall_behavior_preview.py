"""Research-informed rigid wall scenarios; X2-inspired appearance, NOT X2 validation.

Guarded: intact rigid frame, delayed/sampled feedback, possible rebound/recovery.
Exposed: rotating swept propeller envelopes touch the wall; assumed impulse-
triggered rotor damage causes thrust loss, not an animated crash or forced flip.
All modifications are instance-local and leave passage/soft profiles unchanged.
"""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from collision_study import ARMS, Trial, load_config, write_results
from wall_behavior_preview import BehaviorTrial, render_preview


def add_visible_rigid_guards(root):
    """Display the physical guard rings; support artwork adds no mass/contact."""
    for geom in root.findall('.//geom'):
        if '_guard_' in geom.get('name', ''):
            geom.set('rgba', '.38 .44 .50 1')
    for arm in ARMS:
        motor = root.find(f".//body[@name='{arm}_motor']")
        for spoke in range(3):
            angle = 2*math.pi*spoke/3
            c, s = math.cos(angle), math.sin(angle)
            ET.SubElement(motor, 'geom', name=f'rigid_guard_{arm}_support_{spoke}',
                type='capsule', fromto=f'{.023*c} {.023*s} 0 {.102*c} {.102*s} 0',
                size='.0022', mass='0', contype='0', conaffinity='0',
                rgba='.52 .57 .62 1', group='2')


class RigidBehaviorTrial(BehaviorTrial):
    def __init__(self, speed=2.5, scenario='guarded', rotor_damage_impulse=.3):
        if scenario not in ('guarded', 'exposed'):
            raise ValueError('scenario must be guarded or exposed')
        if not np.isfinite(rotor_damage_impulse) or rotor_damage_impulse <= 0:
            raise ValueError('rotor_damage_impulse must be positive and finite')
        def exposed_contacts(root):
            # MuJoCo compiles contact candidates/BVH at load time. Merely
            # changing contype from 0 to 2 later won't add visual-only geoms.
            for geom in root.findall('.//geom'):
                name = geom.get('name', '')
                if '_guard_' in name:
                    geom.set('contype', '0')
                    geom.set('conaffinity', '0')
                elif name in {f'rigid_x2_{a}_prop_blur' for a in ARMS}:
                    geom.set('contype', '2')
                    geom.set('conaffinity', '1')
        super().__init__(speed, kind='rigid',
                         model_transform=exposed_contacts if scenario == 'exposed' else add_visible_rigid_guards)
        self.scenario = scenario
        self.sensor = mujoco.MjData(self.model)
        self.sensor_sample_steps = round((1/150)/self.model.opt.timestep)
        self.sensor_delay_steps = round(.012/self.model.opt.timestep)
        self.sensor_history = deque(maxlen=self.sensor_delay_steps+1)
        self.sensor_freeze_until = -1.
        self.sensor_clock = 0
        self.rotor_impulse = np.zeros(4)
        self.rotor_damage = np.zeros(4, dtype=bool)
        self.rotor_damage_times = np.full(4, np.inf)
        self.rotor_impulse_threshold = rotor_damage_impulse  # N s; uncalibrated assumption
        self.disarmed = False
        self.rotor_geoms = [self.model.geom(f'rigid_x2_{a}_prop_blur').id for a in ARMS]
        self.blade_geoms = [self.model.geom(f'rigid_x2_{a}_prop_blade').id for a in ARMS]
        self.original_rotor_rgba = self.model.geom_rgba[self.rotor_geoms].copy()
        self.original_blade_size = self.model.geom_size[self.blade_geoms].copy()
        mujoco.mj_forward(self.model, self.data)

    def controller(self, settling=False):
        actual = self.data
        self.sensor_history.append((actual.qpos.copy(), actual.qvel.copy(), actual.time))
        if (self.sensor_clock % self.sensor_sample_steps == 0 and
                actual.time >= self.sensor_freeze_until):
            q, v, stamp = self.sensor_history[0]
            self.sensor.qpos[:], self.sensor.qvel[:], self.sensor.time = q, v, stamp
            mujoco.mj_forward(self.model, self.sensor)
        self.sensor_clock += 1
        # Evaluate the exact same feedback law using delayed observations,
        # not ground truth. The true state is never changed by sensor faults.
        self.data = self.sensor
        try:
            super().controller(settling)
        finally:
            self.data = actual
        self.thrust[self.rotor_damage] = 0
        actual.ctrl[:] = 0 if self.disarmed else self.thrust

    def step(self, settling=False):
        super().step(settling)
        if settling:
            return
        if self.scenario == 'exposed':
            newly_failed = (~self.rotor_damage) & (self.rotor_impulse >= self.rotor_impulse_threshold)
            self.rotor_damage[newly_failed] = True
            self.rotor_damage_times[newly_failed] = self.data.time
            self.thrust[self.rotor_damage] = 0
            self.data.ctrl[self.rotor_damage] = 0
            # Broken rotors can no longer present a full spinning disc.
            self.model.geom_contype[np.array(self.rotor_geoms)[newly_failed]] = 0
            self.model.geom_conaffinity[np.array(self.rotor_geoms)[newly_failed]] = 0
            if self.rotor_damage.any():
                self.recovery_time = self.recovery_start = None
                # Scenario-specific impact-to-ground safety disarm prevents
                # an unrealistically powered relaunch after the crash.
                if self.ground_time is not None:
                    self.disarmed = True
                    self.data.ctrl[:] = self.thrust[:] = 0
            self.apply_replay_visuals(self.data.time)

    def contacts(self):
        result = super().contacts()
        # Sample the solved shock before the parent calls mj_forward.
        # 16 g / 40 ms are sensitivity assumptions, not X2 specifications.
        if np.linalg.norm(self.data.qacc[:3]) > 16*9.81:
            self.sensor_freeze_until = max(self.sensor_freeze_until, self.data.time+.04)
        if self.scenario == 'exposed':
            force = np.zeros(6)
            for i in range(self.data.ncon):
                c = self.data.contact[i]
                if self.wall not in (c.geom1, c.geom2):
                    continue
                geom = c.geom2 if c.geom1 == self.wall else c.geom1
                if geom in self.rotor_geoms:
                    mujoco.mj_contactForce(self.model, self.data, i, force)
                    self.rotor_impulse[self.rotor_geoms.index(geom)] += abs(force[0])*self.model.opt.timestep
        return result

    def run(self):
        result = super().run()
        if self.rotor_damage.any():
            self.recovery_time = self.recovery_start = None
        return result

    def apply_replay_visuals(self, time):
        for i, (disc, blade) in enumerate(zip(self.rotor_geoms, self.blade_geoms)):
            failed = time >= self.rotor_damage_times[i]
            self.model.geom_rgba[disc] = self.original_rotor_rgba[i]
            self.model.geom_rgba[disc, 3] = 0 if failed else self.original_rotor_rgba[i, 3]
            self.model.geom_size[blade] = self.original_blade_size[i]
            if failed:
                self.model.geom_size[blade, 0] *= .3

    def replay_status(self, time):
        failed = int(np.count_nonzero(time >= self.rotor_damage_times))
        return f'ASSUMED PROP DAMAGE: {failed} rotor(s) lost' if failed else None

    def summary(self):
        result = super().summary()
        result.update(scenario=self.scenario, x2_identified_model=False,
            damaged_rotors=int(self.rotor_damage.sum()),
            rotor_damage_times_s=[float(v) if np.isfinite(v) else None for v in self.rotor_damage_times],
            rotor_contact_impulse_ns=self.rotor_impulse.tolist(),
            rotor_damage_impulse_threshold_ns=self.rotor_impulse_threshold,
            feedback_sample_hz=1/(self.sensor_sample_steps*self.model.opt.timestep),
            feedback_delay_s=self.sensor_delay_steps*self.model.opt.timestep,
            assumed_shock_sample_hold_s=.04, ground_safety_disarmed=self.disarmed)
        if self.rotor_damage.any():
            result['recovered'] = False
            result['recovery_time_s'] = None
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--speed', type=float, default=2.5)
    parser.add_argument('--output-dir', type=Path,
        default=Path(__file__).resolve().parents[1]/'outputs'/'rigid_wall_behavior_preview')
    parser.add_argument('--no-video', action='store_true')
    parser.add_argument('--rotor-damage-impulse', type=float, default=.3,
        help='Assumed per-rotor normal contact impulse threshold in N s; NOT X2 calibration')
    args = parser.parse_args()
    if not np.isfinite(args.speed) or not 1.5 <= args.speed <= 8:
        parser.error('--speed must be between 1.5 and 8 m/s')
    if args.output_dir.exists():
        parser.error('Choose a new output directory; existing outputs are preserved')
    if not np.isfinite(args.rotor_damage_impulse) or args.rotor_damage_impulse <= 0:
        parser.error('--rotor-damage-impulse must be positive and finite')
    cfg = load_config()
    cfg['post_impact_s'] = 5
    old = Trial('rigid', args.speed, cfg).run()
    guarded = RigidBehaviorTrial(args.speed, 'guarded').run()
    exposed = RigidBehaviorTrial(args.speed, 'exposed', args.rotor_damage_impulse).run()
    soft = BehaviorTrial(args.speed).run()
    for name, trial in [('old', old), ('guarded', guarded), ('exposed', exposed), ('soft', soft)]:
        print(name, json.dumps(trial.summary()), flush=True)
        write_results([trial], args.output_dir/name, trial.cfg)
    if not args.no_video:
        render_preview([guarded, old], args.output_dir/'rigid_intact_vs_old.mp4', titles=[
            'RIGID / intact + sampled feedback', 'RIGID / old ideal-state profile'])
        render_preview([soft, exposed], args.output_dir/'soft_vs_rigid_prop_strike.mp4', titles=[
            'HoLoArm / accepted soft behavior', 'RIGID X2-style / exposed prop strike'])
    (args.output_dir/'README.md').write_text(Path(__file__).with_name('RIGID_WALL_BEHAVIOR.md').read_text())


if __name__ == '__main__':
    main()
