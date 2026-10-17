"""Legacy illustrative mechanics; independent of the frozen CAD model.

Isolated wall-contact behavior preview inspired by reference seconds 10–19.

Does not alter collision_study defaults, soft artwork or narrow-passage flight.
This is qualitative tuning, not an identified reconstruction or an RL policy.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess

import mujoco
import numpy as np

from legacy_collision_study import Trial, load_config, write_results, camera, tilt_deg
from soft_visual import add_soft_ligaments


class BehaviorTrial(Trial):
    def __init__(self, speed=2., kind='soft', model_transform=None):
        cfg = load_config()
        cfg.update(post_impact_s=5., motor_time_constant_s=.04)
        super().__init__(kind, speed, cfg, fracture=False, model_transform=model_transform)
        self.control_interval = round(.002/self.model.opt.timestep)
        self.control_tick = 0
        self.initial_yaw = math.radians(12)
        self.data.qpos[3:7] = [math.cos(self.initial_yaw/2), 0, 0, math.sin(self.initial_yaw/2)]
        mujoco.mj_forward(self.model, self.data)
        self.command = self.thrust.copy()
        self.hover_target = self.recovery_target.copy()

    def controller(self, settling=False):
        m, d = self.model, self.data
        if self.control_tick % self.control_interval == 0:
            R = d.xmat[self.core].reshape(3, 3)
            pos, vel = d.qpos[:3], d.qvel[:3]
            acc = np.zeros(3)
            if settling:
                acc[:2] = -4*pos[:2]-3*vel[:2]
            elif self.impact_time is None:
                acc[0] = 2*(self.speed-vel[0])
                acc[1] = -4*pos[1]-3*vel[1]
            else:
                acc[:2] = 3*(self.recovery_target[:2]-pos[:2])-2.8*vel[:2]
            acc[:2] = np.clip(acc[:2], -3, 3)
            acc[2] = 4*(self.cfg['altitude_m']-pos[2])-2.8*vel[2]
            mass = m.body_mass.sum()
            com = (d.xipos*m.body_mass[:, None]).sum(axis=0)/mass
            force = mass*(acc+np.array([0., 0., 9.81]))
            z = force/max(np.linalg.norm(force), 1e-9)
            heading = np.array([math.cos(self.initial_yaw), math.sin(self.initial_yaw), 0.])
            y = np.cross(z, heading)
            y /= max(np.linalg.norm(y), 1e-9)
            desired = np.column_stack((np.cross(y, z), y, z))
            skew = .5*(desired.T@R-R.T@desired)
            error = np.array([skew[2, 1], skew[0, 2], skew[1, 0]])
            torque = -np.array([1.1, 1.1, .3])*error-np.array([.18, .18, .075])*d.qvel[3:6]
            allocation = np.zeros((4, 4))
            for i, site in enumerate(self.sites):
                direction = d.site_xmat[site].reshape(3, 3)[:, 2]
                moment = np.cross(d.site_xpos[site]-com, direction)+m.actuator_gear[i, 5]*direction
                allocation[0, i] = (R.T@direction)[2]
                allocation[1:, i] = R.T@moment
            self.command = np.clip(np.linalg.lstsq(allocation,
                np.r_[max(0., force@R[:, 2]), torque], rcond=None)[0], 0, self.cfg['max_motor_thrust_n'])
        self.control_tick += 1
        alpha = 1-math.exp(-m.opt.timestep/self.cfg['motor_time_constant_s'])
        self.thrust += alpha*(self.command-self.thrust)
        d.ctrl[:] = self.thrust

    def run(self):
        # Capture a genuine pre-approach hover location. No pose or velocity
        # changes are made after approach starts or at/after impact.
        for _ in range(round(self.cfg['settle_s']/self.model.opt.timestep)):
            self.step(settling=True)
        self.nominal_bend = self.data.qpos[self.bend_indices].copy() if self.bend_indices else np.zeros(8)
        leading = max(self.data.site_xpos[s, 0] for s in self.sites)+self.cfg['guard_radius_m']+.003
        from legacy_collision_study import WALL_FRONT
        shift = WALL_FRONT-leading-self.speed*self.cfg['approach_s']
        for j in range(self.model.njnt):
            if self.model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
                self.data.qpos[self.model.jnt_qposadr[j]] += shift
        self.recovery_target = self.data.qpos[:3].copy()
        self.hover_target = self.recovery_target.copy()
        self.data.time = 0
        # A longer acceleration interval than the old impulse-initialised
        # study: drive toward the wall entirely with motor forces.
        mujoco.mj_forward(self.model, self.data)
        for _ in range(round(7/self.model.opt.timestep)):
            self.step()
        return self

    def summary(self):
        result = super().summary()
        heights = np.array([q[2] for q, *_ in self.history])
        result.update(behavior_profile='wall-preview-only',
            altitude_drop_m=float(max(0., self.hover_target[2]-heights.min())),
            final_position_error_m=float(np.linalg.norm(self.data.qpos[:3]-self.hover_target)),
            final_speed_m_s=float(np.linalg.norm(self.data.qvel[:3])),
            hover_target_m=self.hover_target.tolist(),
            simulation_warning_count=int(sum(w.number for w in self.data.warning)))
        return result


def render_preview(trials, output, fps=30, titles=None):
    """Compare qualitative profiles on a time axis aligned at first contact."""
    from imageio_ffmpeg import get_ffmpeg_exe
    from PIL import Image, ImageDraw, ImageFont
    ffmpeg = get_ffmpeg_exe()
    width, height = 800, 600
    output.parent.mkdir(parents=True, exist_ok=True)
    renderers = []
    states = [(t.data.qpos.copy(), t.data.qvel.copy()) for t in trials]
    font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 21)
    small = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 17)
    times = np.r_[np.arange(-.6, -.04, 1/fps),
                  np.arange(-.04, .26, .2/fps), np.arange(.26, 4.5, 1/fps)]
    process = None
    try:
        for t in trials:
            t.model.vis.global_.offwidth = width
            t.model.vis.global_.offheight = height
            renderers.append(mujoco.Renderer(t.model, height=height, width=width))
        process = subprocess.Popen([ffmpeg, '-hide_banner', '-loglevel', 'error', '-n',
            '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{2*width}x{height}',
            '-r', str(fps), '-i', '-', '-an', '-c:v', 'libx264', '-preset', 'fast',
            '-crf', '18', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(output)],
            stdin=subprocess.PIPE)
        for relative in times:
            panels = []
            for i, (t, renderer) in enumerate(zip(trials, renderers)):
                physical_time = max(0., relative+t.impact_time)
                index = min(len(t.history)-1, round(physical_time/t.model.opt.timestep))
                q, v, eq, _ = t.history[index]
                t.data.qpos[:], t.data.qvel[:], t.data.eq_active[:] = q, v, eq
                if hasattr(t, 'apply_replay_visuals'):
                    t.apply_replay_visuals(physical_time)
                mujoco.mj_forward(t.model, t.data)
                view = camera()
                view.lookat[:] = [-.35, 0, 1.05]
                view.distance = 3.4
                renderer.update_scene(t.data, camera=view)
                if t.kind == 'soft':
                    add_soft_ligaments(renderer.scene, t.model, t.data)
                panel = Image.fromarray(renderer.render())
                view = camera(close=True)
                view.lookat[:] = t.data.qpos[:3]
                view.distance = 1.1
                renderer.update_scene(t.data, camera=view)
                if t.kind == 'soft':
                    add_soft_ligaments(renderer.scene, t.model, t.data)
                close = Image.fromarray(renderer.render()).resize((320, 240))
                panel.paste(close, (width-320, height-240))
                draw = ImageDraw.Draw(panel)
                draw.rectangle((0, 0, width, 112), fill=(15, 21, 30))
                title = (titles[i] if titles is not None else
                         ('HoLoArm / BEHAVIOR PREVIEW' if i == 0 else 'HoLoArm / ORIGINAL WALL PROFILE'))
                draw.text((12, 8), title, font=font, fill=(80, 200, 255) if i == 0 else (235, 235, 235))
                drop = max(0., t.cfg['altitude_m']-q[2])*100
                bend = math.degrees(float(np.max(np.abs(q[t.bend_indices]-t.nominal_bend)))) if t.bend_indices else 0.
                draw.text((12, 37), f'Impact {t.actual_impact_speed:.2f} m/s | tilt {tilt_deg(t.data,t.core):.1f} deg'
                          f' | height loss {drop:.1f} cm', font=small, fill='white')
                state = 'APPROACH' if relative < 0 else 'CONTACT / SELF-RECOVERY'
                if t.recovery_time is not None and relative > t.recovery_time+.3:
                    state = 'RECOVERED: hover'
                if t.ground_time is not None and physical_time >= t.ground_time:
                    state = 'GROUND CONTACT: failed recovery'
                if hasattr(t, 'replay_status') and t.replay_status(physical_time):
                    state = t.replay_status(physical_time)
                    if t.ground_time is not None and physical_time >= t.ground_time:
                        state += ' | GROUND CONTACT'
                draw.text((12, 63), f'{state} | arm deformation {bend:.1f} deg', font=small, fill='white')
                playback = '0.2x slow motion' if -.04 <= relative < .26 else '1x'
                draw.text((12, 86), f'Time from contact {relative:+.2f}s | {playback}', font=small, fill=(255, 220, 90))
                panels.append(np.asarray(panel))
            process.stdin.write(np.concatenate(panels, axis=1).tobytes())
    finally:
        if process is not None:
            process.stdin.close()
            status = process.wait()
        for renderer in renderers:
            renderer.close()
        for t, (q, v) in zip(trials, states):
            t.data.qpos[:], t.data.qvel[:] = q, v
            if hasattr(t, 'apply_replay_visuals'):
                t.apply_replay_visuals(t.data.time)
            mujoco.mj_forward(t.model, t.data)
    if status:
        raise RuntimeError(f'ffmpeg failed: {status}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--speeds', type=float, nargs='+', default=[2.5])
    parser.add_argument('--output-dir', type=Path,
        default=Path(__file__).resolve().parents[1]/'outputs'/'wall_behavior_preview')
    parser.add_argument('--no-video', action='store_true')
    args = parser.parse_args()
    if any(not math.isfinite(s) or not 1.5 <= s <= 8 for s in args.speeds):
        parser.error('Speeds must be between 1.5 and 8 m/s')
    if len(set(args.speeds)) != len(args.speeds):
        parser.error('Speeds must be unique')
    if args.output_dir.exists():
        parser.error('Choose a new output directory; existing results are preserved')
    trials = []
    for speed in args.speeds:
        trial = BehaviorTrial(speed).run()
        original_cfg = load_config()
        original_cfg['post_impact_s'] = 5.
        baseline = Trial('soft', speed, original_cfg).run()
        trials.extend((trial, baseline))
        print(json.dumps(trial.summary(), indent=2), flush=True)
        if not args.no_video:
            # Left: tuned behavior. Right: original soft controller/profile.
            render_preview([trial, baseline], args.output_dir/f'behavior_vs_original_{speed:g}ms.mp4')
    # Distinguish files even though both panels intentionally use the soft model.
    write_results(trials[::2], args.output_dir/'behavior', trials[0].cfg)
    write_results(trials[1::2], args.output_dir/'original', load_config())
    (args.output_dir/'README.md').write_text(
        '# Wall behavior preview — isolated from narrow-passage flight\n\n'
        'Reference: https://www.youtube.com/watch?v=hmfgFP9XoHQ , seconds 10–19.\n'
        'Left panel: behavior preview; right panel: original soft wall-study profile.\n'
        'Both panels are SOFT HoLoArm (not a rigid comparison).\n'
        'Qualitative tuning only: 12-degree oblique approach, 500 Hz controller, '
        '40 ms motor response, stronger attitude damping and gentler altitude/position feedback. '
        'Preview accelerates with thrust rather than injecting approach velocity.\n'
        'The preview returns to its saved pre-approach hover location. No scripted '
        'dip, pose animation, forced motor shutdown or post-impact velocity injection.\n'
        'Joint/contact parameters and narrow-passage controller are unchanged.\n'
        'Requested speeds are command targets; inspect actual_impact_speed_m_s. '
        'This is NOT a controlled A/B experiment: initial yaw, approach, measured '
        'impact speeds and hover targets differ; contact times are aligned in video.\n'
        'The default 2.5 m/s command preview was checked for airborne recovery. '
        'This profile is NOT validated across 1.5–8 m/s; faster runs can fail recovery.\n')


if __name__ == '__main__':
    main()
