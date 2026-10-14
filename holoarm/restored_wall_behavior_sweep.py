"""Original no-footer wall scenario + physical oblique-contact lean + arm fracture.

Preserves the original controller, mass, joints, motor response, initial-speed
protocol and rigid arm-failure thresholds. A configurable wall pitch (default
2 deg) is applied identically to both models. Rigid physical guards are visible.
This illustrative contact angle and fracture thresholds are not hardware fits.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time

import numpy as np

from collision_study import Trial, combine_videos, render_video, write_results
from speed_sweep_config import SPEEDS
from rigid_wall_behavior_preview import add_visible_rigid_guards

ROOT = Path(__file__).resolve().parents[1]
# Keep the reference configuration independent of disposable rendered outputs.
BASELINE = ROOT/'holoarm'/'models'/'wall_collision_baseline.json'


class RestoredTrial(Trial):
    def __init__(self, kind, speed, cfg, wall_pitch_deg=2., recovery_x=-.25):
        self.wall_pitch_deg = wall_pitch_deg
        def transform(root):
            wall = root.find("worldbody/geom[@name='impact_wall']")
            angle = math.radians(wall_pitch_deg)
            wall.set('euler', f'0 {angle} 0')
            # Rotate the existing contact stripe with its wall so it doesn't
            # float above the tilted contact face. It remains non-colliding.
            centre = np.fromstring(wall.get('pos'), sep=' ')
            rotation = np.array([[math.cos(angle), 0, math.sin(angle)],
                                 [0, 1, 0], [-math.sin(angle), 0, math.cos(angle)]])
            for geom in root.find('worldbody').findall('geom'):
                if geom.get('name') is None and geom.get('type') == 'box':
                    pos = np.fromstring(geom.get('pos'), sep=' ')
                    geom.set('pos', ' '.join(map(str, centre+rotation@(pos-centre))))
                    geom.set('euler', f'0 {angle} 0')
            if kind == 'rigid':
                add_visible_rigid_guards(root)
        super().__init__(kind, speed, cfg, fracture=True, model_transform=transform)
        self.recovery_target[0] = recovery_x

    def summary(self):
        result = super().summary()
        result.update(wall_pitch_deg=self.wall_pitch_deg,
            recovery_target_m=self.recovery_target.tolist(),
            altitude_drop_m=float(max(0.,self.cfg['altitude_m']-min(q[2] for q,*_ in self.history))),
            simulation_warning_count=int(sum(w.number for w in self.data.warning)))
        if self.impact_time is not None:
            index = min(len(self.history)-1, max(0,round(self.impact_time/self.model.opt.timestep)-1))
            impact_x = self.history[index][0][0]
            result['post_impact_retreat_m'] = float(max(0.,impact_x-min(q[0] for q,*_ in self.history[index:])))
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path,
        default=ROOT/'outputs'/'wall_collision_restored_lean_arm_fracture')
    parser.add_argument('--no-video', action='store_true')
    parser.add_argument('--resume', action='store_true', help='Resume this output directory; preserve completed clips')
    parser.add_argument('--wall-pitch-deg', type=float, default=2.,
        help='Illustrative oblique-contact angle; 5 degrees produces stronger lean')
    parser.add_argument('--slow-motion-window', type=float, default=.26,
        help='Seconds after contact replayed at .15x')
    parser.add_argument('--recovery-x', type=float, default=-.25,
        help='Hover recovery X position; -.45 extends the original retreat by .20 m')
    args = parser.parse_args()
    if args.output_dir.exists() and not args.resume:
        parser.error('Choose a new output directory; existing videos are preserved')
    if not math.isfinite(args.wall_pitch_deg) or not 0 <= args.wall_pitch_deg <= 8:
        parser.error('--wall-pitch-deg must be in 0..8')
    if not math.isfinite(args.slow_motion_window) or not .1 <= args.slow_motion_window <= 1:
        parser.error('--slow-motion-window must be in .1..1')
    if not math.isfinite(args.recovery_x) or not -2 <= args.recovery_x <= 0:
        parser.error('--recovery-x must be in -2..0 m')
    cfg = json.loads(BASELINE.read_text())['config']
    args.output_dir.mkdir(parents=True, exist_ok=True)
    protocol = dict(wall_pitch_deg=args.wall_pitch_deg, slow_motion_window_s=args.slow_motion_window,
                    recovery_x=args.recovery_x, baseline=str(BASELINE))
    protocol_path = args.output_dir/'protocol.json'
    if protocol_path.exists():
        previous = json.loads(protocol_path.read_text())
        previous.setdefault('recovery_x', -.25)
        if previous != protocol:
            parser.error('Existing output protocol differs; choose a new directory')
    if not protocol_path.exists():
        protocol_path.write_text(json.dumps(protocol, indent=2))
    all_trials, clips = [], []
    for speed in SPEEDS:
        trials = [RestoredTrial(kind, speed, cfg, args.wall_pitch_deg, args.recovery_x).run()
                  for kind in ('soft','rigid')]
        for trial in trials:
            row = trial.summary()
            if row['simulation_warning_count']:
                raise RuntimeError(f'Simulation warning at {speed} {trial.kind}')
            if abs(row['actual_impact_speed_m_s']-speed) > .02*speed:
                raise RuntimeError(f'Impact speed mismatch at {speed} {trial.kind}')
            if trial.kind == 'soft' and (not row['recovered'] or row['ground_contact_time_s'] is not None):
                raise RuntimeError(f'Soft airborne recovery failed at {speed}; no success will be fabricated')
            print(f"{speed:g} m/s {trial.kind}: measured={row['actual_impact_speed_m_s']:.3f}, "
                  f"tilt={row['peak_tilt_deg']:.1f}, drop={row['altitude_drop_m']*100:.1f} cm, "
                  f"arms_broken={row['broken_arms']}, recovered={row['recovered']}", flush=True)
        all_trials.extend(trials)
        path = args.output_dir/f'comparison_{speed:g}ms.mp4'
        if not args.no_video:
            complete = False
            if args.resume and path.exists():
                probe = subprocess.run(['ffprobe','-v','error','-show_entries','format=duration',
                                        '-of','json',str(path)], capture_output=True, text=True)
                complete = probe.returncode == 0 and float(json.loads(probe.stdout)['format']['duration']) > 4
                if not complete:
                    path.rename(path.with_name(f'{path.stem}.incomplete_{time.time_ns()}.mp4'))
            if not complete:
                render_video(trials, path, show_height_loss=True,
                             slow_motion_window_s=args.slow_motion_window)
            clips.append(path)
    summaries = write_results(all_trials, args.output_dir, cfg)
    if not any(r['kind']=='rigid' and r['broken_arms'] for r in summaries):
        raise RuntimeError('No rigid arm fracture in sweep; inspect the scenario')
    (args.output_dir/'README.md').write_text(
        '# Restored no-footer scenario: soft lean + rigid arm fracture\n\n'
        f'Baseline configuration: {BASELINE}\n\n'
        'Original mass/joints/contact solver/controller/motor response and initial '
        'velocity protocol are restored. This is NOT BehaviorTrial or the sampled '
        'feedback/exposed-propeller damage scenario.\n\n'
        f'Only physical contact change: wall pitched {args.wall_pitch_deg:g} degrees, identically for '
        'both models, producing asymmetric/off-axis loads and a natural lean. '
        'This small contact-angle assumption is illustrative, not a measurement '
        'from the reference video. No scripted dip, pose animation or post-contact '
        'velocity/rotation injection. The original feedback law is retained. '
        f'Hover recovery target: ({args.recovery_x:g},0,1.2) m. '
        f'Retreat extension relative to the original -.25 m target: {-.25-args.recovery_x:+.2f} m. '
        'This increases controller-driven retreat distance, NOT passive collision '
        'restitution or a post-impact reverse-velocity injection.\n\n'
        'Rigid has visible physical guard rings, massless artwork spokes, and '
        'the original load-triggered weld release: arm contact moment >110 Nm '
        'or force >2000 N. A detached arm keeps its mass/velocity and loses motor '
        'thrust. No exposed-propeller failure model is used. These fracture '
        'thresholds are assumptions, not measured Skydio X2 material limits.\n\n'
        '12 impact speeds: 2–7 m/s every .5 m/s, plus 7.6 m/s. Measured speeds '
        'are checked within 2% of each requested impact speed. Every soft case '
        'must recover without floor contact; otherwise generation stops rather '
        'than fabricating a successful trajectory. Fracture outcomes remain '
        'determined by contact loads, not hardcoded speed or animation.\n\n'
        'Video: 1280x540,30fps; identical physical time across panels, explicit '
        '.15x slow-motion impact, no bottom disclaimer strip.\n\n'
        'The original videos and narrow-passage viewer/controller are unchanged.\n')
    if clips:
        if not (args.output_dir/'speed_sweep_comparison.mp4').exists():
            print(combine_videos(clips, args.output_dir), flush=True)


if __name__ == '__main__':
    main()
