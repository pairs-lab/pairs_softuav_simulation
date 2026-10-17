"""Legacy illustrative mechanics; independent of the frozen CAD model.

Frontal passive arm compression FIRST, controller-driven recovery bank SECOND.

The wall is vertical and all contact/elastic/fracture parameters match the old
no-footer study. The explicit bank maneuver is an illustrative flight-control
choice, not an identified passive HoLoArm response or a trained RL policy.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time

import numpy as np

from speed_sweep_config import SPEEDS
from legacy_collision_study import render_video, write_results, combine_videos
from restored_wall_behavior_sweep import RestoredTrial, BASELINE, ROOT


class CompressionLeanTrial(RestoredTrial):
    def __init__(self, kind, speed, cfg, bank_gain=1.):
        cfg = dict(cfg)
        cfg['post_impact_s'] = 7.
        super().__init__(kind, speed, cfg, wall_pitch_deg=0, recovery_x=-.45)
        self.bank_start = None
        self.bank_duration = .9
        speed_gain = 1+.35*float(np.clip((speed-3)/5,0,1))
        self.bank_deg = float(np.clip(12+5*speed, 20, 55))*bank_gain*speed_gain

    def step(self, settling=False):
        super().step(settling)
        if (not settling and self.kind == 'soft' and self.bank_start is None
                and self.impact_time is not None and self.last_contact_time is not None
                and self.data.time-self.impact_time >= .16
                and self.data.time-self.last_contact_time >= .02):
            self.bank_start = float(self.data.time)

    def recovery_attitude_reference(self, desired, settling=False):
        if settling or self.kind != 'soft' or self.bank_start is None:
            return desired
        phase = (self.data.time-self.bank_start)/self.bank_duration
        if not 0 < phase < 1:
            return desired
        angle = -math.radians(self.bank_deg)*math.sin(math.pi*phase)**2
        rotation = np.array([[math.cos(angle),0,math.sin(angle)],
                             [0,1,0],[-math.sin(angle),0,math.cos(angle)]])
        return desired@rotation

    def summary(self):
        result = super().summary()
        result.update(bank_start_s=self.bank_start, bank_duration_s=self.bank_duration,
                      bank_reference_deg=self.bank_deg if self.kind=='soft' else 0.,
                      bank_is_control_maneuver=True)
        q = np.array([h[0] for h in self.history])
        if self.kind == 'soft':
            for arm in ('fl','fr'):
                index = self.model.joint(f'{arm}_bend_yaw').qposadr[0]
                result[f'{arm}_peak_yaw_deg'] = float(np.max(np.abs(q[:,index]))*180/math.pi)
                end = min(len(q),round(self.bank_start/self.model.opt.timestep)) if self.bank_start else len(q)
                result[f'{arm}_contact_yaw_deg'] = float(np.max(np.abs(q[:end,index]))*180/math.pi)
        else:
            result['fl_peak_yaw_deg'] = result['fr_peak_yaw_deg'] = 0.
            result['fl_contact_yaw_deg'] = result['fr_contact_yaw_deg'] = 0.
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,default=ROOT/'outputs'/'wall_collision_compression_then_lean')
    parser.add_argument('--no-video',action='store_true')
    parser.add_argument('--resume',action='store_true',help='Keep completed clips and resume this output')
    args = parser.parse_args()
    if args.output_dir.exists() and not args.resume:
        parser.error('Choose a new output directory; existing videos are preserved')
    cfg = json.loads(BASELINE.read_text())['config']
    args.output_dir.mkdir(parents=True,exist_ok=True)
    protocol=dict(wall_pitch_deg=0.,recovery_x=-.45,bank_duration_s=.9,
                  bank_formula='clip(12+5*v,20,55)*(1+.35*clip((v-3)/5,0,1))',
                  post_impact_s=7.,slow_window_s=.65)
    manifest=args.output_dir/'protocol.json'
    if manifest.exists() and json.loads(manifest.read_text())!=protocol:
        parser.error('Output protocol differs; choose a new directory')
    if not manifest.exists():
        manifest.write_text(json.dumps(protocol,indent=2))
    trials, clips = [], []
    for speed in SPEEDS:
        pair = [CompressionLeanTrial(kind,speed,cfg).run() for kind in ('soft','rigid')]
        for t in pair:
            row=t.summary()
            print(f"{speed:g} {t.kind}: tilt={row['peak_tilt_deg']:.1f}, front yaw="
                  f"{row['fl_peak_yaw_deg']:.1f}/{row['fr_peak_yaw_deg']:.1f}, "
                  f"recovered={row['recovered']}, broken={row['broken_arms']}",flush=True)
            if row['simulation_warning_count'] or abs(row['actual_impact_speed_m_s']-speed)>.02*speed:
                raise RuntimeError('Physics warning or speed mismatch')
            if t.kind=='soft' and (not row['recovered'] or row['ground_contact_time_s'] is not None):
                raise RuntimeError(f'Soft recovery failed at {speed}; trajectory is not fabricated')
        trials.extend(pair)
        path=args.output_dir/f'comparison_{speed:g}ms.mp4'
        if not args.no_video:
            complete=False
            if args.resume and path.exists():
                probe=subprocess.run(['ffprobe','-v','error','-show_entries','format=duration',
                                      '-of','json',str(path)],capture_output=True,text=True)
                complete=(probe.returncode==0 and float(json.loads(probe.stdout)['format']['duration'])>10)
                if not complete:
                    path.rename(path.with_name(f'{path.stem}.incomplete_{time.time_ns()}.mp4'))
            if not complete:
                render_video(pair,path,show_height_loss=True,slow_motion_window_s=.65,close_follow=True)
            clips.append(path)
    write_results(trials,args.output_dir,trials[0].cfg)
    (args.output_dir/'README.md').write_text(
        '# Front-arm compression, then recovery lean\n\n'
        'Vertical wall and original no-footer joint/contact/controller/fracture settings. '
        'Recovery target remains X=-.45 m (20 cm farther from wall than the original).\n'
        'First contact compresses the two front arms passively. No bank reference '
        'is added during this phase. After at least .16 s from first contact AND '
        '.02 s without wall contact, an explicit .9 s smooth pitch-bank recovery '
        'maneuver is commanded for soft only through the normal motor allocator. '
        'It is a flight-control reference choice, NOT a passive-impact model fit. '
        'There are no forced arm poses, velocities or artificial contact impulses.\n'
        'Both front yaw peaks are exported. Inset follows the vehicle at a closer '
        'top view; slow motion is extended to .65 s after impact to show both phases.\n'
        'Rigid keeps physical guard rings and the original assumed load-triggered '
        'arm fracture (110 Nm / 2000 N), with no exposed-propeller failure.\n'
        'Soft airborne recovery, measured impact speed and physics warnings are '
        'checked at each of 15 speeds. Not validated HoLoArm/Skydio X2 hardware.\n'
        'Previous videos and narrow-passage controls are unchanged.\n')
    if clips:
        if not (args.output_dir/'speed_sweep_comparison.mp4').exists():
            print(combine_videos(clips,args.output_dir),flush=True)


if __name__=='__main__':
    main()
