"""Legacy illustrative mechanics; independent of the frozen CAD model.

Qualitative rigid speed progression with a level, frontal initial condition.

No speed-conditioned crash/wobble rules or deliberate heading offset.
Delayed feedback and load-triggered arm fracture
produce the response. Assumptions are NOT identified Skydio X2 parameters.
"""
import argparse
from collections import deque
import csv
import json
import math
from pathlib import Path
import subprocess
import time

import mujoco
import numpy as np

from speed_sweep_config import SPEEDS
from legacy_collision_study import render_video, write_results, combine_videos
from compression_then_lean_sweep import CompressionLeanTrial
from restored_wall_behavior_sweep import RestoredTrial, BASELINE, ROOT

SELECTED_SPEEDS = list(SPEEDS)


class OffsetRigidTrial(RestoredTrial):
    def __init__(self, speed, cfg):
        self.heading = 0.
        cfg=dict(cfg)
        cfg['post_impact_s']=7.
        super().__init__('rigid',speed,cfg,wall_pitch_deg=0,recovery_x=-.45)
        self.sensor=mujoco.MjData(self.model)
        self.sample_steps=round((1/150)/self.model.opt.timestep)
        self.delay_steps=round(.012/self.model.opt.timestep)
        self.observations=deque(maxlen=self.delay_steps+1)
        self.sensor_tick=0
        self.freeze_until=-1.
        self.shock_events=0

    def recovery_attitude_reference(self, desired, settling=False):
        # Keep the original force-derived reference, with no artificial yaw.
        return desired

    def controller(self, settling=False):
        actual=self.data
        # The inherited impact protocol sets incoming velocity and resets
        # simulation time after settling. Initialise the observer to that
        # defined test condition, rather than retaining stale hover velocity.
        # This is initialisation only, never a post-impact state reset.
        if self.observations and actual.time<self.observations[-1][2]:
            self.observations.clear()
            self.sensor.qpos[:],self.sensor.qvel[:]=actual.qpos,actual.qvel
            self.sensor.eq_active[:]=actual.eq_active
            self.sensor.time=actual.time
            mujoco.mj_forward(self.model,self.sensor)
            self.sensor_tick=0
            self.freeze_until=-1.
        self.observations.append((actual.qpos.copy(),actual.qvel.copy(),actual.time))
        if self.sensor_tick%self.sample_steps==0 and actual.time>=self.freeze_until:
            q,v,stamp=self.observations[0]
            self.sensor.qpos[:],self.sensor.qvel[:],self.sensor.time=q,v,stamp
            self.sensor.eq_active[:]=actual.eq_active
            mujoco.mj_forward(self.model,self.sensor)
        self.sensor_tick+=1
        self.data=self.sensor
        try:
            super().controller(settling)
        finally:
            self.data=actual
        actual.ctrl[:]=self.thrust

    def contacts(self):
        result=super().contacts()
        # Read the solved acceleration before mj_forward refreshes caches.
        # The 16 g threshold and 40 ms sample hold are sensitivity assumptions.
        if result[0]>.01 and np.linalg.norm(self.data.qacc[:3])>16*9.81:
            if self.data.time>=self.freeze_until:
                self.shock_events+=1
            self.freeze_until=max(self.freeze_until,self.data.time+.04)
        return result

    def summary(self):
        result=super().summary()
        q=np.array([h[0] for h in self.history])
        v=np.array([h[1] for h in self.history])
        quat=q[:,3:7]
        yaw=np.arctan2(2*(quat[:,0]*quat[:,3]+quat[:,1]*quat[:,2]),
                       1-2*(quat[:,2]**2+quat[:,3]**2))-self.heading
        principal=np.arctan2(np.sin(yaw),np.cos(yaw))
        contact_index=round(self.impact_time/self.model.opt.timestep) if self.impact_time else 0
        end=min(len(v),contact_index+round(.25/self.model.opt.timestep))
        result.update(approach_heading_deg=0.,feedback_nominal_sample_hz=150.,
            feedback_actual_sample_hz=1/(self.sample_steps*self.model.opt.timestep),
            feedback_delay_s=.012,assumed_shock_hold_s=.04,assumed_shock_threshold_g=16.,
            shock_events=self.shock_events,
            first_impact_peak_angular_speed_deg_s=float(np.max(np.linalg.norm(v[contact_index:end,3:6],axis=1))*180/math.pi),
            peak_yaw_deviation_deg=float(np.max(np.abs(principal))*180/math.pi),
            x2_identified_model=False,forced_speed_outcome=False)
        return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,default=ROOT/'outputs'/'wall_collision_rigid_level_start')
    parser.add_argument('--resume',action='store_true')
    parser.add_argument('--no-video',action='store_true')
    parser.add_argument('--rigid-profile',choices=('frontal','transient'),default='frontal')
    args=parser.parse_args()
    if args.output_dir.exists() and not args.resume:
        parser.error('Choose a new directory; existing videos are preserved')
    args.output_dir.mkdir(parents=True,exist_ok=True)
    protocol=dict(speeds=SELECTED_SPEEDS,rigid_heading_deg=0.,feedback_delay_s=.012,
                  feedback_nominal_hz=150.,shock_hold_s=.04,shock_threshold_g=16.,
                  arm_fracture_force_n=2000.,arm_fracture_moment_nm=110.,
                  soft_profile='unchanged CompressionLeanTrial',post_impact_s=7.)
    if args.rigid_profile=='transient':
        from rigid_recovery_behavior import RigidRecoveryTrial
        rigid_class=RigidRecoveryTrial
        protocol.update(rigid_profile='illustrative transient feedback degradation',
                        assumed_post_contact_delay_s=.05,
                        assumed_post_contact_delay_duration_s=2.8,
                        controller_matched_to_soft=False)
    else:
        rigid_class=OffsetRigidTrial
    manifest=args.output_dir/'protocol.json'
    if manifest.exists() and json.loads(manifest.read_text())!=protocol:
        parser.error('Output protocol differs')
    if not manifest.exists():
        manifest.write_text(json.dumps(protocol,indent=2))
    cfg=json.loads(BASELINE.read_text())['config']
    rows,clips=[],[]
    for speed in SELECTED_SPEEDS:
        pair=[CompressionLeanTrial('soft',speed,cfg).run(),rigid_class(speed,cfg).run()]
        for t in pair:
            s=t.summary()
            print(f"{speed:g} {t.kind}: tilt={s['peak_tilt_deg']:.1f}, yaw="
                  f"{s.get('peak_yaw_deviation_deg',0):.1f}, recovered={s['recovered']}, "
                  f"broken={s['broken_arms']}",flush=True)
            if s['simulation_warning_count'] or abs(s['actual_impact_speed_m_s']-speed)>.02*speed:
                raise RuntimeError('Invalid physics or impact velocity')
            if t.kind=='soft' and (not s['recovered'] or s['ground_contact_time_s'] is not None):
                raise RuntimeError('Approved soft profile failed')
            rows.append(s)
            write_results([t],args.output_dir/'traces'/f'{speed:g}ms'/t.kind,t.cfg)
        clip=args.output_dir/f'comparison_{speed:g}ms.mp4'
        if not args.no_video:
            complete=False
            if args.resume and clip.exists():
                probe=subprocess.run(['ffprobe','-v','error','-show_entries','format=duration',
                                      '-of','json',str(clip)],capture_output=True,text=True)
                complete=probe.returncode==0 and float(json.loads(probe.stdout)['format']['duration'])>10
                if not complete:
                    clip.rename(clip.with_name(f'{clip.stem}.incomplete_{time.time_ns()}.mp4'))
            if not complete:
                render_video(pair,clip,show_height_loss=True,slow_motion_window_s=.65,
                             close_follow=True,show_yaw_motion=True)
            clips.append(clip)
        if args.rigid_profile=='transient' and 2<=speed<=5:
            soft,rigid=[t.summary() for t in pair]
            if (not rigid['recovered'] or rigid['broken_arms'] or
                rigid['ground_contact_time_s'] is not None or
                rigid['peak_tilt_deg']<=soft['peak_tilt_deg'] or
                rigid['recovery_time_s']<=soft['recovery_time_s']+.5):
                raise RuntimeError('Illustrative low-speed recovery criteria failed')
    fields=list(dict.fromkeys(k for r in rows for k in r))
    with (args.output_dir/'summary.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)
    (args.output_dir/'results.json').write_text(json.dumps(dict(protocol=protocol,results=rows),indent=2))
    profile_note=(
        'TRANSIENT PROFILE: after contact, assumed observation delay increases '
        'from 12 ms to 50 ms for 2.8 s, then returns to 12 ms. The same parameters '
        'apply at every speed. This is a deliberately selected illustrative '
        'control-response scenario, NOT a measured IMU effect, matched-controller '
        'comparison, or proof that compliance alone explains the difference. '
        'No pose/velocity kicks or preset lean are applied. The reference video '
        '(1:50–approximately 3:12) shows hover arm disturbances, NOT known-speed '
        'wall impacts. It does not establish a 2–5 m/s response curve.\n\n'
        if args.rigid_profile=='transient' else '')
    (args.output_dir/'README.md').write_text(
        '# Qualitative rigid impact progression\n\n'
        +profile_note+
        'Includes 2–7 m/s every .5 m/s and 7.6 m/s; omits 1.5,7.5,8 m/s.\n'
        'Soft is the approved compression-then-controlled-bank profile, unchanged. '
        'Rigid has physical guards and starts level, aligned with travel. '
        'No heading offset, forced torque or asymmetric contact setup is added. '
        'A symmetric frontal collision need not produce substantial yaw. '
        'The same rigid controller gains/motor response/material assumptions are '
        'used at every speed. No speed-branch scripts force instability or fracture.\n'
        'Rigid observations are sampled at nominal 150 Hz with 12 ms delay. '
        'A solved wall shock above an assumed 16 g holds observations for an assumed '
        '40 ms; this can produce yaw/tilt overshoot naturally. These are sensitivity '
        'assumptions, NOT Skydio X2 measurements. Arm fracture retains the original '
        '110 Nm /2000 N assumed contact-load thresholds. Rotors stop only when '
        'their arm physically detaches via weld release. No propeller damage rule.\n'
        'The 2–3 /3–5 /5–7.6 m/s descriptions are qualitative expectations, not '
        'calibrated probabilities or guarantees. Inspect JSON for measured impact '
        'speed, body deceleration, first-impact angular rate, yaw, tilt and fracture.\n'
        'This is an illustrative comparison, NOT a controlled hardware superiority '
        'test: soft is frontal with an added bank reference; rigid is frontal with '
        'finite feedback. Insets/overlay now show yaw deviation as well as tilt.\n'
        'Previous videos and narrow-passage control are unchanged.\n'
        'Context: https://arxiv.org/html/2105.12273v1 , '
        'https://arxiv.org/html/2605.25790v1 .\n')
    if clips and not (args.output_dir/'speed_sweep_comparison.mp4').exists():
        print(combine_videos(clips,args.output_dir),flush=True)


if __name__=='__main__':
    main()
