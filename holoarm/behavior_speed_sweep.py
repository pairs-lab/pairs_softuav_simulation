"""Legacy illustrative mechanics; independent of the frozen CAD model.

Render accepted soft behavior and exposed or guarded rigid profiles.

Speed labels are COMMAND targets. The finite-distance, thrust-driven approach
does not achieve every target; measured impact speed is printed in each panel
and exported. Profiles are intentionally not retuned to force recovery at high
speeds. Existing wall-survey videos and narrow-passage control are untouched.
"""
import argparse
import csv
import json
from pathlib import Path

from legacy_collision_study import combine_videos, write_results
from wall_behavior_preview import BehaviorTrial, render_preview
from rigid_wall_behavior_preview import RigidBehaviorTrial


from speed_sweep_config import SPEEDS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path,
        default=Path(__file__).resolve().parents[1]/'outputs'/'wall_collision_latest_behavior')
    parser.add_argument('--no-video', action='store_true')
    parser.add_argument('--rigid-scenario', choices=['exposed', 'guarded'], default='exposed',
        help='Exposed prop strike or rigid frame with visible physical protective rings')
    args = parser.parse_args()
    if args.output_dir.exists():
        parser.error('Choose a new output directory; existing outputs are preserved')
    args.output_dir.mkdir(parents=True)
    rows, clips = [], []
    for speed in SPEEDS:
        soft = BehaviorTrial(speed).run()
        rigid = RigidBehaviorTrial(speed, args.rigid_scenario).run()
        for trial in (soft, rigid):
            summary = trial.summary()
            if trial.impact_time is None:
                raise RuntimeError(f'{trial.kind} command {speed}: no wall contact')
            if summary['simulation_warning_count']:
                raise RuntimeError(f'{trial.kind} command {speed}: simulation warning')
            print(f"command={speed:g} {trial.kind}: impact={summary['actual_impact_speed_m_s']:.3f} m/s, "
                  f"tilt={summary['peak_tilt_deg']:.1f} deg, recovered={summary['recovered']}, "
                  f"ground={summary['ground_contact_time_s']}, "
                  f"rotor_damage={summary.get('damaged_rotors',0)}", flush=True)
            rows.append(summary)
            write_results([trial], args.output_dir/'traces'/f'{speed:g}ms'/trial.kind, trial.cfg)
        clip = args.output_dir/f'comparison_{speed:g}ms.mp4'
        if not args.no_video:
            render_preview([soft, rigid], clip, titles=[
                f'SOFT HoLoArm / command {speed:g} m/s',
                f'RIGID {args.rigid_scenario} / command {speed:g} m/s'])
            clips.append(clip)
    fields = list(dict.fromkeys(key for row in rows for key in row))
    with (args.output_dir/'summary.csv').open('w', newline='') as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    (args.output_dir/'results.json').write_text(json.dumps({
        'speed_protocol': 'Command targets; measured impact speeds differ. Finite-distance motor-driven approach.',
        'command_speeds_m_s': SPEEDS,
        'soft': 'Accepted BehaviorTrial, unchanged parameters',
        'rigid_scenario': args.rigid_scenario,
        'rigid': ('Visible physical guard rings, sampled feedback; no exposed-rotor damage'
                  if args.rigid_scenario == 'guarded' else
                  'Exposed-rotor scenario; assumed .3 N s damage threshold, not X2 validation'),
        'results': rows}, indent=2))
    (args.output_dir/'README.md').write_text(
        '# Latest soft/rigid behavior speed sweep\n\n'
        'Left: accepted soft HoLoArm behavior. Right: rigid X2-style '
        f'({args.rigid_scenario}) with sampled feedback. '
        'Guarded uses visible rigid contact rings and massless visual spokes; '
        'exposed uses assumed rotor damage. No soft profile parameters were retuned.\n\n'
        'Commands: 2–7 m/s in .5 m/s increments, plus 7.6 m/s. '
        '**These are command targets, not verified impact velocities.** '
        'Both panels show measured impact velocity; summary.csv/results.json export it. '
        'The latest finite-distance approach accelerates only with thrust and cannot '
        'reach the high command speeds before contact. This is NOT a measured '
        '2–7.6 m/s impact certification.\n\n'
        'Failures at higher commands are retained, not replaced with forced recovery. '
        'The rigid damage threshold is an uncalibrated scenario assumption; '
        'the soft model has no material-fracture law. Not identified HoLoArm/X2 hardware.\n\n'
        'Contact times are aligned across panels. The impact is replayed at .2x, '
        'remaining motion at 1x. Each clip is 1600x600, 30 fps. '
        'There is no bottom disclaimer strip.\n\n'
        'Per-model traces: traces/<command>ms/{soft,rigid}/ . '
        'Combined video: speed_sweep_comparison.mp4 .\n\n'
        'The previous videos and narrow-passage viewer/controller remain unchanged.\n')
    if clips:
        combined = combine_videos(clips, args.output_dir)
        print(f'Combined video: {combined}', flush=True)


if __name__ == '__main__':
    main()
