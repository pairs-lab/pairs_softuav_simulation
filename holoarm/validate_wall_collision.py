#!/usr/bin/env python3
"""Audit soft-CAD and selected rigid wall impacts without prescribing recovery."""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from collision_study import Trial, load_config


def relative_change(coarse, fine):
    if coarse == 0:
        return 0. if fine == 0 else None
    return abs(fine/coarse-1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parent.parent/
                        'outputs'/'wall_collision_study'/'numerical_audit.json')
    parser.add_argument('--impact-speed', type=float, default=.6)
    parser.add_argument('--rigid-model', choices=('hard-arm', 'locked-cad'), default=None,
                        help='Override the configured rigid reference (default configuration: hard-arm)')
    parser.add_argument('--duration', type=float, default=2., help='Simulated seconds after approach')
    args = parser.parse_args()
    if not math.isfinite(args.impact_speed) or args.impact_speed <= 0:
        parser.error('--impact-speed must be positive and finite')
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('--duration must be positive and finite')
    cfg = load_config()
    if args.rigid_model is not None:
        cfg['rigid_model'] = args.rigid_model
    cfg['post_impact_s'] = args.duration
    cases = []
    for step in (cfg['timestep_s'], cfg['timestep_s']/2):
        for kind in ('soft', 'rigid'):
            trial = Trial(kind, args.impact_speed, cfg, timestep=step, fracture=False).run()
            row = dict(timestep_s=step, **trial.summary())
            row['finite_state'] = bool(np.isfinite(trial.data.qpos).all() and np.isfinite(trial.data.qvel).all())
            cases.append(row)
            print(f"dt={step:g} {row['model_label']}: impact={row['actual_impact_speed_m_s']}, "
                  f"recovered={row['recovered']}, floor={row['ground_contact_time_s'] is not None}, "
                  f"joint_guard={row['joint_guard_reached']}", flush=True)
    comparisons = []
    for kind in ('soft', 'rigid'):
        coarse, fine = [row for row in cases if row['kind'] == kind]
        valid = all(row['actual_impact_speed_m_s'] is not None and row['finite_state'] and
                    row['simulation_warning_count'] == 0 for row in (coarse, fine))
        comparisons.append(dict(
            kind=kind, model_variant=coarse['model_variant'], model_label=coarse['model_label'],
            both_cases_have_finite_warning_free_impact=valid,
            recovery_classification_consistent=coarse['recovered'] == fine['recovered'],
            floor_contact_classification_consistent=(coarse['ground_contact_time_s'] is not None) ==
                                                  (fine['ground_contact_time_s'] is not None),
            joint_guard_classification_consistent=coarse['joint_guard_reached'] == fine['joint_guard_reached'],
            relative_peak_force_change=relative_change(coarse['peak_wall_force_n'], fine['peak_wall_force_n']),
            relative_10ms_deceleration_change=relative_change(coarse['peak_10ms_mean_body_deceleration_m_s2'],
                                                            fine['peak_10ms_mean_body_deceleration_m_s2']),
            slide_overtravel_coarse_mm=coarse['max_slide_limit_violation_mm'],
            slide_overtravel_fine_mm=fine['max_slide_limit_violation_mm']))
    passed = all(row['both_cases_have_finite_warning_free_impact'] and
                 row['recovery_classification_consistent'] and row['floor_contact_classification_consistent'] and
                 row['joint_guard_classification_consistent'] and row['relative_peak_force_change'] is not None and
                 row['relative_peak_force_change'] < .2 for row in comparisons)
    report = dict(config=cfg, requested_speed_m_s=args.impact_speed, cases=cases, comparisons=comparisons,
                  numerical_sensitivity_check_passed=passed,
                  limitations='A two-resolution comparison at one speed is not a convergence proof or flight validation. '
                  'The check compares observed outcome classifications and requires peak wall force to change by less '
                  'than 20%; it does not require a successful recovery. Joint-guard contact and slide overtravel remain '
                  'reported limitations, not material-safety claims. No fracture law is enabled.')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False))
    print(f'Audit: {args.output.resolve()} (passed={passed})')
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
