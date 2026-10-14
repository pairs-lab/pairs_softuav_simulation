#!/usr/bin/env python3
"""Check timestep sensitivity at 7.6 m/s and save the numerical audit."""
import argparse
import json
from pathlib import Path

from collision_study import Trial, load_config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path(__file__).resolve().parent.parent/
                        'outputs'/'wall_collision_study'/'numerical_audit.json')
    args = parser.parse_args()
    cfg = load_config()
    cfg['post_impact_s'] = 2.
    cases = []
    for step in (cfg['timestep_s'],cfg['timestep_s']/2):
        for kind in ('soft','rigid'):
            trial = Trial(kind,7.6,cfg,timestep=step).run()
            row = dict(timestep_s=step,**trial.summary())
            cases.append(row)
            print(f"dt={step:g} {kind}: recovered={row['recovered']}, broken={row['broken_arms']}",flush=True)
    comparisons = []
    for kind in ('soft','rigid'):
        coarse,fine = [r for r in cases if r['kind']==kind]
        comparisons.append(dict(
            kind=kind,
            recovery_classification_consistent=coarse['recovered']==fine['recovered'],
            detached_arm_count_consistent=coarse['broken_arms']==fine['broken_arms'],
            relative_peak_force_change=abs(fine['peak_wall_force_n']/coarse['peak_wall_force_n']-1),
            relative_10ms_deceleration_change=abs(fine['peak_10ms_mean_body_deceleration_m_s2']/
                                                coarse['peak_10ms_mean_body_deceleration_m_s2']-1),
            slide_overtravel_coarse_mm=coarse['max_slide_limit_violation_mm'],
            slide_overtravel_fine_mm=fine['max_slide_limit_violation_mm']))
    passed = all(r['recovery_classification_consistent'] and r['detached_arm_count_consistent']
                 and r['relative_peak_force_change']<.2 for r in comparisons)
    report = dict(config=cfg,cases=cases,comparisons=comparisons,
                  qualitative_outcome_check_passed=passed,
                  limitations='A two-resolution check of one speed is not a full convergence proof. '
                  'Axial limit overtravel does not converge sufficiently for material-safety claims. '
                  'Detached-body final positions are sensitive and are not validated by this check.')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2))
    print(f'Audit: {args.output.resolve()}')
    return 0 if passed else 1


if __name__=='__main__':
    raise SystemExit(main())
