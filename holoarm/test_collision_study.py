"""Soft CAD vs hard-arm wall impacts, optional failure and measured outputs."""
import csv
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import mujoco
import numpy as np

from collision_study import Trial, cli, load_config, write_report, write_results


class CollisionStudyTests(unittest.TestCase):
    def test_frozen_material_state_advances_during_settling(self):
        trial = Trial('soft', .6, load_config())
        joint = trial.model.joint('fl_bend_yaw')
        trial.data.qpos[joint.qposadr[0]] = .1
        mujoco.mj_forward(trial.model, trial.data)
        previous = trial.material.z.copy()
        trial.step(settling=True)
        self.assertLess(trial.data.qfrc_applied[joint.dofadr[0]], 0.)
        self.assertFalse(np.array_equal(trial.material.z, previous))
        self.assertFalse(trial.history)
        self.assertEqual(trial.model.jnt_stiffness[joint.id], 0.)

    def test_rigid_reference_has_no_assumed_fracture_by_default(self):
        trial = Trial('rigid', .6, load_config())
        trial.fracture(np.full(4, 1e6), np.full(4, 1e6))
        self.assertFalse(trial.fracture_enabled)
        self.assertFalse(trial.broken.any())
        self.assertTrue(trial.data.eq_active.all())

    def test_default_rigid_is_hard_arm_and_locked_cad_is_explicit(self):
        cfg = load_config()
        hard = Trial('rigid', .6, cfg)
        self.assertEqual(hard.model_variant, 'hard-arm')
        self.assertEqual(hard.model.mesh('rigid_x2_hull').id, 0)
        self.assertEqual((hard.model.nmesh, hard.model.nskin), (1, 0))
        self.assertEqual(hard.summary()['model_provenance']['geometry_model'], 'legacy_hard_arm')
        self.assertNotIn('rheology_sha256', hard.summary()['model_provenance'])
        cfg['rigid_model'] = 'locked-cad'
        locked = Trial('rigid', .6, cfg)
        self.assertEqual(locked.model_variant, 'locked-cad')
        self.assertEqual((locked.model.nmesh, locked.model.nskin), (37, 20))
        self.assertIn('rheology_sha256', locked.summary()['model_provenance'])

    def test_explicit_assumed_fracture_preserves_arm_state(self):
        trial = Trial('rigid', .6, load_config(), fracture=True)
        q, v = trial.data.qpos.copy(), trial.data.qvel.copy()
        trial.fracture(np.zeros(4), np.zeros(4))
        self.assertFalse(trial.broken.any())
        moments = np.array([trial.cfg['rigid_failure_moment_nm']+1, 0, 0, 0])
        trial.fracture(np.zeros(4), moments)
        self.assertTrue(trial.broken[0])
        self.assertFalse(trial.data.eq_active[trial.model.equality('fl_attachment').id])
        self.assertEqual(trial.data.ctrl[0], 0.)
        np.testing.assert_array_equal(trial.data.qpos, q)
        np.testing.assert_array_equal(trial.data.qvel, v)
        self.assertAlmostEqual(trial.model.body_mass.sum(), trial.cfg['mass_kg'])

    def test_approach_uses_each_actual_contact_envelope_and_free_body(self):
        for kind, variant in [('soft', 'hard-arm'), ('rigid', 'hard-arm'), ('rigid', 'locked-cad')]:
            with self.subTest(kind=kind, variant=variant):
                cfg = load_config()
                cfg['rigid_model'] = variant
                trial = Trial(kind, .6, cfg).prepare_approach()
                m, d = trial.model, trial.data
                self.assertEqual(d.time, 0.)
                self.assertFalse(trial.history)
                for joint in range(m.njnt):
                    if m.jnt_type[joint] == mujoco.mjtJoint.mjJNT_FREE:
                        self.assertEqual(d.qvel[m.jnt_dofadr[joint]], .6)
                geoms = [g for g in range(m.ngeom) if m.geom_bodyid[g] > 0 and
                         ((m.geom_contype[g] & m.geom_conaffinity[trial.wall]) or
                          (m.geom_contype[trial.wall] & m.geom_conaffinity[g]))]
                # Ask MuJoCo for surface distances independently of the
                # approach initializer's primitive-support calculation.
                gap = min(mujoco.mj_geomDistance(m, d, g, trial.wall, 1., None) for g in geoms)
                self.assertAlmostEqual(gap, .6*cfg['approach_s'], places=8)

    def test_low_speed_comparison_records_force_and_signed_compression(self):
        cfg = load_config()
        cfg['post_impact_s'] = 1.
        for kind in ('soft', 'rigid'):
            with self.subTest(kind=kind):
                trial = Trial(kind, .3, cfg).run()
                row = trial.summary()
                self.assertIsNotNone(trial.impact_time)
                self.assertAlmostEqual(row['actual_impact_speed_m_s'], .3, places=4)
                self.assertGreater(row['peak_wall_force_n'], 0.)
                self.assertGreater(row['first_impact_impulse_ns'], 0.)
                self.assertEqual(row['simulation_warning_count'], 0)
                self.assertTrue(np.isfinite(trial.data.qpos).all())
                self.assertTrue(np.isfinite(trial.data.qvel).all())
                self.assertFalse(trial.broken.any())
                if kind == 'soft':
                    start = round(trial.impact_time/trial.model.opt.timestep)-1
                    slides = np.array([h[0][trial.slide_indices] for h in trial.history[start:]])
                    self.assertAlmostEqual(row['peak_compression_mm'], max(0., slides.max())*1000)
                    self.assertAlmostEqual(row['peak_abs_slide_mm'], abs(slides).max()*1000)
                else:
                    self.assertIsNone(trial.material)
                    self.assertEqual(row['peak_compression_mm'], 0.)

    def test_results_keep_cad_provenance_and_unobserved_outcomes(self):
        cfg = load_config()
        for variant in ('hard-arm', 'locked-cad'):
            with self.subTest(variant=variant), tempfile.TemporaryDirectory() as directory:
                cfg['rigid_model'] = variant
                trials = [Trial(kind, .6, cfg) for kind in ('soft', 'rigid')]
                directory = Path(directory)
                summaries = write_results(trials, directory, cfg)
                write_report(summaries, directory, cfg)
                saved = json.loads((directory/'results.json').read_text())
                soft, rigid = saved['results']
                self.assertIsNone(soft['actual_impact_speed_m_s'])
                self.assertFalse(soft['recovered'])
                self.assertIn('rheology_sha256', soft['model_provenance'])
                self.assertEqual(rigid['model_variant'], variant)
                self.assertEqual('rheology_sha256' in rigid['model_provenance'], variant=='locked-cad')
                report = (directory/'REPORT.md').read_text()
                self.assertIn(rigid['model_label'], report)
                self.assertIn('No impact', report)
                self.assertIn('Not observed', report)
                if variant == 'hard-arm':
                    self.assertNotIn('same HoLoArm CAD', saved['visual']['rigid'])
                with (directory/'soft_0.6ms_trace.csv').open() as source:
                    columns = next(csv.reader(source))
                self.assertIn('arm_compression_mm', columns)
                self.assertIn('arm_abs_slide_mm', columns)

    def test_cli_preserves_configured_reference_unless_explicitly_overridden(self):
        for flag, expected in [([], 'locked-cad'), (['--rigid-model', 'hard-arm'], 'hard-arm')]:
            with self.subTest(flag=flag):
                cfg = load_config()
                cfg['rigid_model'] = 'locked-cad'
                trial = mock.Mock(model_label='test model')
                trial.run.return_value = trial
                trial.summary.return_value = dict(actual_impact_speed_m_s=.6, peak_bend_deg=0.,
                    peak_tilt_deg=0., broken_arms=0, recovery_time_s=None)
                with mock.patch('sys.argv', ['comparison', '--no-video', *flag]), \
                     mock.patch('collision_study.load_config', return_value=cfg), \
                     mock.patch('collision_study.Trial', return_value=trial) as constructor, \
                     mock.patch('collision_study.write_results', return_value=[]), \
                     mock.patch('collision_study.write_report'), redirect_stdout(io.StringIO()):
                    self.assertEqual(cli(), 0)
                self.assertEqual([call.args[2]['rigid_model'] for call in constructor.call_args_list],
                                 [expected, expected])


if __name__ == '__main__':
    unittest.main()
