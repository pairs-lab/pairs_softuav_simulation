"""CAD wall-impact integration, optional failure and measured-output contracts."""
import csv
import json
from pathlib import Path
import tempfile
import unittest

import mujoco
import numpy as np

from collision_study import Trial, load_config, write_report, write_results


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

    def test_locked_reference_has_no_assumed_fracture_by_default(self):
        trial = Trial('rigid', .6, load_config())
        trial.fracture(np.full(4, 1e6), np.full(4, 1e6))
        self.assertFalse(trial.fracture_enabled)
        self.assertFalse(trial.broken.any())
        self.assertTrue(trial.data.eq_active.all())

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

    def test_locked_approach_initializes_each_free_body(self):
        trial = Trial('rigid', .6, load_config()).prepare_approach()
        self.assertEqual(trial.data.time, 0.)
        self.assertFalse(trial.history)
        for joint in range(trial.model.njnt):
            if trial.model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_FREE:
                self.assertEqual(trial.data.qvel[trial.model.jnt_dofadr[joint]], .6)

    def test_low_speed_cad_impact_records_force_and_signed_compression(self):
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
        trial = Trial('soft', .6, cfg)
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            summaries = write_results([trial], directory, cfg)
            write_report(summaries, directory, cfg)
            saved = json.loads((directory/'results.json').read_text())
            row = saved['results'][0]
            self.assertIsNone(row['actual_impact_speed_m_s'])
            self.assertFalse(row['recovered'])
            self.assertIn('rheology_sha256', row['model_provenance'])
            report = (directory/'REPORT.md').read_text()
            self.assertIn('No impact', report)
            self.assertIn('Not observed', report)
            with (directory/'soft_0.6ms_trace.csv').open() as source:
                columns = next(csv.reader(source))
            self.assertIn('arm_compression_mm', columns)
            self.assertIn('arm_abs_slide_mm', columns)


if __name__ == '__main__':
    unittest.main()
