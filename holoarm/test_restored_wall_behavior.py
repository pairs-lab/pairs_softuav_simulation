"""Original-controller lean and load-triggered arm-fracture regressions."""
import json
import unittest

import numpy as np

from collision_study import Trial
from restored_wall_behavior_sweep import BASELINE, RestoredTrial


class RestoredBehaviorTests(unittest.TestCase):
    def setUp(self):
        self.cfg = json.loads(BASELINE.read_text())['config']

    def test_original_controller_and_mechanics_are_retained(self):
        self.assertIs(RestoredTrial.controller, Trial.controller)
        self.assertIs(RestoredTrial.run, Trial.run)
        for kind in ('soft','rigid'):
            restored = RestoredTrial(kind,3,self.cfg)
            old = Trial(kind,3,self.cfg)
            for field in ('body_mass','body_inertia','jnt_stiffness','dof_damping','actuator_gear'):
                np.testing.assert_array_equal(getattr(restored.model,field),getattr(old.model,field))
            self.assertEqual(restored.cfg, old.cfg)
            self.assertTrue(restored.fracture_enabled)

    def test_soft_high_speed_leans_but_recovers_airborne(self):
        t = RestoredTrial('soft',8,self.cfg).run()
        s = t.summary()
        self.assertTrue(s['recovered'])
        self.assertIsNone(s['ground_contact_time_s'])
        self.assertGreater(s['peak_tilt_deg'],20)
        self.assertLess(s['peak_tilt_deg'],60)
        self.assertGreater(s['altitude_drop_m'],.03)
        self.assertLess(s['altitude_drop_m'],.2)
        self.assertLess(abs(s['actual_impact_speed_m_s']-8),.16)
        self.assertEqual(s['simulation_warning_count'],0)

    def test_rigid_high_speed_has_guarded_arm_fracture(self):
        t = RestoredTrial('rigid',8,self.cfg).run()
        self.assertGreater(t.summary()['broken_arms'],0)
        for i in np.where(t.broken)[0]:
            self.assertEqual(t.data.ctrl[i],0)
        self.assertEqual(t.cfg['rigid_failure_moment_nm'],110)
        self.assertEqual(t.cfg['rigid_failure_force_n'],2000)

    def test_longer_retreat_keeps_the_feedback_law_and_airborne_recovery(self):
        t = RestoredTrial('soft',3,self.cfg,wall_pitch_deg=5,recovery_x=-.45).run()
        s = t.summary()
        self.assertTrue(s['recovered'])
        self.assertIsNone(s['ground_contact_time_s'])
        self.assertEqual(t.recovery_target[0],-.45)
        self.assertLess(abs(t.data.qpos[0]+.45),.02)
        self.assertGreater(s['post_impact_retreat_m'],.5)
        self.assertIs(RestoredTrial.controller,Trial.controller)


if __name__ == '__main__':
    unittest.main()
