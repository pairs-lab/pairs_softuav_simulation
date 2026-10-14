"""Frontal rigid scenario: level approach and load-dependent failure."""
import json
import unittest

import numpy as np

from rigid_offset_behavior_sweep import OffsetRigidTrial,SELECTED_SPEEDS,BASELINE
from compression_then_lean_sweep import CompressionLeanTrial


class OffsetRigidTests(unittest.TestCase):
    def setUp(self):
        self.cfg=json.loads(BASELINE.read_text())['config']

    def test_trimmed_speed_selection(self):
        from collision_study import SPEEDS as baseline_speeds
        from behavior_speed_sweep import SPEEDS as behavior_speeds
        expected=[2.,2.5,3.,3.5,4.,4.5,5.,5.5,6.,6.5,7.,7.6]
        self.assertEqual(SELECTED_SPEEDS,expected)
        self.assertEqual(baseline_speeds,expected)
        self.assertEqual(behavior_speeds,expected)
        self.assertEqual(len(SELECTED_SPEEDS),12)
        self.assertFalse(set(SELECTED_SPEEDS)&{1.5,7.5,8})
        self.assertIn(7.6,SELECTED_SPEEDS)

    def test_level_frontal_hit_recovers_without_manufactured_yaw(self):
        t=OffsetRigidTrial(2.5,self.cfg).run()
        s=t.summary()
        self.assertTrue(s['recovered'])
        self.assertEqual(s['broken_arms'],0)
        self.assertIsNone(s['ground_contact_time_s'])
        self.assertGreater(s['peak_tilt_deg'],10)
        self.assertEqual(s['approach_heading_deg'],0.)
        # A symmetric frontal collision need not induce yaw. Verify the whole
        # approach is aligned, rather than requiring a scripted rotation.
        approach=np.array([h[0][3:7] for h in t.history[:int(t.impact_time/t.model.opt.timestep)-1]])
        np.testing.assert_allclose(approach[:,1:],0.,atol=1e-5)
        self.assertLess(s['peak_yaw_deviation_deg'],.1)
        self.assertGreater(s['first_impact_peak_angular_speed_deg_s'],20)
        self.assertEqual(s['simulation_warning_count'],0)
        self.assertLess(abs(s['actual_impact_speed_m_s']-2.5),.05)
        self.assertFalse(s['forced_speed_outcome'])

    def test_stronger_hit_fails_by_contact_load_not_speed_script(self):
        t=OffsetRigidTrial(7.6,self.cfg).run()
        s=t.summary()
        self.assertGreater(s['broken_arms'],0)
        self.assertFalse(s['recovered'])
        self.assertTrue(s['peak_arm_contact_force_n']>2000 or s['peak_arm_contact_moment_nm']>110)
        for i in np.where(t.broken)[0]:
            self.assertEqual(t.data.ctrl[i],0)

    def test_soft_model_config_is_not_changed(self):
        before=CompressionLeanTrial('soft',3,self.cfg)
        OffsetRigidTrial(3,self.cfg)
        after=CompressionLeanTrial('soft',3,self.cfg)
        self.assertEqual(before.cfg,after.cfg)
        np.testing.assert_array_equal(before.model.body_mass,after.model.body_mass)
        np.testing.assert_array_equal(before.model.jnt_stiffness,after.model.jnt_stiffness)


if __name__=='__main__':
    unittest.main()
