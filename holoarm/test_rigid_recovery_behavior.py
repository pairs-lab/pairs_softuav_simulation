"""Regression checks for the explicitly illustrative recovery scenario."""
import json
import unittest

import numpy as np

from compression_then_lean_sweep import CompressionLeanTrial
from rigid_offset_behavior_sweep import BASELINE
from rigid_recovery_behavior import RigidRecoveryTrial


class RigidRecoveryTests(unittest.TestCase):
    def test_low_speed_response_and_level_approach(self):
        cfg=json.loads(BASELINE.read_text())['config']
        for speed in (2.,3.,5.):
            with self.subTest(speed=speed):
                rigid=RigidRecoveryTrial(speed,cfg).run()
                soft=CompressionLeanTrial('soft',speed,cfg).run()
                r,s=rigid.summary(),soft.summary()
                self.assertTrue(r['recovered'])
                self.assertEqual(r['broken_arms'],0)
                self.assertIsNone(r['ground_contact_time_s'])
                self.assertGreater(r['peak_tilt_deg'],s['peak_tilt_deg'])
                self.assertGreater(r['recovery_time_s'],s['recovery_time_s']+.5)
                self.assertEqual(r['simulation_warning_count'],0)
                self.assertAlmostEqual(r['actual_impact_speed_m_s'],speed,places=4)
                approach=np.array([h[0][3:7] for h in rigid.history[:int(rigid.impact_time/rigid.model.opt.timestep)-1]])
                np.testing.assert_allclose(approach[:,1:],0.,atol=1e-5)
                self.assertFalse(r['controller_matched_to_soft'])
                self.assertFalse(r['x2_identified_model'])
                self.assertEqual(rigid.active_delay_s,.012)


if __name__=='__main__':
    unittest.main()
