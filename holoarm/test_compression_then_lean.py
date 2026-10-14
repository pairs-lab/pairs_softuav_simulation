"""Protect passive front-arm compression before the optional recovery maneuver."""
import json
import unittest

import numpy as np

from compression_then_lean_sweep import CompressionLeanTrial,BASELINE
from restored_wall_behavior_sweep import RestoredTrial


class CompressionLeanTests(unittest.TestCase):
    def test_contact_phase_matches_original_frontal_physics(self):
        cfg=json.loads(BASELINE.read_text())['config']
        cfg['post_impact_s']=7.
        old=RestoredTrial('soft',7.6,cfg,wall_pitch_deg=0,recovery_x=-.45).run()
        new=CompressionLeanTrial('soft',7.6,cfg).run()
        self.assertIsNotNone(new.bank_start)
        stop=round(new.bank_start/new.model.opt.timestep)-1
        # All poses match until the control maneuver begins; no arm folding
        # animation, geometry change, altered spring or collision force.
        a=np.array([h[0] for h in old.history[:stop]])
        b=np.array([h[0] for h in new.history[:stop]])
        np.testing.assert_allclose(a,b,rtol=0,atol=1e-12)
        s=new.summary()
        self.assertGreater(s['fl_contact_yaw_deg'],35)
        self.assertGreater(s['fr_contact_yaw_deg'],35)
        self.assertTrue(s['recovered'])
        self.assertIsNone(s['ground_contact_time_s'])

    def test_high_speed_bank_recovers_without_floor_contact(self):
        cfg=json.loads(BASELINE.read_text())['config']
        t=CompressionLeanTrial('soft',8,cfg).run()
        s=t.summary()
        self.assertGreater(s['fl_contact_yaw_deg'],38)
        self.assertGreater(s['peak_tilt_deg'],40)
        self.assertTrue(s['recovered'])
        self.assertIsNone(s['ground_contact_time_s'])
        self.assertEqual(s['simulation_warning_count'],0)


if __name__=='__main__':
    unittest.main()
