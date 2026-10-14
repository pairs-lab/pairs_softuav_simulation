"""Preview-only regressions: no shared mechanics or passage changes."""
import unittest

import numpy as np

from collision_study import Trial, load_config
from wall_behavior_preview import BehaviorTrial


class BehaviorPreviewTests(unittest.TestCase):
    def test_shared_model_and_config_are_unchanged(self):
        original = load_config()
        preview = BehaviorTrial(2.5)
        baseline = Trial('soft', 2.5, original)
        self.assertEqual(load_config(), original)
        for name in ('body_mass', 'body_inertia', 'jnt_stiffness', 'dof_damping',
                     'jnt_range', 'geom_contype', 'geom_conaffinity', 'actuator_gear'):
            np.testing.assert_array_equal(getattr(preview.model, name), getattr(baseline.model, name))

    def test_dip_and_airborne_recovery(self):
        trial = BehaviorTrial(2.5).run()
        result = trial.summary()
        self.assertTrue(result['recovered'])
        self.assertIsNone(result['ground_contact_time_s'])
        self.assertGreater(result['peak_tilt_deg'], 15)
        self.assertGreater(result['altitude_drop_m'], .03)
        self.assertLess(result['altitude_drop_m'], .25)
        self.assertLess(result['final_position_error_m'], .03)
        self.assertLess(result['final_speed_m_s'], .05)
        self.assertEqual(result['simulation_warning_count'], 0)


if __name__ == '__main__':
    unittest.main()
