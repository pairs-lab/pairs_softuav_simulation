"""Regression checks for isolated rigid feedback and contact-damage cases."""
import unittest

import numpy as np

from collision_study import Trial, load_config
from rigid_wall_behavior_preview import RigidBehaviorTrial


class RigidBehaviorTests(unittest.TestCase):
    def test_contacts_change_only_in_exposed_instance(self):
        cfg = load_config()
        old = Trial('rigid', 2.5, cfg)
        exposed = RigidBehaviorTrial(2.5, 'exposed')
        guarded = RigidBehaviorTrial(2.5, 'guarded')
        np.testing.assert_array_equal(old.model.body_mass, exposed.model.body_mass)
        np.testing.assert_array_equal(old.model.body_inertia, exposed.model.body_inertia)
        for i in range(old.model.ngeom):
            name = old.model.geom(i).name
            if name:
                j = guarded.model.geom(name).id
                self.assertEqual(old.model.geom_contype[i], guarded.model.geom_contype[j])
        for arm in ('fl', 'fr', 'rl', 'rr'):
            guard = exposed.model.geom(f'{arm}_guard_0').id
            self.assertEqual(exposed.model.geom_contype[guard], 0)
            self.assertEqual(old.model.geom_contype[guard], 2)
        self.assertTrue(all(exposed.model.geom_contype[g] == 2 for g in exposed.rotor_geoms))
        self.assertEqual(load_config(), cfg)

    def test_guard_rings_are_visible_physical_and_preserve_inertia(self):
        guarded = RigidBehaviorTrial(2.5, 'guarded')
        old = Trial('rigid', 2.5, load_config())
        np.testing.assert_array_equal(guarded.model.body_mass, old.model.body_mass)
        np.testing.assert_array_equal(guarded.model.body_inertia, old.model.body_inertia)
        for arm in ('fl', 'fr', 'rl', 'rr'):
            for segment in range(20):
                g = guarded.model.geom(f'{arm}_guard_{segment}').id
                self.assertEqual(guarded.model.geom_rgba[g, 3], 1)
                self.assertEqual(guarded.model.geom_contype[g], 2)
                self.assertEqual(guarded.model.geom_conaffinity[g], 1)

    def test_intact_rigid_can_recover(self):
        s = RigidBehaviorTrial(2.5, 'guarded').run().summary()
        self.assertTrue(s['recovered'])
        self.assertEqual(s['damaged_rotors'], 0)
        self.assertIsNone(s['ground_contact_time_s'])
        self.assertGreater(s['peak_tilt_deg'], 15)
        self.assertGreater(s['recovery_time_s'], 1)
        self.assertFalse(s['x2_identified_model'])

    def test_prop_strike_really_has_contact_impulse_then_thrust_loss(self):
        t = RigidBehaviorTrial(2.5, 'exposed').run()
        s = t.summary()
        self.assertGreater(s['damaged_rotors'], 0)
        self.assertGreater(float(t.rotor_impulse.max()), t.rotor_impulse_threshold)
        self.assertIsNotNone(s['ground_contact_time_s'])
        self.assertFalse(s['recovered'])
        self.assertTrue(t.disarmed)
        np.testing.assert_array_equal(t.data.ctrl, 0)
        self.assertEqual(s['simulation_warning_count'], 0)
        # Replay must not show already-broken props during the approach.
        t.apply_replay_visuals(0)
        np.testing.assert_array_equal(t.model.geom_size[t.blade_geoms], t.original_blade_size)
        t.apply_replay_visuals(t.data.time)
        self.assertTrue(np.any(t.model.geom_size[t.blade_geoms, 0] < t.original_blade_size[:, 0]))

    def test_invalid_damage_threshold_rejected(self):
        with self.assertRaises(ValueError):
            RigidBehaviorTrial(rotor_damage_impulse=-1)


if __name__ == '__main__':
    unittest.main()
