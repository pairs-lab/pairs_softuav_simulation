"""Headless checks for the interactive HoLoArm scene and keyboard targets."""
import unittest
from types import SimpleNamespace

import mujoco
import numpy as np

from holoarm_narrow_passage_viewer import Flight, HeldControls, clean_flight_visuals
from soft_visual import add_soft_ligaments


class ViewerTests(unittest.TestCase):
    def setUp(self):
        self.flight = Flight()

    def test_gap_and_mass(self):
        m = self.flight.model
        self.assertAlmostEqual(float(m.body_mass.sum()), .97, places=10)
        left = m.geom('slot_wall_1').id
        right = m.geom('slot_wall_-1').id
        width = (m.geom_pos[left, 1]-m.geom_size[left, 1]
                 -m.geom_pos[right, 1]-m.geom_size[right, 1])
        self.assertAlmostEqual(width, .46)
        self.assertTrue(all(m.geom_bodyid[m.geom(name).id] == 0
            for name in ('slot_wall_1', 'slot_wall_-1')))
        names = [m.geom(i).name for i in range(m.ngeom)]
        self.assertNotIn('door_header', names)
        self.assertNotIn('start_pad', names)
        self.assertNotIn('goal_pad', names)
        self.assertFalse(any(name.startswith('route_') for name in names))

    def test_keys_hover_reset_and_bounds(self):
        f = self.flight
        keys = HeldControls()
        initial = f.target.copy()
        keys.update(f, {'I', 'J', 'U', 'Q'})
        np.testing.assert_allclose(f.target, initial)
        np.testing.assert_allclose(f.manual_velocity, [.6/np.sqrt(2), .6/np.sqrt(2), .45])
        self.assertGreater(f.yaw_rate, 0)
        keys.update(f, {'I', 'H'})
        np.testing.assert_allclose(f.manual_velocity, 0)
        np.testing.assert_allclose(f.target, f.data.qpos[:3])
        keys.update(f, {'I'})  # H remains latched until movement keys release.
        np.testing.assert_allclose(f.manual_velocity, 0)
        keys.update(f, set())
        keys.update(f, {'I'})
        self.assertGreater(f.manual_velocity[0], 0)
        f.target[:] = [100, -100, 100]
        f.clamp_target()
        np.testing.assert_allclose(f.target, [4.5, -1.6, 2.1])
        keys.update(f, {'R'})
        np.testing.assert_allclose(f.target, initial)
        self.assertEqual(f.yaw, 0)

    def test_continuous_hold_release_and_focus_loss(self):
        f = self.flight
        keys = HeldControls()
        keys.update(f, {'I'})
        # No autorepeat events at all: a held state must sustain flight.
        for _ in range(round(1/f.model.opt.timestep)):
            f.step()
        self.assertGreater(f.data.qpos[0], .3)
        release_position = f.data.qpos[:3].copy()
        keys.update(f, set())  # Also the poll result when focus is lost.
        self.assertAlmostEqual(f.target[0], release_position[0])
        np.testing.assert_allclose(f.manual_velocity, 0)
        for _ in range(round(2/f.model.opt.timestep)):
            f.step()
        self.assertLess(abs(f.data.qvel[0]), .05)
        self.assertLess(abs(f.data.qpos[0]-release_position[0]), .05)

    def test_buttons_are_edge_triggered_and_pause_brakes(self):
        keys = HeldControls()
        f = self.flight
        self.assertIn('F', keys.update(f, {'F'}))
        self.assertNotIn('F', keys.update(f, {'F'}))
        keys.update(f, {'I'})
        keys.update(f, {'I'}, enabled=False)
        self.assertFalse(f.manual_axes.any())
        keys.update(f, {'I'}, enabled=True)
        self.assertFalse(f.manual_axes.any())
        keys.update(f, set())
        keys.update(f, {'I'})
        self.assertTrue(f.manual_axes[0])

    def test_ligaments_are_visible_and_follow_bending(self):
        f = self.flight
        scene = mujoco.MjvScene(f.model, maxgeom=300)
        add_soft_ligaments(scene, f.model, f.data)
        self.assertEqual(scene.ngeom, 160)
        positions = np.array([scene.geoms[i].pos.copy() for i in range(scene.ngeom)])
        f.data.qpos[f.model.joint('fl_bend_yaw').qposadr[0]] = .3
        mujoco.mj_forward(f.model, f.data)
        scene.ngeom = 0
        add_soft_ligaments(scene, f.model, f.data)
        self.assertGreater(np.linalg.norm(scene.geoms[9].pos-positions[9]), .001)

    def test_hover(self):
        f = self.flight
        for _ in range(round(1/f.model.opt.timestep)):
            f.step()
        self.assertLess(np.linalg.norm(f.data.qpos[:3]-f.target), .01)
        self.assertTrue(np.isfinite(f.data.qpos).all())

    def test_native_shortcuts_do_not_change_flight_appearance(self):
        opt = mujoco.MjvOption()
        scene = mujoco.MjvScene(self.flight.model, maxgeom=300)
        opt.flags[mujoco.mjtVisFlag.mjVIS_INERTIA] = 1
        scene.flags[mujoco.mjtRndFlag.mjRND_ADDITIVE] = 1
        clean_flight_visuals(SimpleNamespace(opt=opt, user_scn=scene))
        self.assertEqual(opt.flags[mujoco.mjtVisFlag.mjVIS_INERTIA], 0)
        self.assertEqual(scene.flags[mujoco.mjtRndFlag.mjRND_ADDITIVE], 0)


if __name__ == '__main__':
    unittest.main()
