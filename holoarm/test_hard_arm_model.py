"""Identity and physics contracts for the restored generic hard-arm model."""
import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from cad_model import build_cad_model, patch_config
from hard_arm_model import ARMS, build_hard_arm_model, hard_arm_provenance
from legacy_collision_study import build_model as build_legacy_model
from legacy_collision_study import load_config as load_legacy_config


def config():
    path = Path(__file__).resolve().parent/"models/wall_collision_config.json"
    return patch_config(json.loads(path.read_text()))


class HardArmModelTests(unittest.TestCase):
    def test_restores_distinct_rigid_frame_and_artwork(self):
        hard = build_hard_arm_model(config())
        soft = build_cad_model("soft", config())
        self.assertEqual((hard.nq, hard.nv, hard.nu, hard.neq), (35, 30, 4, 4))
        self.assertEqual(hard.nskin, 0)
        self.assertEqual(hard.nmesh, 1)
        self.assertEqual(soft.nskin, 20)
        self.assertEqual(soft.nmesh, 37)
        self.assertTrue(np.all(hard.jnt_type == mujoco.mjtJoint.mjJNT_FREE))
        self.assertEqual(hard.mesh("rigid_x2_hull").id, 0)
        for arm in ARMS:
            self.assertGreaterEqual(hard.geom(f"rigid_x2_{arm}_boom").id, 0)
            self.assertGreaterEqual(hard.equality(f"{arm}_attachment").id, 0)

    def test_preserves_legacy_inertia_layout_and_guard_envelope(self):
        cfg = config()
        cfg["mass_kg"] = 1.17
        hard = build_hard_arm_model(cfg)
        legacy_cfg = load_legacy_config()
        legacy = build_legacy_model("rigid", legacy_cfg)
        np.testing.assert_allclose(hard.body_mass, legacy.body_mass * cfg["mass_kg"]/legacy_cfg["mass_kg"])
        np.testing.assert_allclose(hard.body_inertia, legacy.body_inertia * cfg["mass_kg"]/legacy_cfg["mass_kg"])
        hd, ld = mujoco.MjData(hard), mujoco.MjData(legacy)
        mujoco.mj_forward(hard, hd)
        mujoco.mj_forward(legacy, ld)
        for arm in ARMS:
            hs, ls = hard.site(f"thrust_{arm}").id, legacy.site(f"thrust_{arm}").id
            np.testing.assert_allclose(hd.site_xpos[hs], ld.site_xpos[ls])
            for segment in range(20):
                hg = hard.geom(f"{arm}_guard_{segment}").id
                lg = legacy.geom(f"{arm}_guard_{segment}").id
                np.testing.assert_allclose(hard.geom_size[hg], legacy.geom_size[lg])
                np.testing.assert_allclose(hard.geom_pos[hg], legacy.geom_pos[lg])
                self.assertEqual(hard.geom_rgba[hg, 3], 1)
                self.assertEqual(hard.geom_condim[hg], 1)
                self.assertTrue(hard.geom_contype[hg] or hard.geom_conaffinity[hg])

    def test_shared_wall_contact_timestep_and_motor_limits(self):
        cfg = config()
        cfg.update(wall_front_x_m=.72, wall_thickness_m=.18,
                   wall_width_m=3.1, wall_height_m=2.2)
        hard = build_hard_arm_model(cfg, width=800, height=600)
        wall = hard.geom("impact_wall").id
        self.assertAlmostEqual(hard.geom_pos[wall, 0] - hard.geom_size[wall, 0], .72)
        np.testing.assert_allclose(hard.geom_size[wall], [.09, 1.55, 1.1])
        self.assertEqual(hard.geom_condim[wall], 1)
        self.assertEqual(hard.vis.global_.offwidth, 800)
        self.assertEqual(hard.vis.global_.offheight, 600)
        self.assertEqual(hard.opt.timestep, cfg["timestep_s"])
        self.assertAlmostEqual(hard.body_mass.sum(), cfg["mass_kg"])
        self.assertTrue(hard.actuator_ctrllimited.all())
        np.testing.assert_allclose(hard.actuator_ctrlrange, np.tile([0, cfg["max_motor_thrust_n"]], (4, 1)))
        np.testing.assert_allclose(hard.actuator_gear[:, 5], [-.02, .02, .02, -.02])
        for arm in ARMS:
            guard = hard.geom(f"{arm}_guard_0").id
            self.assertTrue((hard.geom_contype[guard] & hard.geom_conaffinity[wall]) or
                            (hard.geom_contype[wall] & hard.geom_conaffinity[guard]))
            np.testing.assert_allclose(hard.geom_solref[guard], [cfg["contact_time_constant_s"], cfg["contact_damping_ratio"]])
        # The rotor forces and reaction moments belong to the detached arm,
        # rather than being artificial forces on the central body.
        data = mujoco.MjData(hard)
        data.ctrl[:] = 0
        data.ctrl[0] = 1
        mujoco.mj_forward(hard, data)
        np.testing.assert_allclose(data.qfrc_actuator[:6], 0)
        self.assertGreater(np.linalg.norm(data.qfrc_actuator[6:]), 0)

    def test_environment_is_cloned_and_contact_compatible(self):
        world = ET.Element("worldbody")
        ET.SubElement(world, "geom", name="test_barrier", type="box",
                      pos="1 0 1", size=".1 1 1", contype="2", conaffinity="1")
        original = ET.tostring(world)
        hard = build_hard_arm_model(config(), environment=world)
        self.assertEqual(ET.tostring(world), original)
        wall = hard.geom("test_barrier").id
        guard = hard.geom("fl_guard_0").id
        self.assertEqual(hard.geom_condim[wall], 1)
        self.assertTrue((hard.geom_contype[guard] & hard.geom_conaffinity[wall]) or
                        (hard.geom_contype[wall] & hard.geom_conaffinity[guard]))

    def test_visible_guard_produces_a_normal_wall_contact(self):
        cfg = config()
        hard = build_hard_arm_model(cfg)
        data = mujoco.MjData(hard)
        mujoco.mj_forward(hard, data)
        guards = [hard.geom(f"{arm}_guard_{segment}").id
                  for arm in ARMS for segment in range(20)]
        # Capsule support along world X is its radius plus the projected
        # half-length; translate all welded free bodies as one assembly.
        front = max(data.geom_xpos[g, 0] + hard.geom_size[g, 0] +
                    hard.geom_size[g, 1] * abs(data.geom_xmat[g].reshape(3, 3)[0, 2])
                    for g in guards)
        shift = cfg["wall_front_x_m"] + .001 - front
        for address in hard.jnt_qposadr:
            data.qpos[address] += shift
        mujoco.mj_forward(hard, data)
        wall = hard.geom("impact_wall").id
        contacts = [i for i, contact in enumerate(data.contact)
                    if wall in (contact.geom1, contact.geom2) and
                    any(g in (contact.geom1, contact.geom2) for g in guards)]
        self.assertTrue(contacts)
        for index in contacts:
            self.assertEqual(data.contact[index].dim, 1)
            force = np.empty(6)
            mujoco.mj_contactForce(hard, data, index, force)
            self.assertGreater(force[0], 0)
            np.testing.assert_allclose(force[1:], 0)

    def test_provenance_does_not_claim_soft_cad_or_measured_x2_physics(self):
        metadata = hard_arm_provenance(config())
        self.assertEqual(metadata["geometry_model"], "legacy_hard_arm")
        self.assertAlmostEqual(metadata["guard_radius_m"], .105)
        self.assertIn("not measured Skydio", metadata["scope"])
        self.assertNotIn("rheology_sha256", metadata)
        self.assertEqual(metadata["shared_trial_parameters"]["motor_time_constant_s"], .04)


if __name__ == "__main__":
    unittest.main()
