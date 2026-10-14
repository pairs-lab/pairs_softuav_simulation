"""Physics invariants and controller/fracture regressions for the wall study."""
import unittest

import mujoco
import numpy as np

from collision_study import Trial, build_model, load_config
from soft_visual import ligament_paths


class CollisionStudyTests(unittest.TestCase):
    def test_holoarm_visual_preserves_physics(self):
        cfg=load_config()
        legacy=build_model('soft',cfg,soft_visual='legacy')
        styled=build_model('soft',cfg)
        for name in ('body_mass','body_inertia','body_ipos','body_iquat','body_pos','body_quat',
                     'jnt_type','jnt_qposadr','jnt_dofadr','jnt_stiffness','dof_damping',
                     'actuator_gear','tendon_stiffness','tendon_damping'):
            np.testing.assert_allclose(getattr(legacy,name),getattr(styled,name),rtol=1e-12,atol=1e-14)
        for g in range(styled.ngeom):
            name=mujoco.mj_id2name(styled,mujoco.mjtObj.mjOBJ_GEOM,g) or ''
            if name.startswith('soft_holoarm_'):
                self.assertEqual(styled.geom_contype[g],0)
                self.assertEqual(styled.geom_conaffinity[g],0)

    def test_visual_ligaments_follow_actual_joint_sites(self):
        trial=Trial('soft',2.,load_config())
        before=ligament_paths(trial.model,trial.data)
        trial.data.qpos[trial.model.joint('fl_bend_yaw').qposadr[0]]=.5
        mujoco.mj_forward(trial.model,trial.data)
        after=ligament_paths(trial.model,trial.data)
        self.assertGreater(np.linalg.norm(after[0][-1]-before[0][-1]),.001)
        for arm_index,arm in enumerate(('fl','fr','rl','rr')):
            for band in range(4):
                path=after[arm_index*4+band]
                for point,end in ((path[0],'fixed'),(path[-1],'moving')):
                    site=trial.model.site(f'{arm}_ligament_{end}_{band}').id
                    np.testing.assert_allclose(point,trial.data.site_xpos[site],atol=1e-14)

    def test_x2_visual_preserves_physics(self):
        cfg=load_config()
        legacy=build_model('rigid',cfg,rigid_visual='legacy')
        styled=build_model('rigid',cfg)
        for name in ('body_mass','body_inertia','body_ipos','body_iquat','body_pos','body_quat',
                     'jnt_type','jnt_qposadr','jnt_dofadr','eq_data','actuator_gear'):
            np.testing.assert_allclose(getattr(legacy,name),getattr(styled,name),rtol=1e-12,atol=1e-14)
        self.assertGreater(styled.ngeom,legacy.ngeom)
        for g in range(styled.ngeom):
            name=mujoco.mj_id2name(styled,mujoco.mjtObj.mjOBJ_GEOM,g) or ''
            if name.startswith('rigid_x2_'):
                self.assertEqual(styled.geom_contype[g],0)
                self.assertEqual(styled.geom_conaffinity[g],0)
        for arm in ('fl','fr','rl','rr'):
            self.assertEqual(styled.geom(f'{arm}_guard_0').rgba[3],0)

    def test_mass_and_guard_geometry_match(self):
        cfg = load_config()
        soft, rigid = (build_model(kind,cfg) for kind in ('soft','rigid'))
        self.assertAlmostEqual(soft.body_mass.sum(),cfg['mass_kg'],places=10)
        self.assertAlmostEqual(rigid.body_mass.sum(),cfg['mass_kg'],places=10)
        for arm in ('fl','fr','rl','rr'):
            for segment in range(20):
                name = f'{arm}_guard_{segment}'
                np.testing.assert_allclose(soft.geom(name).size,rigid.geom(name).size)
        self.assertEqual(rigid.neq,4)

    def test_soft_yaw_has_passive_restoring_force(self):
        t = Trial('soft',2.,load_config())
        joint = t.model.joint('fl_bend_yaw')
        t.data.qpos[joint.qposadr[0]] = .3
        mujoco.mj_forward(t.model,t.data)
        self.assertLess(t.data.qfrc_passive[joint.dofadr[0]],0)

    def test_fracture_requires_load_and_preserves_arm_state(self):
        t = Trial('rigid',2.,load_config())
        q, v = t.data.qpos.copy(),t.data.qvel.copy()
        t.fracture(np.zeros(4),np.zeros(4))
        self.assertFalse(t.broken.any())
        load = np.array([t.cfg['rigid_failure_moment_nm']+1,0,0,0])
        t.fracture(np.zeros(4),load)
        self.assertTrue(t.broken[0])
        self.assertFalse(t.data.eq_active[t.model.equality('fl_attachment').id])
        self.assertEqual(t.data.ctrl[0],0)
        np.testing.assert_array_equal(t.data.qpos,q)
        np.testing.assert_array_equal(t.data.qvel,v)
        self.assertAlmostEqual(t.model.body_mass.sum(),t.cfg['mass_kg'],places=10)

    def test_low_speed_both_recover_without_damage(self):
        for kind in ('soft','rigid'):
            with self.subTest(kind=kind):
                t = Trial(kind,1.5,load_config()).run()
                self.assertAlmostEqual(t.actual_impact_speed,1.5,places=3)
                self.assertIsNotNone(t.recovery_time)
                self.assertIsNone(t.ground_time)
                self.assertFalse(t.broken.any())
                self.assertTrue(np.all(np.isfinite(t.data.qpos)))

    def test_no_fracture_control_is_unbreakable(self):
        t = Trial('rigid',8.,load_config(),fracture=False)
        t.fracture(np.full(4,1e6),np.full(4,1e6))
        self.assertFalse(t.broken.any())
        self.assertTrue(t.data.eq_active.all())


if __name__ == '__main__':
    unittest.main()
