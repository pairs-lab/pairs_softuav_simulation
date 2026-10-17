"""CAD/material and Windows focus contracts; no simulated success criteria."""
import ctypes as C
import hashlib,json,os
import unittest
from pathlib import Path
import numpy as np
import mujoco
from cad_model import CAD,FLIGHT,RHEO,ARM_NUMBERS,restoring
from collision_study import build_model,load_config,Trial
from viewer_keyboard import WindowsKeyboard

class CADIntegrationTests(unittest.TestCase):
    def test_geometry_mass_numbering_and_implicit_damping(self):
        m=build_model('soft',load_config());d=mujoco.MjData(m);mujoco.mj_forward(m,d)
        self.assertEqual((m.nq,m.nv,m.nu,m.nmesh,m.nskin),(19,18,4,37,20))
        self.assertAlmostEqual(m.body_mass.sum(),.97)
        for arm,i in ARM_NUMBERS.items():
            p=d.site_xpos[m.site(f'thrust_{arm}').id]-d.xpos[m.body('holoarm_core').id]
            np.testing.assert_allclose(p,FLIGHT['arms'][str(i)]['motor_body_m'],atol=1e-7)
            for suffix,key in [('slide','x'),('bend_yaw','alpha'),('bend_pitch','beta_up')]:
                j=m.joint(f'{arm}_{suffix}');self.assertEqual(m.jnt_stiffness[j.id],0)
                self.assertEqual(m.dof_damping[j.dofadr[0]],RHEO[key][0])
        j=m.joint('fr_slide');d.qvel[j.dofadr[0]]=.01;mujoco.mj_forward(m,d)
        self.assertAlmostEqual(d.qfrc_passive[j.dofadr[0]],-.01*RHEO['x'][0])

    def test_frozen_inputs_and_nonlinear_response(self):
        manifest=json.loads((CAD/'frozen_manifest.json').read_text())
        for name,digest in manifest['files'].items():self.assertEqual(hashlib.sha256((CAD/name).read_bytes()).hexdigest(),digest,name)
        self.assertAlmostEqual(restoring([.00294,0,0])[0],25.56*(1-np.exp(-1)))
        self.assertAlmostEqual(restoring([0,.1,0])[1],.383*np.tanh(1.8694*.1/.383))
        self.assertGreater(abs(restoring([0,0,-.4])[2]),abs(restoring([0,0,.4])[2]))

    def test_rigid_is_same_cad_and_no_legacy_transforms(self):
        soft=build_model('soft',load_config());rigid=build_model('rigid',load_config())
        self.assertEqual(rigid.nmesh,37);self.assertEqual(rigid.neq,4)
        self.assertAlmostEqual(soft.body_mass.sum(),rigid.body_mass.sum())
        self.assertFalse(any(rigid.jnt_type==mujoco.mjtJoint.mjJNT_SLIDE))
        with self.assertRaisesRegex(ValueError,'Legacy behavior transforms'):
            build_model('soft',load_config(),model_transform=lambda r:None)

    def test_arena_contact_has_correct_masks(self):
        from holoarm_narrow_passage_viewer import arena
        m=build_model('soft',load_config(),environment=arena(.46))
        wall=m.geom('slot_wall_1').id;guard=m.geom('guard_contact_1_0').id
        self.assertTrue((m.geom_contype[wall]&m.geom_conaffinity[guard]) or (m.geom_contype[guard]&m.geom_conaffinity[wall]))
        self.assertEqual(m.geom_condim[wall],1);self.assertEqual(m.geom_condim[guard],1)

    def test_prepare_approach_shares_viewer_protocol(self):
        t=Trial('soft',.6,load_config(),fracture=False);t.prepare_approach()
        self.assertEqual(t.data.time,0);self.assertAlmostEqual(t.data.qvel[0],.6)
        self.assertFalse(t.history);self.assertTrue(np.isfinite(t.material.z).all())

    def test_wall_thickness_keeps_front_contact_and_protocol(self):
        trials=[]
        for thickness in [.04,.18]:
            cfg=load_config();cfg['wall_thickness_m']=thickness;cfg['post_impact_s']=.4
            t=Trial('soft',.6,cfg,fracture=False)
            wall=t.model.geom('impact_wall').id
            self.assertAlmostEqual(2*t.model.geom_size[wall,0],thickness)
            self.assertAlmostEqual(t.model.geom_pos[wall,0]-t.model.geom_size[wall,0],cfg['wall_front_x_m'])
            t.run();self.assertIsNotNone(t.impact_time);trials.append(t)
        self.assertAlmostEqual(trials[0].impact_time,trials[1].impact_time)
        np.testing.assert_allclose(trials[0].data.qpos,trials[1].data.qpos,atol=1e-7,rtol=0)

    def test_passage_thickness_keeps_gap_and_connects_guides(self):
        from holoarm_narrow_passage_viewer import arena
        for thickness in [.2,.8]:
            m=build_model('soft',load_config(),environment=arena(.46,thickness))
            for side in [-1,1]:
                wall=m.geom(f'slot_wall_{side}').id
                self.assertAlmostEqual(2*m.geom_size[wall,0],thickness)
                self.assertAlmostEqual(abs(m.geom_pos[wall,1])-m.geom_size[wall,1],.23)
                start=m.geom_pos[wall,0]-thickness/2;end=m.geom_pos[wall,0]+thickness/2
                for label,x,sign in [('entrance',start,1),('exit',end,-1)]:
                    g=m.geom(f'{label}_guide_{side}').id;R=np.empty(9);mujoco.mju_quat2Mat(R,m.geom_quat[g]);R=R.reshape(3,3)
                    endpoint=m.geom_pos[g]-side*m.geom_size[g,1]*R[:,1]+sign*m.geom_size[g,0]*R[:,0]
                    np.testing.assert_allclose(endpoint[:2],[x,side*.23],atol=1e-9)

class WindowsFocusTests(unittest.TestCase):
    def keyboard(self,focused=True):
        class User:
            calls=0
            def GetForegroundWindow(self):return 10
            def GetWindowThreadProcessId(self,h,p):p._obj.value=os.getpid() if focused else os.getpid()+1
            def GetWindowTextW(self,h,b,n):b.value='MuJoCo test'
            def GetAsyncKeyState(self,key):self.calls+=1;return 0x8000 if key in [ord('I'),0x20] else 0
        k=WindowsKeyboard.__new__(WindowsKeyboard);k.user=User();k.DWORD=C.c_uint32;return k
    def test_focus_and_release_poll_contract(self):
        k=self.keyboard();self.assertEqual(k.poll(),{'I','SPACE'})
        k=self.keyboard(False);self.assertEqual(k.poll(),set());self.assertEqual(k.user.calls,0)

if __name__=='__main__':unittest.main()
