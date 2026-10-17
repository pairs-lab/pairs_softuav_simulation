"""Latest model/control shared across keyboard and ROS without requiring ROS."""
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from pairs_softuav_sim.holoarm import HoLoArmSim
from holoarm_narrow_passage_viewer import Flight


class HoLoArmAdapterTests(unittest.TestCase):
    def test_model_and_trajectory_match_viewer(self):
        sim=HoLoArmSim()
        flight=Flight()
        np.testing.assert_array_equal(sim.m.body_mass,flight.model.body_mass)
        np.testing.assert_array_equal(sim.m.jnt_stiffness,flight.model.jnt_stiffness)
        np.testing.assert_array_equal(sim.d.qpos,flight.data.qpos)
        self.assertEqual(sim.m.nmesh,37)
        self.assertEqual(sim.m.nskin,20)
        for _ in range(1000):
            sim.step();flight.step()
        sim.set_target(.7,0,1.2)
        flight.target[:]=[.7,0,1.2]
        for _ in range(2000):
            sim.step();flight.step()
        np.testing.assert_array_equal(sim.d.qpos,flight.data.qpos)
        np.testing.assert_array_equal(sim.d.qvel,flight.data.qvel)
        scene=mujoco.MjvScene(sim.m,maxgeom=300)
        sim.add_visuals(scene)
        self.assertEqual(scene.ngeom,0)  # CAD skins are built into the model
        self.assertEqual(sim.get_state()['time'],float(sim.d.time))

    def test_ros_targets_cross_the_gap(self):
        sim=HoLoArmSim()
        for _ in range(round(2/sim.timestep)):
            sim.step()
        sim.set_target(3.65,0,1.2)
        contacts=0
        low=sim.d.qpos[2]
        for _ in range(round(14/sim.timestep)):
            sim.step()
            low=min(low,sim.d.qpos[2])
            contacts+=int(sim.d.ncon>0)
        self.assertGreater(sim.d.qpos[0],3.3)
        self.assertLess(np.linalg.norm(sim.d.qvel[:3]),.05)
        self.assertGreater(contacts,0)
        self.assertGreater(low,.7)
        self.assertEqual(sum(w.number for w in sim.d.warning),0)

    def test_passage_thickness_matches_viewer(self):
        thickness = .18
        sim = HoLoArmSim(passage_thickness=thickness)
        flight = Flight(passage_thickness=thickness)
        self.assertEqual(sim.flight.passage_thickness, thickness)
        np.testing.assert_array_equal(sim.m.geom_pos, flight.model.geom_pos)
        np.testing.assert_array_equal(sim.m.geom_size, flight.model.geom_size)
        for side in (-1, 1):
            wall = sim.m.geom(f'slot_wall_{side}').id
            self.assertAlmostEqual(2*sim.m.geom_size[wall, 0], thickness)

    def test_launch_has_no_legacy_model(self):
        launch=ET.parse(ROOT/'launch'/'holoarm_sim.launch').getroot()
        params={p.get('name'):p.get('value') for p in launch.findall('node/param')}
        self.assertEqual(params['backend'],'holoarm')
        self.assertNotIn('model_path',params)
        self.assertFalse((ROOT/'holoarm'/'narrow_passage_test.py').exists())
        self.assertFalse((ROOT/'holoarm'/'models'/'holoarm_narrow_passage.xml').exists())


if __name__=='__main__':
    unittest.main()
