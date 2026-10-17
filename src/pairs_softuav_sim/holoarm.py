"""ROS adapter for the same HoLoArm Flight used by the keyboard viewer.

No alternate PID, collision geometry or behavior-video maneuvers are added.
"""
from pathlib import Path
import sys

import mujoco


class HoLoArmSim:
    def __init__(self, holoarm_dir=None, target=(0.,0.,1.2), gap=.46, speed=.6,
                 passage_thickness=None):
        # catkin installs the self-contained scene alongside launch files.
        scene_dir=Path(holoarm_dir) if holoarm_dir else Path(__file__).resolve().parents[2]/'holoarm'
        if not (scene_dir/'holoarm_narrow_passage_viewer.py').is_file():
            raise FileNotFoundError(f'HoLoArm scene not found: {scene_dir}')
        if str(scene_dir) not in sys.path:
            sys.path.insert(0,str(scene_dir))
        from holoarm_narrow_passage_viewer import Flight
        self.flight=Flight(gap=gap,speed=speed,passage_thickness=passage_thickness)
        self.m,self.d=self.flight.model,self.flight.data
        self.reset_pose(*target)

    @property
    def timestep(self):
        return self.m.opt.timestep

    def reset_pose(self,x,y,z):
        self.flight.reset()
        self.d.qpos[:3]=[x,y,z]
        mujoco.mj_forward(self.m,self.d)
        self.flight.hold()

    def set_target(self,x,y,z):
        self.flight.target[:]=[x,y,z]
        self.flight.clamp_target()

    def step(self):
        self.flight.step()

    def get_state(self):
        return dict(position=self.d.qpos[:3].copy(),orientation=self.d.qpos[3:7].copy(),
                    linear_velocity=self.d.qvel[:3].copy(),
                    angular_velocity=self.d.qvel[3:6].copy(),time=float(self.d.time))

    def add_visuals(self,scene):
        """Add legacy ligament artwork; CAD skins already belong to the model."""
        from soft_visual import add_soft_ligaments
        add_soft_ligaments(scene,self.m,self.d)
