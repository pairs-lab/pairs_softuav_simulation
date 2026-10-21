"""Legacy illustrative wall impacts with the original assumed spring mechanics.

This module preserves historical behavior demos; collision_study is the CAD default.

Reference: https://arxiv.org/html/2605.25790v1
This reduced-order model is not the paper's RL policy or identified hardware.
All structural/contact assumptions are exposed in wall_collision_config.json.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

if os.name != 'nt' and "MUJOCO_GL" not in os.environ and not os.environ.get("DISPLAY"):
    os.environ["MUJOCO_GL"] = "egl"

import mujoco
import numpy as np

from rigid_visual import apply_rigid_visual
from soft_visual import apply_soft_visual, add_soft_ligaments
from speed_sweep_config import SPEEDS

HERE = Path(__file__).resolve().parent
CONFIG = HERE / "models" / "legacy_wall_collision_config.json"
ARMS = ("fl", "fr", "rl", "rr")
WALL_X = 0.55
WALL_FRONT = WALL_X - 0.045


def load_config(path=CONFIG):
    cfg = json.loads(Path(path).read_text())
    for name, value in cfg.items():
        if isinstance(value, (float, int)) and (not math.isfinite(value) or value <= 0):
            raise ValueError(f"{name} must be finite and positive")
    return cfg


def build_model(kind, cfg, width=640, height=540, rigid_visual='x2', soft_visual='holoarm', environment=None,
                model_transform=None):
    root = ET.parse(HERE / "models" / "holoarm_physics.xml").getroot()
    root.set("model", f"HoLoArm wall impact - {kind}")
    root.find("compiler").set("settotalmass", str(cfg["mass_kg"]))
    option = root.find("option")
    option.set("timestep", str(cfg["timestep_s"]))
    option.set("integrator", "implicitfast")
    option.set("iterations", "80")
    option.set("cone", "elliptic")
    geom_default = root.find("default/geom")
    geom_default.set("contype", "2")
    geom_default.set("conaffinity", "1")
    geom_default.set("friction", f"{cfg['wall_friction']} 0.01 0.001")
    geom_default.set("solref", f"{cfg['contact_time_constant_s']} {cfg['contact_damping_ratio']}")
    geom_default.set("solimp", "0.95 0.99 0.001")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth=str(width), offheight=str(height))
    ET.SubElement(visual, "headlight", ambient="0.4 0.4 0.4", diffuse="0.7 0.7 0.7")
    world = root.find("worldbody")
    floor = world.find("geom[@name='floor']")
    floor.set("contype", "1")
    floor.set("conaffinity", "2")
    ET.SubElement(world, "geom", name="impact_wall", type="box", pos=f"{WALL_X} 0 1.25",
                  size="0.045 1.2 1.25", rgba="0.63 0.67 0.72 1", contype="1", conaffinity="2")
    # Orange stripe placed behind the contact surface: purely visual.
    stripe = ET.SubElement(world, "geom", type="box", pos=f"{WALL_FRONT-0.001} 0 1.2",
                  size="0.002 1.2 0.012", rgba="1 0.5 0.08 1", contype="0", conaffinity="0")
    if environment is not None:
        world.remove(world.find("geom[@name='impact_wall']"))
        world.remove(stripe)
        for geom in environment:
            world.append(geom)
        root.set("model", "HoLoArm keyboard narrow passage")
    core = world.find("body[@name='holoarm_core']")
    core.set("pos", f"0 0 {cfg['altitude_m']}")
    colour = "0.1 0.62 0.95 1" if kind == "soft" else "1 0.42 0.12 1"
    core.find("geom").set("rgba", colour)
    tendons = ET.SubElement(root,"tendon")
    # Give every geometry a name, making contact attribution unambiguous.
    for i, geom in enumerate(core.iter("geom")):
        geom.set("name", f"drone_geom_{i}")
    for arm in ARMS:
        base = core.find(f"body[@name='{arm}_base']")
        for geom in base.findall("geom"):
            geom.set("contype", "0")
            geom.set("conaffinity", "0")
        for joint in base.findall("joint"):
            pitch = "pitch" in joint.get("name")
            joint.set("solreflimit", "0.001 1")
            joint.set("solimplimit", "0.99 0.999 0.0001")
            joint.set("stiffness", str(cfg['pitch_stiffness_nm_rad'] if pitch else cfg['yaw_stiffness_nm_rad']))
            joint.set("damping", str(cfg['pitch_damping_nms_rad'] if pitch else cfg['yaw_damping_nms_rad']))
            limit = math.radians(cfg['bend_limit_deg'])
            joint.set("range", f"{-limit} {limit}")
        slide = base.find(f"body[@name='{arm}_arm_sliding']/joint")
        slide.set("stiffness", str(cfg['slide_stiffness_n_m']))
        slide.set("damping", str(cfg['slide_damping_ns_m']))
        slide.set("solreflimit", "0.001 1")
        slide.set("solimplimit", "0.99 0.999 0.0001")
        slide.set("range", f"{-cfg['slide_range_m']} {cfg['slide_range_m']}")
        sliding_body = base.find(f"body[@name='{arm}_arm_sliding']")
        nominal_pos = np.fromstring(base.get('pos'),sep=' ')
        nominal_yaw = float(base.get('euler').split()[2])
        rot = np.array([[math.cos(nominal_yaw),-math.sin(nominal_yaw),0],
                        [math.sin(nominal_yaw),math.cos(nominal_yaw),0],[0,0,1]])
        for ligament,(y,z) in enumerate(((.014,0),(-.014,0),(0,.014),(0,-.014))):
            fixed_name, moving_name = f'{arm}_ligament_fixed_{ligament}',f'{arm}_ligament_moving_{ligament}'
            fixed = nominal_pos+rot@np.array([.002,y,z])
            ET.SubElement(core,'site',name=fixed_name,pos=' '.join(map(str,fixed)),size='.001',rgba='0 0 0 0')
            ET.SubElement(sliding_body,'site',name=moving_name,pos=f'.037 {y} {z}',size='.001',rgba='0 0 0 0')
            spatial = ET.SubElement(tendons,'spatial',name=f'{arm}_ligament_{ligament}',width='.0018',rgba='1 .65 .1 1')
            ET.SubElement(spatial,'site',site=fixed_name)
            ET.SubElement(spatial,'site',site=moving_name)
        motor = base.find(f"body[@name='{arm}_arm_sliding']/body[@name='{arm}_motor']")
        motor.findall("geom")[1].set("size", "0.10 0.001")
        for geom in base.iter("geom"):
            if geom.get("type") == "capsule":
                geom.set("rgba", colour)
        # Guards are the first physical contact, as in the HoLoArm experiment.
        radius = cfg['guard_radius_m']
        for segment in range(20):
            a, b = 2*math.pi*segment/20, 2*math.pi*(segment+1)/20
            points = (radius*math.cos(a), radius*math.sin(a), 0,
                      radius*math.cos(b), radius*math.sin(b), 0)
            ET.SubElement(motor, "geom", name=f"{arm}_guard_{segment}", type="capsule",
                          fromto=" ".join(map(str, points)), size="0.003", mass="0.0008", rgba=colour)

        if kind == "rigid":
            # A free arm welded to the body can physically detach after failure.
            for body in base.iter("body"):
                for joint in list(body.findall("joint")):
                    body.remove(joint)
            pos = np.fromstring(base.get("pos"), sep=" ")
            pos[2] += cfg['altitude_m']
            base.set("pos", " ".join(map(str, pos)))
            base.insert(0, ET.Element("freejoint", name=f"{arm}_fracture_free"))
            core.remove(base)
            world.append(base)
    if kind=='rigid':
        # Rigid joints do not have elastic ligaments spanning a detached arm.
        root.remove(tendons)
    if kind == "rigid":
        equality = ET.SubElement(root, "equality")
        for arm in ARMS:
            ET.SubElement(equality, "weld", name=f"{arm}_attachment", body1="holoarm_core",
                          body2=f"{arm}_base", solref="0.001 1", solimp="0.99 0.999 0.001", torquescale="0.15")
    elif kind != "soft":
        raise ValueError("kind must be soft or rigid")
    if kind=='rigid' and rigid_visual=='x2':
        apply_rigid_visual(root)
    elif rigid_visual not in ('x2','legacy'):
        raise ValueError('rigid_visual must be x2 or legacy')
    if kind=='soft' and soft_visual=='holoarm':
        apply_soft_visual(root)
    elif soft_visual not in ('holoarm','legacy'):
        raise ValueError('soft_visual must be holoarm or legacy')
    if model_transform is not None:
        model_transform(root)
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))


def tilt_deg(data, core):
    # Actual angle of the body z-axis to gravity, including upside-down poses.
    return math.degrees(math.acos(float(np.clip(data.xmat[core].reshape(3,3)[2,2], -1, 1))))


class Trial:
    def __init__(self, kind, speed, cfg, fracture=True, timestep=None, model_transform=None):
        self.kind, self.speed, self.cfg = kind, speed, dict(cfg)
        if timestep is not None:
            self.cfg['timestep_s'] = timestep
        self.model = build_model(kind, self.cfg, model_transform=model_transform)
        self.data = mujoco.MjData(self.model)
        self.core = self.model.body('holoarm_core').id
        self.wall = self.model.geom('impact_wall').id
        self.floor = self.model.geom('floor').id
        self.sites = [self.model.site(f'thrust_{a}').id for a in ARMS]
        self.arm_roots = [self.model.body(f'{a}_base').id for a in ARMS]
        self.geom_arm = np.full(self.model.ngeom, -1, dtype=int)
        self.body_arm = np.full(self.model.nbody, -1, dtype=int)
        for b in range(1,self.model.nbody):
            body = b
            while body > 0:
                if body in self.arm_roots:
                    self.body_arm[b] = self.arm_roots.index(body)
                    break
                body = self.model.body_parentid[body]
        for g in range(self.model.ngeom):
            body = self.model.geom_bodyid[g]
            while body > 0:
                if body in self.arm_roots:
                    self.geom_arm[g] = self.arm_roots.index(body)
                    break
                body = self.model.body_parentid[body]
        self.broken = np.zeros(4, dtype=bool)
        self.failure_times = []
        self.fracture_enabled = fracture
        self.thrust = np.full(4, self.cfg['mass_kg']*9.81/4)
        self.recovery_target = np.array([-0.25, 0., self.cfg['altitude_m']])
        self.impact_time = None
        self.actual_impact_speed = None
        self.ground_time = None
        self.last_contact_time = None
        self.recovery_start = None
        self.recovery_time = None
        self.peak_force = self.peak_accel = self.peak_tilt = self.peak_bend = self.peak_slide = 0.
        self.peak_arm_moment = self.peak_arm_force = 0.
        self.max_slide_limit_violation_mm = 0.
        self.impulse = 0.
        self.history = []
        self.trace = []
        self.nominal_bend = np.zeros(8)
        self.bend_indices = [self.model.joint(f'{a}_bend_{axis}').qposadr[0]
                             for a in ARMS for axis in ('pitch','yaw')] if kind == 'soft' else []
        self.slide_indices = [self.model.joint(f'{a}_slide').qposadr[0] for a in ARMS] if kind == 'soft' else []
        mujoco.mj_forward(self.model, self.data)

    def controller(self, settling=False):
        d, m, cfg = self.data, self.model, self.cfg
        R = d.xmat[self.core].reshape(3,3)
        pos, vel = d.qpos[:3], d.qvel[:3]
        acc = np.zeros(3)
        if settling:
            acc[:2] = -4*pos[:2] - 3*vel[:2]
        elif self.impact_time is None:
            # Maintain the requested approach speed; no pre-contact braking.
            acc[0] = 2*(self.speed-vel[0])
            acc[1] = -4*pos[1] - 3*vel[1]
        else:
            acc[:2] = 5*(self.recovery_target[:2]-pos[:2]) - 4*vel[:2]
        acc[:2] = np.clip(acc[:2], -5, 5)
        acc[2] = 8*(cfg['altitude_m']-pos[2])-4*vel[2]
        attached = np.array([b for b in range(1,m.nbody)
                             if self.body_arm[b]<0 or not self.broken[self.body_arm[b]]])
        mass = float(m.body_mass[attached].sum())
        com = np.sum(d.xipos[attached]*m.body_mass[attached,None],axis=0)/mass
        desired_force = mass*(acc+np.array([0.,0.,9.81]))
        z = desired_force / max(np.linalg.norm(desired_force), 1e-9)
        y = np.cross(z, np.array([1.,0.,0.]))
        y /= max(np.linalg.norm(y),1e-9)
        Rd = np.column_stack((np.cross(y,z), y,z))
        reference_hook = getattr(self, 'recovery_attitude_reference', None)
        if reference_hook is not None:
            Rd = reference_hook(Rd, settling)
        skew = 0.5*(Rd.T@R-R.T@Rd)
        attitude_error = np.array([skew[2,1],skew[0,2],skew[1,0]])
        omega = d.qvel[3:6]
        torque = -np.array([.75,.75,.28])*attitude_error-np.array([.12,.12,.06])*omega
        # Allocate from actual motor positions and thrust axes, so folded arms
        # do not use the fixed X-quad mixing matrix from the old demo.
        A = np.zeros((4,4))
        for i, site in enumerate(self.sites):
            direction = d.site_xmat[site].reshape(3,3)[:,2]
            r = d.site_xpos[site]-com
            moment_world = np.cross(r,direction)+m.actuator_gear[i,5]*direction
            A[0,i] = (R.T@direction)[2]
            A[1:,i] = R.T@moment_world
        collective = max(0.,float(desired_force@R[:,2]))
        # Failed motors cannot contribute; allocate against remaining rotors.
        alive = np.where(~self.broken)[0]
        command = np.zeros(4)
        if len(alive):
            command[alive] = np.linalg.lstsq(A[:,alive], np.r_[collective,torque], rcond=None)[0]
        command = np.clip(command,0,cfg['max_motor_thrust_n'])
        alpha = 1-math.exp(-m.opt.timestep/cfg['motor_time_constant_s'])
        self.thrust += alpha*(command-self.thrust)
        self.thrust[self.broken] = 0
        d.ctrl[:] = self.thrust

    def contacts(self):
        d, m = self.data, self.model
        forces, moments = np.zeros((4,3)), np.zeros((4,3))
        total = np.zeros(3)
        force6 = np.zeros(6)
        ground = False
        for i in range(d.ncon):
            c = d.contact[i]
            if self.floor in (c.geom1,c.geom2):
                ground = True
            if self.wall not in (c.geom1,c.geom2):
                continue
            mujoco.mj_contactForce(m,d,i,force6)
            # Contact frame rows are world-coordinate normal/tangents. The
            # returned force acts on geom2; negate when the drone is geom1.
            f = c.frame.reshape(3,3).T@force6[:3]
            drone_geom = c.geom2 if c.geom1 == self.wall else c.geom1
            if c.geom2 == self.wall:
                f = -f
            total += f
            arm = self.geom_arm[drone_geom]
            if arm >= 0 and not self.broken[arm]:
                forces[arm] += f
                moments[arm] += np.cross(c.pos-d.xpos[self.arm_roots[arm]], f)
        return float(np.linalg.norm(total)), np.linalg.norm(forces,axis=1), np.linalg.norm(moments,axis=1), ground

    def fracture(self, forces, moments):
        if self.kind != 'rigid' or not self.fracture_enabled:
            return
        cfg = self.cfg
        for arm in range(4):
            if not self.broken[arm] and (moments[arm] > cfg['rigid_failure_moment_nm'] or
                                        forces[arm] > cfg['rigid_failure_force_n']):
                self.broken[arm] = True
                self.failure_times.append(float(self.data.time))
                self.data.eq_active[self.model.equality(f'{ARMS[arm]}_attachment').id] = False
                # The detached arm retains its mass/velocity; its motor stops.
                self.data.ctrl[arm] = self.thrust[arm] = 0

    def step(self, settling=False):
        d, m, cfg = self.data, self.model, self.cfg
        previous_vx = float(d.qvel[0])
        self.controller(settling)
        # mj_step leaves contact locations and cached xpos at the solved
        # pre-integration state. Read their forces before the next mj_forward.
        mujoco.mj_step(m,d)
        horizontal_accel = abs(float(d.qacc[0]))
        wall_force, forces, moments, ground = self.contacts()
        if settling:
            return
        if wall_force > .01 and self.impact_time is None:
            self.impact_time = float(d.time)
            self.actual_impact_speed = previous_vx
        if wall_force > .01:
            self.last_contact_time = float(d.time)
        if ground and self.ground_time is None:
            self.ground_time = float(d.time)
        self.fracture(forces,moments)
        mujoco.mj_forward(m,d)
        tilt = tilt_deg(d,self.core)
        bend = float(np.max(np.abs(d.qpos[self.bend_indices]-self.nominal_bend))) if self.bend_indices else 0.
        slide = float(np.max(np.abs(d.qpos[self.slide_indices]))) if self.slide_indices else 0.
        # Record first impact separately from a later floor strike or detached
        # arm hitting the wall, which would otherwise corrupt the peak metric.
        first_impact = self.impact_time is not None and d.time-self.impact_time <= .25
        if first_impact:
            self.peak_force = max(self.peak_force,wall_force)
            self.peak_accel = max(self.peak_accel, horizontal_accel)
            self.peak_arm_force = max(self.peak_arm_force,float(max(forces)))
            self.peak_arm_moment = max(self.peak_arm_moment,float(max(moments)))
            self.impulse += wall_force*m.opt.timestep
        if self.impact_time is not None:
            self.peak_tilt = max(self.peak_tilt,tilt)
            self.peak_bend = max(self.peak_bend,math.degrees(bend))
            self.peak_slide = max(self.peak_slide,slide*1000)
            self.max_slide_limit_violation_mm = max(
                self.max_slide_limit_violation_mm,max(0.,slide-cfg['slide_range_m'])*1000)
            stable = (not self.broken.any() and self.ground_time is None and wall_force < .01 and
                      np.linalg.norm(d.qpos[:3]-self.recovery_target) < cfg['recovery_position_tolerance_m'] and
                      np.linalg.norm(d.qvel[:3]) < cfg['recovery_speed_tolerance_m_s'] and
                      tilt < cfg['recovery_tilt_tolerance_deg'])
            if stable:
                if self.recovery_start is None:
                    self.recovery_start = float(d.time)
                if d.time-self.recovery_start >= cfg['recovery_hold_s'] and self.recovery_time is None:
                    self.recovery_time = self.recovery_start-self.impact_time
            else:
                self.recovery_start = None
        self.history.append((d.qpos.copy(),d.qvel.copy(),d.eq_active.copy(),self.broken.copy()))
        if len(self.history)%10 == 0:
            self.trace.append((float(d.time),*d.qpos[:3],*d.qvel[:3],tilt,math.degrees(bend),slide*1000,
                               wall_force,float(max(moments)),int(self.broken.sum())))

    def run(self):
        for _ in range(round(self.cfg['settle_s']/self.model.opt.timestep)):
            self.step(settling=True)
        if self.bend_indices:
            self.nominal_bend = self.data.qpos[self.bend_indices].copy()
        # Position both settled assemblies the same distance from first contact
        # and inject velocity into every free body (including welded arms).
        leading = max(self.data.site_xpos[s,0] for s in self.sites)+self.cfg['guard_radius_m']+.003
        shift = WALL_FRONT-leading-self.speed*self.cfg['approach_s']
        for j in range(self.model.njnt):
            if self.model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
                q, v = self.model.jnt_qposadr[j], self.model.jnt_dofadr[j]
                self.data.qpos[q] += shift
                self.data.qvel[v] = self.speed
        self.data.time = 0
        mujoco.mj_forward(self.model,self.data)
        for _ in range(round((self.cfg['approach_s']+self.cfg['post_impact_s'])/self.model.opt.timestep)):
            self.step()
        return self

    def summary(self):
        # A sensor bandwidth must be stated when comparing shock peaks. Retain
        # unfiltered solver peaks as well as a 10 ms mean body deceleration.
        vx = np.array([sample[1][0] for sample in self.history])
        acc = -np.diff(vx)/self.model.opt.timestep
        samples = max(1,round(.01/self.model.opt.timestep))
        averaged = np.convolve(acc,np.ones(samples)/samples,mode='valid')
        start = max(0,round((self.impact_time or 0)/self.model.opt.timestep)-samples)
        stop = min(len(averaged),start+round(.25/self.model.opt.timestep))
        filtered_peak = float(max(0.,np.max(averaged[start:stop]))) if stop>start else 0.
        return dict(kind=self.kind, requested_speed_m_s=self.speed, actual_impact_speed_m_s=self.actual_impact_speed,
                    impact_time_s=self.impact_time, peak_wall_force_n=self.peak_force,
                    peak_horizontal_acceleration_m_s2=self.peak_accel, first_impact_impulse_ns=self.impulse,
                    peak_10ms_mean_body_deceleration_m_s2=filtered_peak,
                    peak_arm_contact_moment_nm=self.peak_arm_moment, peak_arm_contact_force_n=self.peak_arm_force,
                    peak_tilt_deg=self.peak_tilt, peak_bend_deg=self.peak_bend, peak_compression_mm=self.peak_slide,
                    max_slide_limit_violation_mm=self.max_slide_limit_violation_mm,
                    fracture_enabled=self.fracture_enabled,
                    broken_arms=int(self.broken.sum()), failure_times_s=self.failure_times,
                    ground_contact_time_s=self.ground_time, recovered=self.recovery_time is not None,
                    recovery_time_s=self.recovery_time, final_position_m=self.data.qpos[:3].tolist(),
                    final_speed_m_s=float(np.linalg.norm(self.data.qvel[:3])))


def camera(close=False):
    c = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(c)
    c.lookat[:] = (-.05,0,1.15) if close else (-.8,0,1.0)
    c.distance = 1.5 if close else 4.5
    c.azimuth = -90
    c.elevation = -85 if close else -20
    return c


def render_video(trials, output, width=640, height=540, fps=30, show_height_loss=False,
                 slow_motion_window_s=.26, close_follow=False, show_yaw_motion=False,
                 overwrite=False):
    from imageio_ffmpeg import get_ffmpeg_exe
    from PIL import Image, ImageDraw, ImageFont
    ffmpeg = get_ffmpeg_exe()
    renderers = []
    saved_states = [(t.data.qpos.copy(),t.data.qvel.copy(),t.data.eq_active.copy()) for t in trials]
    try:
        for t in trials:
            t.model.vis.global_.offwidth = width
            t.model.vis.global_.offheight = height
            renderers.append(mujoco.Renderer(t.model,height=height,width=width))
        font_path = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
        font = ImageFont.truetype(font_path,max(14,round(width/30)))
        small = ImageFont.truetype(font_path,max(12,round(width/40)))
        end = min(len(t.history)*t.model.opt.timestep for t in trials)
        impact = min(t.impact_time for t in trials if t.impact_time is not None)
        slow_start, slow_end = max(0,impact-.04), min(end,impact+slow_motion_window_s)
        # Identical time axis in both panels. Slow motion is explicit and does
        # not alter physics or conceal the actual impact velocity.
        times = np.r_[np.arange(0,slow_start,1/fps),
                      np.arange(slow_start,slow_end,.15/fps),
                      np.arange(slow_end,end,1/fps)]
        output = Path(output)
        output.parent.mkdir(parents=True,exist_ok=True)
        command = [ffmpeg,'-hide_banner','-loglevel','error','-y' if overwrite else '-n',
                   '-f','rawvideo','-pix_fmt','rgb24',
                   '-s',f'{width*2}x{height}','-r',str(fps),'-i','-','-an','-c:v','libx264',
                   '-preset','fast','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(output)]
        process = subprocess.Popen(command,stdin=subprocess.PIPE)
        try:
            for time in times:
                panels = []
                for t, renderer in zip(trials,renderers):
                    index = min(len(t.history)-1,round(time/t.model.opt.timestep))
                    q,v,eq,broken = t.history[index]
                    t.data.qpos[:],t.data.qvel[:] = q,v
                    t.data.eq_active[:] = eq
                    mujoco.mj_forward(t.model,t.data)
                    renderer.update_scene(t.data,camera=camera())
                    if t.kind=='soft':
                        add_soft_ligaments(renderer.scene,t.model,t.data)
                    panel = Image.fromarray(renderer.render())
                    close_camera = camera(close=True)
                    if close_follow:
                        close_camera.lookat[:] = t.data.qpos[:3]
                        close_camera.distance = 1.2
                    renderer.update_scene(t.data,camera=close_camera)
                    if t.kind=='soft':
                        add_soft_ligaments(renderer.scene,t.model,t.data)
                    close = Image.fromarray(renderer.render()).resize((width//2,height//2))
                    panel.paste(close,(width-width//2,height-height//2))
                    draw = ImageDraw.Draw(panel)
                    draw.rectangle((0,0,width,102),fill=(15,21,30))
                    colour = (70,190,255) if t.kind=='soft' else (255,145,70)
                    title = 'SOFT / HoLoArm' if t.kind=='soft' else 'RIGID / X2-style'
                    draw.text((12,8),f'{title}  |  {t.speed:g} m/s',font=font,fill=colour)
                    if broken.any():
                        state = f'FRACTURE: {broken.sum()} arm(s) detached'
                    elif t.ground_time is not None and time>=t.ground_time:
                        state = 'GROUND CONTACT: recovery test failed'
                    elif t.impact_time is None or time<t.impact_time:
                        state = 'APPROACH'
                    elif t.recovery_time is not None and time >= t.impact_time+t.recovery_time+.3:
                        state = 'RECOVERED: stable hover'
                    else:
                        state = 'IMPACT / RECOVERY'
                    draw.text((12,38),state,font=small,fill='white')
                    bend = math.degrees(float(np.max(np.abs(q[t.bend_indices]-t.nominal_bend)))) if t.bend_indices else 0
                    detail = f'Arm bend {bend:.1f} deg  |  tilt {tilt_deg(t.data,t.core):.1f} deg'
                    if show_height_loss:
                        detail += f'  |  drop {max(0.,t.cfg["altitude_m"]-q[2])*100:.1f} cm'
                    if show_yaw_motion:
                        R = t.data.xmat[t.core].reshape(3,3)
                        heading = getattr(t,'heading',0.)
                        yaw = math.atan2(R[1,0],R[0,0])-heading
                        yaw = math.degrees(math.atan2(math.sin(yaw),math.cos(yaw)))
                        detail = f'Arm {bend:.0f} deg | tilt {tilt_deg(t.data,t.core):.0f} deg | yaw {yaw:+.0f} deg'
                        detail += f' | drop {max(0.,t.cfg["altitude_m"]-q[2])*100:.0f} cm'
                    draw.text((12,62),detail,font=small,fill='white')
                    playback = '0.15x SLOW MOTION' if slow_start <= time < slow_end else '1x'
                    draw.text((12,81),f't={time:.3f}s  |  {playback}',font=small,fill=(255,220,90))
                    panels.append(np.asarray(panel))
                process.stdin.write(np.concatenate(panels,axis=1).tobytes())
        finally:
            process.stdin.close()
            code = process.wait()
        if code:
            raise RuntimeError(f'ffmpeg failed: {code}')
    finally:
        for renderer in renderers:
            renderer.close()
        for t,(q,v,eq) in zip(trials,saved_states):
            t.data.qpos[:],t.data.qvel[:],t.data.eq_active[:] = q,v,eq
            mujoco.mj_forward(t.model,t.data)


def write_results(trials, directory, cfg):
    directory = Path(directory)
    directory.mkdir(parents=True,exist_ok=True)
    summaries = [t.summary() for t in trials]
    (directory/'results.json').write_text(json.dumps(dict(config=cfg,visual=dict(
        soft='Original HoLoArm-inspired white frame/guards, exposed electronics and curved orange ligaments',
        rigid='Original X2-inspired artwork; not a Skydio X2 dynamics model',
        physics='Same mass, inertia, contact geometry and fracture settings as the matched rigid baseline'),
        results=summaries),indent=2))
    with (directory/'summary.csv').open('w',newline='') as f:
        writer = csv.DictWriter(f,fieldnames=list(summaries[0]))
        writer.writeheader()
        writer.writerows(summaries)
    for t in trials:
        with (directory/f'{t.kind}_{t.speed:g}ms_trace.csv').open('w',newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time_s','x_m','y_m','z_m','vx_m_s','vy_m_s','vz_m_s','tilt_deg',
                             'arm_bend_deg','arm_compression_mm','wall_force_n','arm_contact_moment_nm','broken_arms'])
            writer.writerows(t.trace)
    return summaries


def write_report(summaries,directory,cfg):
    lines = [
        '# Khảo sát va tường: soft và rigid drone', '',
        'Mô hình cơ học rút gọn, bộ điều khiển PD giống nhau; không phải policy RL hay digital twin đã hiệu chuẩn.', '',
        f"Khối lượng {cfg['mass_kg']} kg; bước vật lý {cfg['timestep_s']} s; đáp ứng động cơ {cfg['motor_time_constant_s']} s.",
        f"Ngưỡng gãy rigid giả định: mô-men tiếp xúc > {cfg['rigid_failure_moment_nm']} Nm hoặc lực > {cfg['rigid_failure_force_n']} N tại một tay.", '',
        '| Tốc độ đặt (m/s) | Loại | Biến dạng tay (°) | Nghiêng thân tối đa (°) | Tay tách | Phục hồi hover (s) | Chạm sàn |',
        '|---:|:---|---:|---:|---:|---:|:---|'
    ]
    for r in summaries:
        recovered = f"{r['recovery_time_s']:.3f}" if r['recovery_time_s'] is not None else 'Không'
        ground = 'Có' if r['ground_contact_time_s'] is not None else 'Không'
        lines.append(f"| {r['requested_speed_m_s']:g} | {r['kind']} | {r['peak_bend_deg']:.1f} | "
                     f"{r['peak_tilt_deg']:.1f} | {r['broken_arms']} | {recovered} | {ground} |")
    lines += ['',
              'Thời gian phục hồi tính từ tiếp xúc đầu tiên tới đầu khoảng ổn định kéo dài 0,3 s. Điều kiện: cách mục tiêu dưới 12 cm, tốc độ dưới 0,25 m/s, nghiêng dưới 8°, không tiếp xúc tường/sàn hoặc tách tay.', '',
              'Xem summary.csv/results.json để đối chiếu vận tốc va chạm thực, lực đỉnh chưa lọc, gia tốc đỉnh chưa lọc, gia tốc giảm tốc trung bình 10 ms, xung lực và độ vượt giới hạn trượt.', '',
              '**Giới hạn:** mô hình soft chưa tái hiện lợi thế về đỉnh lực tức thời của thực nghiệm. Các chặn hành trình trượt có thể tạo xung lực rất ngắn; độ vượt giới hạn được ghi riêng. Không dùng kết quả này làm ngưỡng an toàn vật liệu hoặc khẳng định HoLoArm thật sẽ sống sót khi đâm tường 8 m/s. Soft không có mô hình phá hủy vật liệu; rigid chỉ gãy theo các ngưỡng giả định ở trên.', '',
              '[Bài báo HoLoArm](https://arxiv.org/html/2605.25790v1) và [video tham chiếu](https://www.youtube.com/watch?v=hmfgFP9XoHQ) phân biệt phục hồi khi bay với thử thả rơi 3 m. Kết quả tốc độ cao trong thử thả rơi không xác định trực tiếp ngưỡng gãy khi bay đâm tường.', '',
              'Chi tiết phương pháp: ../../holoarm/WALL_COLLISION_STUDY.md', '']
    (Path(directory)/'REPORT.md').write_text('\n'.join(lines))


def plot_results(summaries,directory):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes = plt.subplots(3,2,figsize=(11,10),layout='constrained')
    plots = [('peak_wall_force_n','First impact peak wall force (N)'),
             ('peak_bend_deg','Peak arm angular deformation (deg)'),
             ('peak_tilt_deg','Peak body tilt after impact (deg)'),
             ('broken_arms','Detached arms (assumed fracture limits)'),
             ('peak_10ms_mean_body_deceleration_m_s2','10 ms mean body deceleration peak (m/s2)'),
             ('recovery_time_s','Hover recovery time (s); missing = failed')]
    for kind,colour in [('soft','#168eca'),('rigid','#ec752a')]:
        rows = [r for r in summaries if r['kind']==kind]
        for ax,(key,label) in zip(axes.flat,plots):
            ax.plot([r['actual_impact_speed_m_s'] for r in rows],
                    [np.nan if r[key] is None else r[key] for r in rows],'-o',label=kind,color=colour,ms=4)
            ax.set(xlabel='Measured impact speed (m/s)',ylabel=label)
            ax.grid(alpha=.25)
            ax.legend()
    fig.suptitle('Wall impact sweep: assumed mechanics, same PD controller, 40 ms motor response')
    fig.savefig(Path(directory)/'speed_sweep.png',dpi=180)
    plt.close(fig)


def combine_videos(paths,directory,filename='speed_sweep_comparison.mp4',overwrite=False):
    """Join compatible survey clips without re-encoding them."""
    from imageio_ffmpeg import get_ffmpeg_exe
    directory = Path(directory)
    manifest = directory/'video_list.ffconcat'
    if any(path.resolve().parent != directory.resolve() for path in paths):
        raise ValueError('Survey clips must reside in the results directory')
    manifest.write_text('ffconcat version 1.0\n'+''.join(f"file '{path.name}'\n" for path in paths))
    output = directory/filename
    subprocess.run([get_ffmpeg_exe(),'-hide_banner','-loglevel','error','-y' if overwrite else '-n',
                    '-f','concat','-safe','1',
                    '-i',str(manifest),'-c','copy','-movflags','+faststart',str(output)],check=True)
    return output


def cli():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=CONFIG)
    p.add_argument('--impact-speed',type=float,default=2.)
    p.add_argument('--sweep',action='store_true',help='2..7 m/s every 0.8 m/s, plus 7.6 m/s (12 cases)')
    p.add_argument('--speeds',type=float,nargs='+',help='custom survey speeds')
    p.add_argument('--output',type=Path,default=HERE.parent/'outputs'/'wall_collision_recovery.mp4')
    p.add_argument('--results-dir',type=Path,default=HERE.parent/'outputs'/'wall_collision_study')
    p.add_argument('--no-video',action='store_true')
    p.add_argument('--overwrite',action='store_true',help='replace existing target videos and regenerate results')
    p.add_argument('--no-fracture',action='store_true',help='unbreakable rigid control case')
    p.add_argument('--duration',type=float,help='simulated seconds after first expected impact')
    p.add_argument('--width',type=int,default=640)
    p.add_argument('--height',type=int,default=540)
    p.add_argument('--fps',type=int,default=30)
    args = p.parse_args()
    cfg = load_config(args.config)
    if args.duration is not None:
        if not math.isfinite(args.duration) or args.duration <= 0:
            p.error('--duration must be positive and finite')
        cfg['post_impact_s'] = args.duration
    speeds = args.speeds or (list(SPEEDS) if args.sweep else [args.impact_speed])
    if any(not math.isfinite(s) or s<=0 for s in speeds):
        p.error('speeds must be positive and finite')
    if len(set(speeds)) != len(speeds):
        p.error('survey speeds must be unique')
    if args.width<320 or args.height<240 or args.width%2 or args.height%2 or not 1<=args.fps<=120:
        p.error('use even dimensions >=320x240 and fps in 1..120')
    paths = ([args.results_dir/f'comparison_{s:g}ms.mp4' for s in speeds]
             if args.sweep or args.speeds else [args.output])
    if not args.no_video and not args.overwrite and any(path.exists() for path in paths):
        p.error('a target video already exists; use --overwrite or select a new output/results directory')
    if len(speeds)>1 and not args.no_video and not args.overwrite and (args.results_dir/'speed_sweep_comparison.mp4').exists():
        p.error('the combined video already exists; use --overwrite or select a new results directory')
    all_trials = []
    for speed,path in zip(speeds,paths):
        trials = [Trial(kind,speed,cfg,fracture=not args.no_fracture).run() for kind in ('soft','rigid')]
        all_trials.extend(trials)
        for t in trials:
            s=t.summary()
            print(f"{speed:g} m/s {t.kind}: actual={s['actual_impact_speed_m_s']:.3f}, "
                  f"bend={s['peak_bend_deg']:.1f} deg, tilt={s['peak_tilt_deg']:.1f} deg, "
                  f"broken={s['broken_arms']}, recovery={s['recovery_time_s']}",flush=True)
        if not args.no_video:
            render_video(trials,path,args.width,args.height,args.fps,overwrite=args.overwrite)
    summaries=write_results(all_trials,args.results_dir,cfg)
    write_report(summaries,args.results_dir,cfg)
    if len(speeds)>1:
        plot_results(summaries,args.results_dir)
        if not args.no_video:
            print(f'Combined video: {combine_videos(paths,args.results_dir,overwrite=args.overwrite).resolve()}')
    print(f'Results: {args.results_dir.resolve()}')
    return 0
