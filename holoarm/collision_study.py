"""Wall impacts with the supplied HoLoArm CAD and frozen bench material model.

The default rigid reference uses the previous hard-arm / X2-style model.
The same CAD with locked arms remains an explicit alternate reference.
Neutral-hover preload, flight control and ideal-slip wall contacts are assumptions.
Historical illustrative mechanics live separately in legacy_collision_study.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
import subprocess

if os.name != 'nt' and "MUJOCO_GL" not in os.environ and not os.environ.get("DISPLAY"):
    os.environ["MUJOCO_GL"] = "egl"

import mujoco
import numpy as np

from soft_visual import add_soft_ligaments
from cad_model import build_cad_model, CADMaterial, patch_config, provenance as cad_provenance, load_scene

HERE = Path(__file__).resolve().parent
CONFIG = HERE / "models" / "wall_collision_config.json"
ARMS = ("fl", "fr", "rl", "rr")
WALL_FRONT = load_scene()['impact_wall']['front_x_m']
WALL_X = WALL_FRONT + load_scene()['impact_wall']['thickness_m']/2
SPEEDS = (.3, .6, .9)
MODEL_LABELS = {'soft-cad': 'Soft HoLoArm CAD', 'hard-arm': 'Hard-arm / X2-style',
                'locked-cad': 'Locked HoLoArm CAD'}
RIGID_DESCRIPTIONS = {
    'hard-arm': 'Previous hard-arm geometry and X2-style artwork, with its own rotor layout and inertia; '
                'this is an illustrative rigid model, not calibrated Skydio X2 hardware.',
    'locked-cad': 'The same HoLoArm CAD with its arms locked: a counterfactual reference.'}


def load_config(path=CONFIG):
    cfg = json.loads(Path(path).read_text())
    for name, value in cfg.items():
        if isinstance(value, (float, int)) and (not math.isfinite(value) or value <= 0):
            raise ValueError(f"{name} must be finite and positive")
    return patch_config(cfg)


def build_model(kind, cfg, width=640, height=540, rigid_visual='x2', soft_visual='holoarm', environment=None,
                model_transform=None):
    """Build compliant CAD or the selected rigid reference."""
    variant = model_variant(kind, cfg)
    if variant == 'hard-arm':
        from hard_arm_model import build_hard_arm_model
        return build_hard_arm_model(cfg, width, height, environment, model_transform)
    return build_cad_model(kind,cfg,width,height,environment,model_transform)


def model_variant(kind, cfg):
    if kind == 'soft':
        return 'soft-cad'
    if kind != 'rigid':
        raise ValueError('kind must be soft or rigid')
    variant = cfg.get('rigid_model', 'hard-arm')
    if variant not in ('hard-arm', 'locked-cad'):
        raise ValueError('rigid_model must be hard-arm or locked-cad')
    return variant


def model_provenance(variant, cfg):
    if variant == 'hard-arm':
        from hard_arm_model import hard_arm_provenance
        return hard_arm_provenance(cfg)
    result = cad_provenance()
    result['reference'] = 'compliant CAD' if variant == 'soft-cad' else 'same CAD with arms locked'
    return result


def contact_front_x(model, data, wall):
    """Exact world-X support of primitives eligible to collide with the wall."""
    fronts = []
    for geom in range(model.ngeom):
        if model.geom_bodyid[geom] == 0:
            continue
        if not ((model.geom_contype[geom] & model.geom_conaffinity[wall]) or
                (model.geom_contype[wall] & model.geom_conaffinity[geom])):
            continue
        kind = model.geom_type[geom]
        size = model.geom_size[geom]
        row = data.geom_xmat[geom].reshape(3, 3)[0]
        if kind == mujoco.mjtGeom.mjGEOM_SPHERE:
            extent = size[0]
        elif kind == mujoco.mjtGeom.mjGEOM_CAPSULE:
            extent = size[0]+abs(row[2])*size[1]
        elif kind == mujoco.mjtGeom.mjGEOM_CYLINDER:
            extent = size[0]*np.linalg.norm(row[:2])+abs(row[2])*size[1]
        elif kind == mujoco.mjtGeom.mjGEOM_BOX:
            extent = np.abs(row)@size
        elif kind == mujoco.mjtGeom.mjGEOM_ELLIPSOID:
            extent = np.linalg.norm(row*size)
        else:
            raise ValueError(f'Unsupported wall-contact geometry: {model.geom(geom).name}')
        fronts.append(float(data.geom_xpos[geom, 0]+extent))
    if not fronts:
        raise ValueError('The vehicle has no geometry eligible for wall contact')
    return max(fronts)


def tilt_deg(data, core):
    # Actual angle of the body z-axis to gravity, including upside-down poses.
    return math.degrees(math.acos(float(np.clip(data.xmat[core].reshape(3,3)[2,2], -1, 1))))


class Trial:
    def __init__(self, kind, speed, cfg, fracture=False, timestep=None, model_transform=None):
        self.kind, self.speed, self.cfg = kind, speed, dict(cfg)
        if timestep is not None:
            self.cfg['timestep_s'] = timestep
        self.model_variant = model_variant(kind, self.cfg)
        self.model_label = MODEL_LABELS[self.model_variant]
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
        self.peak_axial_compression_mm = 0.
        self.joint_guard_reached = False
        self.impulse = 0.
        self.history = []
        self.trace = []
        self.nominal_bend = np.zeros(8)
        self.bend_indices = [self.model.joint(f'{a}_bend_{axis}').qposadr[0]
                             for a in ARMS for axis in ('pitch','yaw')] if kind == 'soft' else []
        self.slide_indices = [self.model.joint(f'{a}_slide').qposadr[0] for a in ARMS] if kind == 'soft' else []
        self.slide_ranges = np.array([self.model.joint(f'{a}_slide').range for a in ARMS]) if kind == 'soft' else []
        self.bend_ranges = np.array([self.model.joint(f'{a}_bend_{axis}').range
                                    for a in ARMS for axis in ('pitch','yaw')]) if kind == 'soft' else []
        mujoco.mj_forward(self.model, self.data)
        self.material = CADMaterial(self.model,self.data) if kind=='soft' else None

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
        if self.material:self.material.before_step()
        # mj_step leaves contact locations and cached xpos at the solved
        # pre-integration state. Read their forces before the next mj_forward.
        mujoco.mj_step(m,d)
        if self.material:self.material.after_step()
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
            if self.slide_indices:
                values=d.qpos[self.slide_indices]
                ranges=self.slide_ranges
                violation=np.max(np.maximum(np.maximum(ranges[:,0]-values,values-ranges[:,1]),0))
                self.max_slide_limit_violation_mm=max(self.max_slide_limit_violation_mm,float(violation)*1000)
                self.peak_axial_compression_mm=max(self.peak_axial_compression_mm,float(np.max(values))*1000)
                angles=d.qpos[self.bend_indices]
                self.joint_guard_reached |= bool(
                    np.any(values <= ranges[:,0]+.00002) or np.any(values >= ranges[:,1]-.00002) or
                    np.any(angles <= self.bend_ranges[:,0]+.01) or np.any(angles >= self.bend_ranges[:,1]-.01))
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
            compression=max(0.,float(np.max(d.qpos[self.slide_indices]))) if self.slide_indices else 0.
            self.trace.append((float(d.time),*d.qpos[:3],*d.qvel[:3],tilt,math.degrees(bend),compression*1000,slide*1000,
                               wall_force,float(max(moments)),int(self.broken.sum())))

    def prepare_approach(self):
        for _ in range(round(self.cfg['settle_s']/self.model.opt.timestep)):
            self.step(settling=True)
        if self.bend_indices:
            self.nominal_bend = self.data.qpos[self.bend_indices].copy()
        # Position both settled assemblies the same distance from first contact
        # and inject velocity into every free body (including welded arms).
        leading = contact_front_x(self.model, self.data, self.wall)
        wall_front=float(self.model.geom_pos[self.wall,0]-self.model.geom_size[self.wall,0])
        shift = wall_front-leading-self.speed*self.cfg['approach_s']
        for j in range(self.model.njnt):
            if self.model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
                q, v = self.model.jnt_qposadr[j], self.model.jnt_dofadr[j]
                self.data.qpos[q] += shift
                self.data.qvel[v] = self.speed
        self.data.time = 0
        mujoco.mj_forward(self.model,self.data)
        return self

    def run(self):
        self.prepare_approach()
        for _ in range(round((self.cfg['approach_s']+self.cfg['post_impact_s'])/self.model.opt.timestep)):
            self.step()
        return self

    def summary(self):
        # A sensor bandwidth must be stated when comparing shock peaks. Retain
        # unfiltered solver peaks as well as a 10 ms mean body deceleration.
        vx = np.array([sample[1][0] for sample in self.history])
        acc = -np.diff(vx)/self.model.opt.timestep
        samples = max(1,round(.01/self.model.opt.timestep))
        averaged = np.convolve(acc,np.ones(samples)/samples,mode='valid') if len(acc)>=samples else np.array([])
        start = max(0,round((self.impact_time or 0)/self.model.opt.timestep)-samples)
        stop = min(len(averaged),start+round(.25/self.model.opt.timestep))
        filtered_peak = float(max(0.,np.max(averaged[start:stop]))) if stop>start else 0.
        return dict(kind=self.kind, model_variant=self.model_variant, model_label=self.model_label,
                    requested_speed_m_s=self.speed, actual_impact_speed_m_s=self.actual_impact_speed,
                    impact_time_s=self.impact_time, peak_wall_force_n=self.peak_force,
                    peak_horizontal_acceleration_m_s2=self.peak_accel, first_impact_impulse_ns=self.impulse,
                    peak_10ms_mean_body_deceleration_m_s2=filtered_peak,
                    peak_arm_contact_moment_nm=self.peak_arm_moment, peak_arm_contact_force_n=self.peak_arm_force,
                    peak_tilt_deg=self.peak_tilt, peak_bend_deg=self.peak_bend, peak_compression_mm=self.peak_axial_compression_mm,
                    peak_abs_slide_mm=self.peak_slide,
                    max_slide_limit_violation_mm=self.max_slide_limit_violation_mm,
                    fracture_enabled=self.fracture_enabled,
                    broken_arms=int(self.broken.sum()), failure_times_s=self.failure_times,
                    ground_contact_time_s=self.ground_time, recovered=self.recovery_time is not None,
                    recovery_time_s=self.recovery_time, final_position_m=self.data.qpos[:3].tolist(),
                    final_speed_m_s=float(np.linalg.norm(self.data.qvel[:3])),
                    simulation_warning_count=int(sum(w.number for w in self.data.warning)),
                    model_provenance=model_provenance(self.model_variant, self.cfg),
                    joint_guard_reached=self.joint_guard_reached)


def camera(close=False):
    c = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(c)
    c.lookat[:] = (.12,0,1.15) if close else (.10,0,1.05)
    c.distance = .95 if close else 2.1
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
        font_path = next((p for p in ['/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf','C:/Windows/Fonts/arial.ttf'] if Path(p).exists()),None)
        font = ImageFont.truetype(font_path,max(14,round(width/30))) if font_path else ImageFont.load_default()
        small = ImageFont.truetype(font_path,max(12,round(width/40))) if font_path else ImageFont.load_default()
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
                    title = t.model_label
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
    rigid_variants = {t.model_variant for t in trials if t.kind == 'rigid'} or {cfg.get('rigid_model', 'hard-arm')}
    (directory/'results.json').write_text(json.dumps(dict(config=cfg,visual=dict(
        soft='HoLoArm user CAD, frozen nonlinear K and v2 viscoelastic model',
        rigid=' '.join(RIGID_DESCRIPTIONS[v] for v in sorted(rigid_variants)),
        physics='Shared total mass, controller, thrust limits, motor response and guard-wall contact parameters; '
                'each model retains its own collision geometry, inertia and yaw-reaction coefficient. '
                'Normal-only ideal-slip wall contact; no assumed fracture by default'),
        results=summaries),indent=2))
    with (directory/'summary.csv').open('w',newline='') as f:
        writer = csv.DictWriter(f,fieldnames=list(summaries[0]))
        writer.writeheader()
        writer.writerows(summaries)
    for t in trials:
        with (directory/f'{t.kind}_{t.speed:g}ms_trace.csv').open('w',newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time_s','x_m','y_m','z_m','vx_m_s','vy_m_s','vz_m_s','tilt_deg',
                             'arm_bend_deg','arm_compression_mm','arm_abs_slide_mm','wall_force_n',
                             'arm_contact_moment_nm','broken_arms'])
            writer.writerows(t.trace)
    return summaries


def write_report(summaries,directory,cfg):
    fracture_enabled=any(r['fracture_enabled'] for r in summaries)
    rigid_variants={row['model_variant'] for row in summaries if row['kind']=='rigid'} or {cfg.get('rigid_model','hard-arm')}
    lines = [
        '# Soft HoLoArm CAD vs rigid reference: wall collision', '',
        'The compliant vehicle uses the supplied CAD and frozen nonlinear stiffness and v2 '
        'viscoelastic parameters.',
        ' '.join(RIGID_DESCRIPTIONS[v] for v in sorted(rigid_variants)), '',
        f"Mass: {cfg['mass_kg']:g} kg; timestep: {cfg['timestep_s']:g} s; motor response: {cfg['motor_time_constant_s']:g} s.",
        f"Wall front: {cfg['wall_front_x_m']:g} m; full thickness: {cfg['wall_thickness_m']:g} m.",
        'Both cases use the same total mass, PD controller, thrust limits, motor response, '
        'guard-wall contact parameters and initial-speed protocol. '
        'Each starts the same distance from wall contact using its actual collision envelope. '
        'Neutral-hover preload and normal-only ideal-slip wall contacts remain assumptions.', '',
        ('Uncalibrated legacy fracture thresholds were explicitly enabled: '
         f"arm moment > {cfg['rigid_failure_moment_nm']:g} Nm or force > {cfg['rigid_failure_force_n']:g} N."
         if fracture_enabled else 'Assumed arm fracture is disabled for the rigid reference.'), '',
        '| Set speed (m/s) | Model | Actual impact (m/s) | Bend (deg) | Compression (mm) | Joint guard reached | Recovery (s) | Floor contact |',
        '|---:|:---|---:|---:|---:|:---:|---:|:---:|'
    ]
    for row in summaries:
        recovery=f"{row['recovery_time_s']:.3f}" if row['recovery_time_s'] is not None else 'Not observed'
        actual=f"{row['actual_impact_speed_m_s']:.3f}" if row['actual_impact_speed_m_s'] is not None else 'No impact'
        model=row['model_label']
        guard='Yes' if row['joint_guard_reached'] else 'No'
        floor='Yes' if row['ground_contact_time_s'] is not None else 'No'
        lines.append(f"| {row['requested_speed_m_s']:g} | {model} | {actual} | {row['peak_bend_deg']:.1f} | "
                     f"{row['peak_compression_mm']:.3f} | {guard} | {recovery} | {floor} |")
    lines += ['',
        f"Recovery requires position error < {cfg['recovery_position_tolerance_m']:g} m, "
        f"speed < {cfg['recovery_speed_tolerance_m_s']:g} m/s and tilt < {cfg['recovery_tilt_tolerance_deg']:g} deg "
        f"for {cfg['recovery_hold_s']:g} s, without wall/floor contact or detached arms. "
        'The time is measured from first contact to the start of that hold.', '',
        'See results.json and summary.csv for the actual model provenance, soft material hashes, raw first-impact force peaks, '
        '10 ms mean body deceleration, slide overtravel, solver warnings and final states. '
        'Traces distinguish positive axial compression from absolute slide displacement.', '',
        '**Limits:** Frozen bench identification does not validate flight behavior, preload, collision '
        'contact or failure safety. Joint guards may dominate stronger impacts; reaching them is reported '
        'rather than interpreted as material survival. Recovery and floor-contact outcomes are measured '
        'from the simulation, with no required success classification. Soft material fracture is not modeled.', '',
        'Method: ../../holoarm/WALL_COLLISION_STUDY.md', '']
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
        if not rows:
            continue
        for ax,(key,label) in zip(axes.flat,plots):
            ax.plot([r['actual_impact_speed_m_s'] for r in rows],
                    [np.nan if r[key] is None else r[key] for r in rows],'-o',label=rows[0]['model_label'],color=colour,ms=4)
            ax.set(xlabel='Measured impact speed (m/s)',ylabel=label)
            ax.grid(alpha=.25)
            ax.legend()
    references=', '.join(sorted({r['model_label'] for r in summaries if r['kind']=='rigid'}))
    fig.suptitle(f'Soft HoLoArm CAD vs {references}: same PD controller')
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
    p.add_argument('--impact-speed',type=float,default=.6)
    p.add_argument('--rigid-model',choices=('hard-arm','locked-cad'),default=None,
                   help='Rigid comparison model (default: previous hard-arm / X2-style model)')
    p.add_argument('--wall-thickness',type=float,help='Full thickness in metres; front contact face stays fixed')
    p.add_argument('--sweep',action='store_true',help='CAD comparison at 0.3, 0.6 and 0.9 m/s')
    p.add_argument('--speeds',type=float,nargs='+',help='custom survey speeds')
    p.add_argument('--output',type=Path,default=HERE.parent/'outputs'/'wall_collision_recovery.mp4')
    p.add_argument('--results-dir',type=Path,default=HERE.parent/'outputs'/'wall_collision_study')
    p.add_argument('--no-video',action='store_true')
    p.add_argument('--overwrite',action='store_true',help='replace existing target videos and regenerate results')
    fracture_group=p.add_mutually_exclusive_group()
    fracture_group.add_argument('--no-fracture',action='store_true',default=True,help='rigid reference without assumed fracture (default)')
    fracture_group.add_argument('--assumed-fracture',dest='no_fracture',action='store_false',help='explicitly enable uncalibrated legacy fracture thresholds')
    p.add_argument('--duration',type=float,help='simulated seconds after first expected impact')
    p.add_argument('--width',type=int,default=640)
    p.add_argument('--height',type=int,default=540)
    p.add_argument('--fps',type=int,default=30)
    args = p.parse_args()
    cfg = load_config(args.config)
    if args.rigid_model is not None:
        cfg['rigid_model'] = args.rigid_model
    if args.wall_thickness is not None:
        if not math.isfinite(args.wall_thickness) or args.wall_thickness<=0:p.error('wall thickness must be positive and finite')
        cfg['wall_thickness_m']=args.wall_thickness
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
            print(f"{speed:g} m/s {t.model_label}: actual={s['actual_impact_speed_m_s']:.3f}, "
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
