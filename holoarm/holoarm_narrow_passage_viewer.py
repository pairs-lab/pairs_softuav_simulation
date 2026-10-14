"""Keyboard flight with the reference-style HoLoArm and passive folding arms.

Run from the repository root:
    python3 holoarm/holoarm_narrow_passage_viewer.py
Uses window-local X11 input: focus the MuJoCo window, hold to fly, release to brake.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from collision_study import ARMS, build_model, load_config, tilt_deg
from soft_visual import add_soft_ligaments
from viewer_keyboard import X11Keyboard


def arena(gap):
    """World-fixed contacts: a tapered doorway, not an invisible constraint."""
    world = ET.Element('worldbody')

    def box(name, pos, size, colour, contact=True, yaw=0.):
        return ET.SubElement(world, 'geom', name=name, type='box',
            pos=' '.join(map(str, pos)), size=' '.join(map(str, size)),
            euler=f'0 0 {yaw}', rgba=colour, mass='0',
            contype='1' if contact else '0', conaffinity='2' if contact else '0')

    half = gap / 2
    for side in (-1, 1):
        box(f'slot_wall_{side}', (2.1, side*(half+.8), 1.2),
            (.25, .8, 1.2), '.42 .49 .56 1')
        # Inner faces of guides join the exact slot boundary; offset their
        # thickness outwards so --gap measures clear width, not centre lines.
        for label, start, end in (
            ('entrance', (1.15, side*(half+.28)), (1.85, side*half)),
            ('exit', (2.35, side*half), (3.05, side*(half+.28))),
        ):
            a, b = np.array(start), np.array(end)
            delta = b-a
            length = np.linalg.norm(delta)
            normal = np.array([-delta[1], delta[0]]) / length * side
            centre = (a+b)/2 + .025*normal
            box(f'{label}_guide_{side}', (*centre, 1.2),
                (length/2, .025, 1.2), '.62 .70 .75 1', yaw=math.atan2(delta[1], delta[0]))
        box(f'slot_edge_{side}', (1.845, side*(half+.008), 1.2),
            (.004, .008, 1.18), '1 .55 .08 1', contact=False)
    return world


class Flight:
    """Position/yaw hold with allocation from the current folded motor poses."""
    control_period = .002

    def __init__(self, gap=.46, speed=.6):
        self.cfg = load_config()
        self.model = build_model('soft', self.cfg, environment=arena(gap))
        self.data = mujoco.MjData(self.model)
        self.core = self.model.body('holoarm_core').id
        self.sites = [self.model.site(f'thrust_{a}').id for a in ARMS]
        self.bend = [self.model.joint(f'{a}_bend_{axis}').qposadr[0]
                     for a in ARMS for axis in ('pitch', 'yaw')]
        self.speed = speed
        self.gap = gap
        self.mass = self.model.body_mass.sum()
        self.control_steps = max(1, round(self.control_period/self.model.opt.timestep))
        self.reset()

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self.target = self.data.qpos[:3].copy()
        self.yaw = 0.
        self.thrust = np.full(4, self.mass*9.81/4)
        self.data.ctrl[:] = self.thrust
        self.step_count = 0
        self.max_bend = 0.
        self.manual_velocity = np.zeros(3)
        self.manual_axes = np.zeros(3, dtype=bool)
        self.yaw_rate = 0.

    def hold(self):
        self.target = self.data.qpos[:3].copy()
        R = self.data.xmat[self.core].reshape(3, 3)
        self.yaw = math.atan2(R[1, 0], R[0, 0])
        self.manual_velocity[:] = 0
        self.manual_axes[:] = False
        self.yaw_rate = 0.

    def set_manual_input(self, velocity, axes, yaw_rate):
        # A released axis holds its actual position, not an old distant goal.
        released = self.manual_axes & ~axes
        self.target[released] = self.data.qpos[:3][released]
        if self.yaw_rate and not yaw_rate:
            R = self.data.xmat[self.core].reshape(3, 3)
            self.yaw = math.atan2(R[1, 0], R[0, 0])
        self.manual_velocity[:] = velocity
        self.manual_axes[:] = axes
        self.yaw_rate = yaw_rate

    def clamp_target(self):
        self.target[:] = np.clip(self.target, [-1., -1.6, .35], [4.5, 1.6, 2.1])

    def controller(self):
        m, d = self.model, self.data
        R = d.xmat[self.core].reshape(3, 3)
        desired_velocity = 1.8*(self.target-d.qpos[:3])
        norm = np.linalg.norm(desired_velocity[:2])
        if norm > self.speed:
            desired_velocity[:2] *= self.speed/norm
        desired_velocity[2] = np.clip(desired_velocity[2], -.45, .45)
        self.target[self.manual_axes] = d.qpos[:3][self.manual_axes]
        desired_velocity[self.manual_axes] = self.manual_velocity[self.manual_axes]
        for axis, (lower, upper) in enumerate(((-1., 4.5), (-1.6, 1.6), (.35, 2.1))):
            if d.qpos[axis] <= lower and desired_velocity[axis] < 0:
                desired_velocity[axis] = 0
            elif d.qpos[axis] >= upper and desired_velocity[axis] > 0:
                desired_velocity[axis] = 0
        dt = self.control_steps*m.opt.timestep
        self.yaw += self.yaw_rate*dt
        acc = np.clip(4*(desired_velocity-d.qvel[:3]), [-3, -3, -4], [3, 3, 4])
        force = self.mass*(acc+np.array([0., 0., 9.81]))
        z = force/np.linalg.norm(force)
        heading = np.array([math.cos(self.yaw), math.sin(self.yaw), 0])
        y = np.cross(z, heading)
        y /= np.linalg.norm(y)
        desired_rotation = np.column_stack((np.cross(y, z), y, z))
        skew = .5*(desired_rotation.T@R-R.T@desired_rotation)
        error = np.array([skew[2, 1], skew[0, 2], skew[1, 0]])
        torque = -np.array([.75, .75, .28])*error-np.array([.12, .12, .06])*d.qvel[3:6]
        com = (d.xipos*m.body_mass[:, None]).sum(axis=0)/self.mass
        allocation = np.zeros((4, 4))
        for i, site in enumerate(self.sites):
            direction = d.site_xmat[site].reshape(3, 3)[:, 2]
            moment = np.cross(d.site_xpos[site]-com, direction)+m.actuator_gear[i, 5]*direction
            allocation[0, i] = (R.T@direction)[2]
            allocation[1:, i] = R.T@moment
        command = np.linalg.lstsq(allocation, np.r_[max(0., force@R[:, 2]), torque], rcond=None)[0]
        command = np.clip(command, 0, self.cfg['max_motor_thrust_n'])
        self.thrust += (1-math.exp(-dt/self.cfg['motor_time_constant_s']))*(command-self.thrust)
        d.ctrl[:] = self.thrust

    def step(self):
        if self.step_count % self.control_steps == 0:
            self.controller()
        mujoco.mj_step(self.model, self.data)
        self.step_count += 1
        self.max_bend = max(self.max_bend, float(np.max(np.abs(self.data.qpos[self.bend]))))


CONTROLS = '''HoLoArm | Keyboard flight (focus this window)
I / Up: +X forward     K / Down: -X backward
J / Left: +Y left      L / Right: -Y right
U / O: up / down      Q / E: yaw left / right
H: brake + hover      R: reset     Space: pause
F: camera (overview / follow / top)    Esc: quit
Hold movement keys for continuous world-coordinate velocity.
Release to brake and hover; losing window focus also brakes.
White frame + orange bands follow actual arm motion.'''


class HeldControls:
    """Physical held states, with single edge-triggered UI actions."""
    movement = set('IKJLUOQE')

    def __init__(self):
        self.previous = set()
        self.blocked = set()

    def update(self, flight, down, enabled=True):
        pressed = down-self.previous
        self.previous = set(down)
        self.blocked &= down
        if 'R' in pressed:
            flight.reset()
            self.blocked |= down & self.movement
        if 'H' in pressed or not enabled:
            flight.hold()
            self.blocked |= down & self.movement
        active = (down-self.blocked) & self.movement if enabled else set()
        axes = np.array([bool(active & set('IK')), bool(active & set('JL')),
                         bool(active & set('UO'))])
        velocity = np.array([int('I' in active)-int('K' in active),
                             int('J' in active)-int('L' in active),
                             int('U' in active)-int('O' in active)], dtype=float)
        norm = np.linalg.norm(velocity[:2])
        if norm:
            velocity[:2] *= flight.speed/norm
        velocity[2] *= .45
        yaw_rate = math.radians(60)*(int('Q' in active)-int('E' in active))
        flight.set_manual_input(velocity, axes, yaw_rate)
        return pressed


def clean_flight_visuals(viewer):
    """Native shortcuts also run on key_callback events; hide their overlays.

    I/J/H/Q/U/E/F are native debug toggles and L enables additive rendering.
    Enforce flight presentation each frame, while leaving C contact markers,
    texture toggles, mouse controls and all other useful viewer options alone.
    """
    for name in ('mjVIS_INERTIA', 'mjVIS_JOINT', 'mjVIS_CONVEXHULL',
                 'mjVIS_CAMERA', 'mjVIS_ACTUATOR', 'mjVIS_CONSTRAINT',
                 'mjVIS_CONTACTFORCE'):
        viewer.opt.flags[getattr(mujoco.mjtVisFlag, name)] = 0
    for name, value in (('mjRND_ADDITIVE', 0), ('mjRND_REFLECTION', 1),
                        ('mjRND_SKYBOX', 1)):
        viewer.user_scn.flags[getattr(mujoco.mjtRndFlag, name)] = value


def smoke_test(flight):
    """Check hover and physically pass through the slot without teleportation."""
    m, d = flight.model, flight.data
    for _ in range(round(2/m.opt.timestep)):
        flight.step()
    hover_error = float(np.linalg.norm(d.qpos[:3]-flight.target))
    controls = HeldControls()
    controls.update(flight, {'I'})
    released_at = None
    contacts = 0
    peak_tilt = 0.
    min_height = float(d.qpos[2])
    for _ in range(round(14/m.opt.timestep)):
        if released_at is None and d.qpos[0] >= 3.65:
            controls.update(flight, set())
            released_at = d.time
        flight.step()
        contacts += int(d.ncon > 0)
        peak_tilt = max(peak_tilt, tilt_deg(d, flight.core))
        min_height = min(min_height, float(d.qpos[2]))
        if released_at is not None and d.time-released_at > 2:
            break
    mujoco.mj_forward(m, d)
    result = dict(gap_m=flight.gap, hover_error_m=hover_error,
        final_position_m=d.qpos[:3].tolist(), contacts_steps=contacts,
        max_bend_deg=math.degrees(flight.max_bend), peak_tilt_deg=peak_tilt,
        min_height_m=min_height, final_speed_m_s=float(np.linalg.norm(d.qvel[:3])),
        simulation_warning_count=int(sum(w.number for w in d.warning)))
    print(json.dumps(result, indent=2))
    if not (hover_error < .05 and d.qpos[0] > 3.3 and min_height > .7
            and contacts > 0 and result['max_bend_deg'] > 5
            and released_at is not None and result['final_speed_m_s'] < .05
            and result['simulation_warning_count'] == 0):
        raise SystemExit('Narrow-passage smoke test failed')


def run_viewer(flight):
    import mujoco.viewer

    controls = HeldControls()
    paused = False
    camera_mode = 0
    m, d = flight.model, flight.data
    print(CONTROLS, flush=True)
    print(f'Clear slot: {flight.gap:.2f} m. Flight speed limit: {flight.speed:.2f} m/s.', flush=True)
    with mujoco.viewer.launch_passive(m, d,
            show_left_ui=False, show_right_ui=False) as viewer, \
            X11Keyboard('MuJoCo : HoLoArm keyboard narrow passage') as keyboard:
        with viewer.lock():
            viewer.cam.lookat[:] = [1.5, 0, 1.0]
            viewer.cam.distance = 4.6
            viewer.cam.azimuth = 0
            viewer.cam.elevation = -40
        deadline = time.monotonic()
        frame_steps = round(1/60/m.opt.timestep)
        while viewer.is_running():
            down = keyboard.poll()
            pressed = down-controls.previous
            if 'ESC' in pressed:
                return
            if 'SPACE' in pressed:
                paused = not paused
            camera_changed = 'F' in pressed
            if camera_changed:
                camera_mode = (camera_mode+1) % 3
            with viewer.lock():
                controls.update(flight, down, enabled=not paused)
                if not paused:
                    for _ in range(frame_steps):
                        flight.step()
                mujoco.mj_forward(m, d)
                clean_flight_visuals(viewer)
                viewer.user_scn.ngeom = 0
                add_soft_ligaments(viewer.user_scn, m, d)
                if camera_mode:
                    viewer.cam.lookat[:] = d.qpos[:3]
                    viewer.cam.distance = 1.7 if camera_mode == 1 else 2.5
                    viewer.cam.azimuth = 0 if camera_mode == 1 else 180
                    viewer.cam.elevation = -45 if camera_mode == 1 else -89
                # Restore overview only on F, preserving manual mouse orbit.
                elif camera_changed:
                    viewer.cam.lookat[:] = [1.5, 0, 1.0]
                    viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 4.6, 0, -40
            if hasattr(viewer, 'set_texts'):
                state = 'PAUSED' if paused else 'FLYING'
                bend = math.degrees(float(np.max(np.abs(d.qpos[flight.bend]))))
                viewer.set_texts([(mujoco.mjtFontScale.mjFONTSCALE_100,
                    mujoco.mjtGridPos.mjGRID_TOPLEFT,
                    f'HoLoArm | {state} | gap {flight.gap*100:.0f} cm\n'
                    f'XYZ {d.qpos[0]:.2f}, {d.qpos[1]:.2f}, {d.qpos[2]:.2f} m\n'
                    f'Arm bend {bend:.1f} deg | contacts {d.ncon}',
                    'I/K forward/back | J/L left/right\nU/O up/down | Q/E yaw\n'
                    'Hold to fly / release to brake\nH hover | R reset | Space pause | F camera')])
            viewer.sync()
            deadline += frame_steps*m.opt.timestep
            delay = deadline-time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                deadline = time.monotonic()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gap', type=float, default=.46, help='Clear slot width in metres (default .46)')
    parser.add_argument('--speed', type=float, default=.6, help='Horizontal speed limit, m/s (default .6)')
    parser.add_argument('--smoke-test', action='store_true', help='Run headless hover and slot-crossing checks')
    args = parser.parse_args()
    if not math.isfinite(args.gap) or not .3 <= args.gap <= 1.:
        parser.error('--gap must be between .3 and 1 metre')
    if not math.isfinite(args.speed) or not .1 <= args.speed <= 2.:
        parser.error('--speed must be between .1 and 2 m/s')
    if not args.smoke_test and not os.environ.get('DISPLAY'):
        parser.error('A desktop DISPLAY is required for the interactive viewer')
    flight = Flight(args.gap, args.speed)
    smoke_test(flight) if args.smoke_test else run_viewer(flight)


if __name__ == '__main__':
    main()
