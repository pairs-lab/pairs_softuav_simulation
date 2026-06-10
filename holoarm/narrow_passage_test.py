import time
import math
import numpy as np
import threading
import pickle
from pathlib import Path

import mujoco
import mujoco.viewer

from simple_pid import PID
from pynput import keyboard

# Resolve the model path relative to this file so the project is portable.
MODELS_DIR = Path(__file__).resolve().parent / "models"
NARROW_PASSAGE_XML = MODELS_DIR / "holoarm_narrow_passage.xml"

# KEYBOARD CONTROLLER
class KeyboardController:
    def __init__(self, step_size=0.3, alt_step=0.3):
        self.pressed_keys = set()
        self.lock = threading.Lock()
        self.step_size = step_size
        self.alt_step = alt_step

        self._listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release
        )

    def start(self):
        self._listener.start()
        print("\n" + "="*40)
        print("Keyboard Controller")
        print(" ↑ / ↓ : forward / back")
        print(" ← / → : left / right")
        print(" u / o : up / down")
        print(" ESC : quit")

    def stop(self):
        self._listener.stop()

    def _on_press(self, key):
        with self.lock:
            self.pressed_keys.add(key)

    def _on_release(self, key):
        with self.lock:
            self.pressed_keys.discard(key)
        if key == keyboard.Key.esc:
            return False

    def _is_pressed(self, *keys):
        with self.lock:
            return any(k in self.pressed_keys for k in keys)

    def get_target_delta_world(self):
        dx, dy, dz = 0.0, 0.0, 0.0
        if self._is_pressed(keyboard.KeyCode.from_char('i'), keyboard.Key.up):
            dx += self.step_size
        if self._is_pressed(keyboard.KeyCode.from_char('k'), keyboard.Key.down):
            dx -= self.step_size
        if self._is_pressed(keyboard.KeyCode.from_char('j'), keyboard.Key.left):
            dy += self.step_size
        if self._is_pressed(keyboard.KeyCode.from_char('l'), keyboard.Key.right):
            dy -= self.step_size
        if self._is_pressed(keyboard.KeyCode.from_char('u')):
            dz += self.alt_step
        if self._is_pressed(keyboard.KeyCode.from_char('o')):
            dz -= self.alt_step

        return np.array([dx, dy, dz])

# UTILITY FUNCTIONS
def quat_to_euler(qw, qx, qy, qz):
    # Roll
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    # Pitch
    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)
    # Yaw
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

def world_vel_to_body_vel(vx_world, vy_world, yaw):
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    vx_body =  cos_yaw * vx_world + sin_yaw * vy_world
    vy_body = -sin_yaw * vx_world + cos_yaw * vy_world
    return vx_body, vy_body

# planner
class dummyPlanner:
    def __init__(self, target, vel_limit=4.0) -> None:
        self.target = target
        self.vel_limit = vel_limit
        self.pid_x = PID(2, 0.15, 1.5, setpoint=self.target[0], output_limits=(-vel_limit, vel_limit))
        self.pid_y = PID(2, 0.15, 1.5, setpoint=self.target[1], output_limits=(-vel_limit, vel_limit))

    def __call__(self, loc: np.array):
        velocities = np.array([0.0, 0.0, 0.0])
        velocities[0] = self.pid_x(loc[0])
        velocities[1] = self.pid_y(loc[1])
        return velocities

    def get_alt_setpoint(self, loc: np.array) -> float:
        distance = self.target[2] - loc[2]
        if abs(distance) > 0.5:
            time_to_target = abs(distance) / self.vel_limit
            number_steps = int(time_to_target / 0.25)
            delta_alt = distance / max(number_steps, 1)
            alt_set = loc[2] + 2 * delta_alt
        else:
            alt_set = self.target[2]
        return alt_set

    def update_target(self, target):
        self.target = target
        self.pid_x.setpoint = self.target[0]
        self.pid_y.setpoint = self.target[1]

class dummySensor:
    def __init__(self, d):
        self.d = d

    def get_position(self):
        return self.d.qpos

    def get_velocity(self):
        return self.d.qvel

# DRONE CLASS 
class drone:
    def __init__(self, target=np.array((0, 0, 1.0))):
        self.m = mujoco.MjModel.from_xml_path(str(NARROW_PASSAGE_XML))
        self.d = mujoco.MjData(self.m)

        self.planner = dummyPlanner(target=target)
        self.sensor  = dummySensor(self.d)

        self.HOVER_THRUST = 2.3789
        self.MAX_THRUST = 8.0

        # Inner control
        self.pid_alt   = PID(5.50844, 0.57871, 1.2,      setpoint=0)
        self.pid_roll  = PID(2.6785,  0.56871, 1.2508,   setpoint=0, output_limits=(-1.0, 1.0))
        self.pid_pitch = PID(2.6785,  0.56871, 1.2508,   setpoint=0, output_limits=(-1.0, 1.0))
        self.pid_yaw   = PID(0.54,    0.0,     5.358333, setpoint=0, output_limits=(-3.0, 3.0))

        # Outer control
        self.pid_v_x = PID(0.1, 0.003, 0.02, setpoint=0, output_limits=(-0.5, 0.5))
        self.pid_v_y = PID(0.1, 0.003, 0.02, setpoint=0, output_limits=(-0.5, 0.5))
        self.current_yaw = 0.0

    def reset_pose(self, x, y, z):
        self.d.qpos[0:3] = [x, y, z]
        self.d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        self.d.qvel[:] = 0.0
        mujoco.mj_forward(self.m, self.d)

    def update_outer_control(self):
        pos      = self.sensor.get_position()
        v        = self.sensor.get_velocity()
        location = pos[:3]

        qw, qx, qy, qz = pos[3], pos[4], pos[5], pos[6]
        _, _, self.current_yaw = quat_to_euler(qw, qx, qy, qz)

        vel_world = self.planner(loc=location)
        vx_world, vy_world = vel_world[0], vel_world[1]

        vx_body_setpoint, vy_body_setpoint = world_vel_to_body_vel(vx_world, vy_world, self.current_yaw)
        vx_body_measured, vy_body_measured = world_vel_to_body_vel(v[0], v[1], self.current_yaw)

        self.pid_alt.setpoint = self.planner.get_alt_setpoint(location)
        self.pid_v_x.setpoint = vx_body_setpoint
        self.pid_v_y.setpoint = vy_body_setpoint

        # Convert velocity error to target angle
        angle_pitch =  self.pid_v_x(vx_body_measured)
        angle_roll  = -self.pid_v_y(vy_body_measured)

        self.pid_pitch.setpoint = angle_pitch
        self.pid_roll.setpoint  = angle_roll

    def update_inner_control(self):
        pos = self.sensor.get_position()
        alt = pos[2]
        qw, qx, qy, qz = pos[3], pos[4], pos[5], pos[6]
        roll, pitch, yaw = quat_to_euler(qw, qx, qy, qz)

        # Calculate Base Thrust 
        cmd_thrust = self.pid_alt(alt) + self.HOVER_THRUST

        cmd_roll   = -self.pid_roll(roll)
        cmd_pitch  =  self.pid_pitch(pitch)
        cmd_yaw    = self.pid_yaw(yaw)

        out = self.compute_motor_control(cmd_thrust, cmd_roll, cmd_pitch, cmd_yaw)
        self.d.ctrl[:4] = out

    def compute_motor_control(self, T, roll, pitch, yaw):
        # Motor Mixing for X-Quadrotor
        m_fl = T - pitch - roll - yaw
        m_fr = T - pitch + roll + yaw
        m_rl = T + pitch - roll + yaw
        m_rr = T + pitch + roll - yaw

        return [
            float(np.clip(m_fl, 0, self.MAX_THRUST)),
            float(np.clip(m_fr, 0, self.MAX_THRUST)),
            float(np.clip(m_rl, 0, self.MAX_THRUST)),
            float(np.clip(m_rr, 0, self.MAX_THRUST))
        ]

# MAIN LOOP
if __name__ == "__main__":
    # Starting position
    INITIAL_POS = np.array([0.0, 0.0, 1.0])
    my_drone = drone(target=INITIAL_POS)
    
    kb = KeyboardController(step_size=0.1, alt_step=0.2)
    kb.start()

    current_target = INITIAL_POS.copy()
    KEYBOARD_UPDATE_INTERVAL = 30

    with mujoco.viewer.launch_passive(my_drone.m, my_drone.d) as viewer:
        my_drone.reset_pose(*INITIAL_POS)
        
        print("[SYSTEM] Waiting for stabilization (3s)...")
        time.sleep(3)
        
        start_time = time.time()
        step  = 1

        while viewer.is_running():
            step_start = time.time()

            # Listen to the keyboard to change Target (World Frame)
            if step % KEYBOARD_UPDATE_INTERVAL == 0:
                delta = kb.get_target_delta_world()
                if np.any(delta != 0):
                    current_target = current_target + delta
                    # Set the altitude limit not to touch the ground or fly too high
                    current_target[2] = np.clip(current_target[2], 0.2, 5.0) 
                    my_drone.planner.update_target(current_target)
                    print(f"Target: X={current_target[0]:.2f} | Y={current_target[1]:.2f} | Z={current_target[2]:.2f}")

            # Outer Loop: Run every 20 steps (40ms)
            if step % 20 == 0:
                my_drone.update_outer_control()

            # Inner Loop: Run every step (2ms)
            my_drone.update_inner_control()

            mujoco.mj_step(my_drone.m, my_drone.d)
            viewer.sync()

            step += 1
            
            # Balance actual time with simulation time (0.002s/step)
            time_until_next_step = my_drone.m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    kb.stop()
    print("\n[SYSTEM] Exit simulation.")