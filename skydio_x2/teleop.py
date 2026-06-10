import time
import numpy as np
import threading
from pathlib import Path

import mujoco
import mujoco.viewer
import pickle

from simple_pid import PID
from pynput import keyboard

# Resolve the model path relative to this file so the project is portable.
INDOOR_ENV_XML = Path(__file__).resolve().parent / "models" / "indoor_env.xml"



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
        print("=== Skydio X2 Indoor Controller ===")
        print("  i / k  or ↑ / ↓  : forward / backward (World Y)")
        print("  j / l  or ← / →  : left / right (World X)")
        print("  u / o : up / down (World Z)")
        print("  ESC : exit")
        print("  [Start pos: Living room center (3, 5, 1.0)]")

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
            dy += self.step_size
        if self._is_pressed(keyboard.KeyCode.from_char('k'), keyboard.Key.down):
            dy -= self.step_size
        if self._is_pressed(keyboard.KeyCode.from_char('j'), keyboard.Key.left):
            dx -= self.step_size
        if self._is_pressed(keyboard.KeyCode.from_char('l'), keyboard.Key.right):
            dx += self.step_size
        if self._is_pressed(keyboard.KeyCode.from_char('u')):
            dz += self.alt_step
        if self._is_pressed(keyboard.KeyCode.from_char('o')):
            dz -= self.alt_step

        return np.array([dx, dy, dz])

# UTILITY FUNCTIONS

def save_data(filename, positions, velocities):
    data = {'positions': positions, 'velocities': velocities}
    with open(filename, 'wb') as f:
        pickle.dump(data, f)

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


def pid_to_thrust(input: np.array):
    c_to_F = np.array([
        [-0.25, 0.25, 0.25, -0.25],
        [ 0.25, 0.25,-0.25, -0.25],
        [-0.25, 0.25,-0.25,  0.25]
    ]).transpose()
    return np.dot((c_to_F * input), np.array([1, 1, 1]))

def outer_pid_to_thrust(input: np.array):
    c_to_F = np.array([
        [ 0.25, 0.25,-0.25,-0.25],
        [ 0.25,-0.25,-0.25, 0.25],
        [ 0.25, 0.25, 0.25, 0.25]
    ]).transpose()
    return np.dot((c_to_F * input), np.array([1, 1, 1]))


# CONTROLLERS

class PDController:
    def __init__(self, kp, kd, setpoint):
        self.kp = kp
        self.kd = kd
        self.setpoint = setpoint
        self.prev_error = 0

    def compute(self, measured_value):
        error = self.setpoint - measured_value
        derivative = error - self.prev_error
        output = (self.kp * error) + (self.kd * derivative)
        self.prev_error = error
        return output

class PIDController:
    def __init__(self, kp, ki, kd, setpoint):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0

    def compute(self, measured_value):
        error = self.setpoint - measured_value
        self.integral += error
        derivative = error - self.prev_error
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.prev_error = error
        return output


# PLANNER

class dummyPlanner:
    def __init__(self, target, vel_limit=2) -> None:
        self.target = target
        self.vel_limit = vel_limit
        self.pid_x = PID(2, 0.15, 1.5, setpoint=self.target[0],
                         output_limits=(-vel_limit, vel_limit))
        self.pid_y = PID(2, 0.15, 1.5, setpoint=self.target[1],
                         output_limits=(-vel_limit, vel_limit))

    def __call__(self, loc: np.array):
        velocities = np.array([0.0, 0.0, 0.0])
        velocities[0] = self.pid_x(loc[0])
        velocities[1] = self.pid_y(loc[1])
        return velocities

    def get_velocities(self, loc, target, time_to_target=None, flight_speed=0.5):
        direction = target - loc
        distance = np.linalg.norm(direction)
        if distance > 1:
            velocities = flight_speed * direction / distance
        else:
            velocities = direction * distance
        return velocities

    def get_alt_setpoint(self, loc: np.array) -> float:
        target = self.target
        distance = target[2] - loc[2]
        if distance > 0.5:
            time_to_target = distance / self.vel_limit
            number_steps = int(time_to_target / 0.25)
            delta_alt = distance / max(number_steps, 1)
            alt_set = loc[2] + 2 * delta_alt
        else:
            alt_set = target[2]
        return alt_set

    def update_target(self, target):
        self.target = target
        self.pid_x.setpoint = self.target[0]
        self.pid_y.setpoint = self.target[1]


# SENSOR

class dummySensor:
    def __init__(self, d):
        self.position = d.qpos
        self.velocity = d.qvel
        self.acceleration = d.qacc

    def get_position(self):
        return self.position

    def get_velocity(self):
        return self.velocity

    def get_acceleration(self):
        return self.acceleration


# DRONE

class drone:
    def __init__(self, target=np.array((0, 0, 0))):
        self.m = mujoco.MjModel.from_xml_path(str(INDOOR_ENV_XML))
        self.d = mujoco.MjData(self.m)

        self.planner = dummyPlanner(target=target)
        self.sensor  = dummySensor(self.d)

        # --- Inner control ---
        self.pid_alt   = PID(5.50844, 0.57871, 1.2,      setpoint=0)
        self.pid_roll  = PID(2.6785,  0.56871, 1.2508,   setpoint=0, output_limits=(-1, 1))
        self.pid_pitch = PID(2.6785,  0.56871, 1.2508,   setpoint=0, output_limits=(-1, 1))
        self.pid_yaw   = PID(0.54,    0.0,     5.358333, setpoint=0, output_limits=(-3, 3))

        # --- Outer control ---
        self.pid_v_x = PID(0.1, 0.003, 0.02, setpoint=0, output_limits=(-0.5, 0.5))
        self.pid_v_y = PID(0.1, 0.003, 0.02, setpoint=0, output_limits=(-0.5, 0.5))
        self.current_yaw = 0.0

    def reset_pose(self, x, y, z):
        """Đặt vị trí ban đầu của drone (freejoint: qpos[0:7] = [x,y,z, qw,qx,qy,qz])."""
        self.d.qpos[0] = x
        self.d.qpos[1] = y
        self.d.qpos[2] = z
        self.d.qpos[3] = 1.0   # quaternion w (no rotation)
        self.d.qpos[4] = 0.0
        self.d.qpos[5] = 0.0
        self.d.qpos[6] = 0.0
        self.d.qvel[:] = 0.0
        mujoco.mj_forward(self.m, self.d)

    def update_outer_control(self):
        pos      = self.sensor.get_position()
        v        = self.sensor.get_velocity()
        location = pos[:3]

        qw, qx, qy, qz = pos[3], pos[4], pos[5], pos[6]
        _, _, self.current_yaw = quat_to_euler(qw, qx, qy, qz)

        # Planner return desired velocity in World Frame
        vel_world = self.planner(loc=location)
        vx_world, vy_world = vel_world[0], vel_world[1]

        # BODY FRAME TRANSFORMATION 
        vx_body_setpoint, vy_body_setpoint = world_vel_to_body_vel(
            vx_world, vy_world, self.current_yaw
        )
        vx_body_measured, vy_body_measured = world_vel_to_body_vel(
            v[0], v[1], self.current_yaw
        )

        # Update alt setpoint
        self.pid_alt.setpoint = self.planner.get_alt_setpoint(location)

        # Set velocity setpoint in Body Frame
        self.pid_v_x.setpoint = vx_body_setpoint
        self.pid_v_y.setpoint = vy_body_setpoint

        # PID calculate angle command based on body-frame velocity error
        angle_pitch =  self.pid_v_x(vx_body_measured)
        angle_roll  = -self.pid_v_y(vy_body_measured)

        self.pid_pitch.setpoint = angle_pitch
        self.pid_roll.setpoint  = angle_roll

    def update_inner_control(self):
        # Inner loop: read Quaternion → Euler then put into PID
        pos = self.sensor.get_position()
        alt = pos[2]

        # Convert Quaternion → Euler
        qw, qx, qy, qz = pos[3], pos[4], pos[5], pos[6]
        roll, pitch, yaw = quat_to_euler(qw, qx, qy, qz)

        cmd_thrust = self.pid_alt(alt) + 3.2495

        cmd_roll   = -self.pid_roll(roll)
        cmd_pitch  =  self.pid_pitch(pitch)
        cmd_yaw    = -self.pid_yaw(yaw)

        out = self.compute_motor_control(cmd_thrust, cmd_roll, cmd_pitch, cmd_yaw)
        self.d.ctrl[:4] = out

    def compute_motor_control(self, thrust, roll, pitch, yaw):
        return [
            thrust + roll + pitch - yaw,
            thrust - roll + pitch + yaw,
            thrust - roll - pitch - yaw,
            thrust + roll - pitch + yaw,
        ]


# MAIN

# Spawn drone at living room center, 1 m above floor
my_drone = drone(target=np.array((3.0, 5.0, 1.0)))
kb = KeyboardController(step_size=0.3, alt_step=0.3)
kb.start()

current_target = np.array([3.0, 5.0, 1.0])
KEYBOARD_UPDATE_INTERVAL = 30

with mujoco.viewer.launch_passive(my_drone.m, my_drone.d) as viewer:
    # Teleport drone to living room center before simulation starts
    my_drone.reset_pose(3.0, 5.0, 1.0)
    time.sleep(5)
    start = time.time()
    step  = 1

    while viewer.is_running() and time.time() - start < 300:
        step_start = time.time()

        # Update target from keyboard (World Frame)
        if step % KEYBOARD_UPDATE_INTERVAL == 0:
            delta = kb.get_target_delta_world()
            if np.any(delta != 0):
                current_target = current_target + delta
                current_target[2] = np.clip(current_target[2], 0.2, 5.0)
                my_drone.planner.update_target(current_target)
                print(f"[Target] {current_target}  |  Yaw: {np.degrees(my_drone.current_yaw):.1f}°")

        # Outer loop (every 20 steps)
        if step % 20 == 0:
            my_drone.update_outer_control()

        # Inner loop (every step)
        my_drone.update_inner_control()

        mujoco.mj_step(my_drone.m, my_drone.d)

        with viewer.lock():
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(my_drone.d.time % 2)

        viewer.sync()

        step += 1
        time_until_next_step = my_drone.m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

kb.stop()