"""ROS-agnostic MuJoCo quadrotor simulation + cascaded PID controller.

This module has **no ROS imports** so it can be shared verbatim by the ROS 1
(rospy) and ROS 2 (rclpy) wrapper nodes, and exercised in plain Python tests.

The control stack mirrors the standalone demos:
    outer loop (position -> world velocity -> body velocity -> attitude setpoint)
    inner loop (attitude/altitude -> motor thrusts via X-quad mixing)
"""
import numpy as np
import mujoco
from simple_pid import PID


def quat_to_euler(qw, qx, qy, qz):
    """Convert a (w, x, y, z) quaternion to (roll, pitch, yaw) in radians."""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0)
    pitch = np.arcsin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def world_vel_to_body_vel(vx_world, vy_world, yaw):
    """Rotate a world-frame XY velocity into the body frame given yaw."""
    cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)
    vx_body = cos_yaw * vx_world + sin_yaw * vy_world
    vy_body = -sin_yaw * vx_world + cos_yaw * vy_world
    return vx_body, vy_body


class PositionPlanner:
    """Turns a target position into a world-frame velocity + altitude setpoint."""

    def __init__(self, target, vel_limit=2.0):
        self.target = np.asarray(target, dtype=float)
        self.vel_limit = vel_limit
        self.pid_x = PID(2, 0.15, 1.5, setpoint=self.target[0],
                         output_limits=(-vel_limit, vel_limit))
        self.pid_y = PID(2, 0.15, 1.5, setpoint=self.target[1],
                         output_limits=(-vel_limit, vel_limit))

    def velocity(self, loc):
        return np.array([self.pid_x(loc[0]), self.pid_y(loc[1]), 0.0])

    def alt_setpoint(self, loc):
        dz = self.target[2] - loc[2]
        if abs(dz) > 0.5:
            steps = max(int((abs(dz) / self.vel_limit) / 0.25), 1)
            return loc[2] + 2.0 * (dz / steps)
        return self.target[2]

    def update_target(self, target):
        self.target = np.asarray(target, dtype=float)
        self.pid_x.setpoint = self.target[0]
        self.pid_y.setpoint = self.target[1]


class QuadrotorSim:
    """Loads a MuJoCo model and flies it toward a target with cascaded PID control.

    Call :meth:`step` once per simulation tick (every ``timestep`` seconds) and
    read :meth:`get_state` to publish the result. ``set_target`` replaces the
    keyboard teleop of the standalone demos.
    """

    def __init__(self, model_path, target=(0.0, 0.0, 1.0),
                 hover_thrust=2.3789, max_thrust=8.0, outer_decimation=20):
        self.m = mujoco.MjModel.from_xml_path(str(model_path))
        self.d = mujoco.MjData(self.m)
        self.hover_thrust = hover_thrust
        self.max_thrust = max_thrust
        self.planner = PositionPlanner(target)
        self.current_yaw = 0.0

        # Inner loop (attitude + altitude)
        self.pid_alt = PID(5.50844, 0.57871, 1.2, setpoint=0)
        self.pid_roll = PID(2.6785, 0.56871, 1.2508, setpoint=0, output_limits=(-1.0, 1.0))
        self.pid_pitch = PID(2.6785, 0.56871, 1.2508, setpoint=0, output_limits=(-1.0, 1.0))
        self.pid_yaw = PID(0.54, 0.0, 5.358333, setpoint=0, output_limits=(-3.0, 3.0))

        # Outer loop (body-frame velocity -> attitude setpoint)
        self.pid_v_x = PID(0.1, 0.003, 0.02, setpoint=0, output_limits=(-0.5, 0.5))
        self.pid_v_y = PID(0.1, 0.003, 0.02, setpoint=0, output_limits=(-0.5, 0.5))

        self._outer_decimation = outer_decimation
        self._k = 0

    @property
    def timestep(self):
        return self.m.opt.timestep

    def reset_pose(self, x, y, z):
        self.d.qpos[0:3] = [x, y, z]
        self.d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        self.d.qvel[:] = 0.0
        mujoco.mj_forward(self.m, self.d)

    def set_target(self, x, y, z):
        self.planner.update_target((x, y, z))

    def _update_outer_control(self):
        pos, v = self.d.qpos, self.d.qvel
        loc = pos[:3]
        _, _, self.current_yaw = quat_to_euler(pos[3], pos[4], pos[5], pos[6])

        vel_world = self.planner.velocity(loc)
        vx_sp, vy_sp = world_vel_to_body_vel(vel_world[0], vel_world[1], self.current_yaw)
        vx_meas, vy_meas = world_vel_to_body_vel(v[0], v[1], self.current_yaw)

        self.pid_alt.setpoint = self.planner.alt_setpoint(loc)
        self.pid_v_x.setpoint = vx_sp
        self.pid_v_y.setpoint = vy_sp
        self.pid_pitch.setpoint = self.pid_v_x(vx_meas)
        self.pid_roll.setpoint = -self.pid_v_y(vy_meas)

    def _update_inner_control(self):
        pos = self.d.qpos
        roll, pitch, yaw = quat_to_euler(pos[3], pos[4], pos[5], pos[6])
        thrust = self.pid_alt(pos[2]) + self.hover_thrust
        cmd_roll = -self.pid_roll(roll)
        cmd_pitch = self.pid_pitch(pitch)
        cmd_yaw = self.pid_yaw(yaw)
        self.d.ctrl[:4] = self._mix(thrust, cmd_roll, cmd_pitch, cmd_yaw)

    def _mix(self, thrust, roll, pitch, yaw):
        motors = [
            thrust - pitch - roll - yaw,
            thrust - pitch + roll + yaw,
            thrust + pitch - roll + yaw,
            thrust + pitch + roll - yaw,
        ]
        return [float(np.clip(m, 0.0, self.max_thrust)) for m in motors]

    def step(self):
        """Advance the simulation by one ``timestep``."""
        if self._k % self._outer_decimation == 0:
            self._update_outer_control()
        self._update_inner_control()
        mujoco.mj_step(self.m, self.d)
        self._k += 1

    def get_state(self):
        """Return the current drone state as plain numpy arrays."""
        pos, vel = self.d.qpos, self.d.qvel
        return {
            "position": np.array(pos[0:3]),
            "orientation": np.array(pos[3:7]),       # quaternion (w, x, y, z)
            "linear_velocity": np.array(vel[0:3]),
            "angular_velocity": np.array(vel[3:6]),
            "time": float(self.d.time),
        }
