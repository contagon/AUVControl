import numpy as np
from tools import State

class Planner:
    def __init__(self, pos_func, rot_func):
        self.pos_func = pos_func
        self.rot_func = rot_func
        self.eps = 1e-5

    def _traj(self, t):
        """Get desired trajectory at time t"""
        pos = self.pos_func(t)
        rot = self.rot_func(t)

        lin_vel = (self.pos_func(t+self.eps) - pos) / self.eps
        ang_vel = (self.rot_func(t+self.eps) - rot) / self.eps

        return np.concatenate((pos, lin_vel, rot, ang_vel)).T

    def tick(self, t):
        """Gets desired trajectory at time t"""
        if not isinstance(t, float):
            raise ValueError("Can't tick with an array")

        return State(self._traj(t))

    def draw_step(self, env, t, ts):
        """Draw points on the next 5 steps"""
        des = self._traj(t)
        env.draw_point(des[:3].tolist(), color=[0,255,0], thickness=20, lifetime=ts)

    def draw_traj(self, env, t):
        """Makes trajectory line show up"""
        # Get all positions
        t = np.arange(0, t, 0.5)
        des_state = self._traj(t)
        des_pos = des_state[:,0:3]

        # Draw line between each
        for i in range(len(des_pos)-1):
            env.draw_line(des_pos[i].tolist(), des_pos[i+1].tolist(), thickness=2.0, lifetime=0.0)