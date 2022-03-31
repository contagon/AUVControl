import numpy as np
from auv_control import State

class Traj:
    def __init__(self, route, num_seconds):
        pos, rot = self._make_route(route, num_seconds)
        self.pos_func = pos
        self.rot_func = rot
        self.eps = 1e-5

    def _traj(self, t):
        """Get desired trajectory at time t"""
        pos = self.pos_func(t)
        rot = self.rot_func(t)

        lin_vel = (self.pos_func(t+self.eps) - pos) / self.eps
        ang_vel = (self.rot_func(t+self.eps) - rot) / self.eps

        return np.hstack((pos, lin_vel, rot, ang_vel))

    def tick(self, t):
        """Gets desired trajectory at time t"""
        if not isinstance(t, float):
            raise ValueError("Can't tick with an array")

        return State(self._traj(t))

    def draw_step(self, env, t, ts):
        """Draw points on the next steps"""
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

    def _make_route(self, route, num_seconds):
        # Setup trajectory
        if route == "helix":
            R = 3
            tau = 2
            pos = lambda t: np.array([R*np.cos(t*tau*2*np.pi/num_seconds), R*np.sin(t*tau*2*np.pi/num_seconds), -5-0.05*t]).T
            rot = lambda t: np.array([0*t, 15-30*t/num_seconds, 170+t*tau*360/num_seconds]).T
        
        elif route == "wave":
            pos = lambda t: np.array([t/2, 0*t, -2*np.cos(t*4*2*np.pi/num_seconds)-3]).T
            rot = lambda t: np.array([0*t, -np.arctan(2*2*np.pi/num_seconds*np.sin(t*4*2*np.pi/num_seconds))*180/np.pi, 0*t]).T
        
        elif route == "square":
            q = num_seconds/4

            def pos(t):
                v = 0.5
                if t <= q:
                    return np.array([0*t, v*t, 0*t-5])
                elif q < t and t <= 2*q:
                    t -= q
                    return np.array([0*t, 0*t + v*q, -v*t-5])
                elif 2*q < t and t <= 3*q:
                    t -= 2*q
                    return np.array([0*t, -v*t + v*q, 0*t - v*q-5])
                elif 3*q < t:
                    t -= 3*q
                    return np.array([0*t, 0*t, v*t - v*q-5])
            pos = np.vectorize(pos, signature='()->(n)')
            

            def rot(t):
                if t <= q:
                    return np.array([0*t, 0*t, 0*t])
                elif q < t and t <= 2*q:
                    return np.array([0*t, 0*t, 90+0*t])
                elif 2*q < t and t <= 3*q:
                    return np.array([0*t, 0*t, 180+0*t])
                elif 3*q < t:
                    return np.array([0*t, 0*t, 270+0*t])
            rot = np.vectorize(rot, signature='()->(n)')

        return pos, rot