import numpy as np
from scipy.spatial.transform import Rotation
from inekf import SE3

class State:
    def __init__(self, state, last_meas_omega=None):
        self.vec = np.zeros(12)
        self.mat = np.eye(5)
        self.bias = np.zeros(6)

        # True State
        if isinstance(state, dict):
            self.vec[0:3] = state["PoseSensor"][:3,3]
            self.vec[3:6] = state["VelocitySensor"]
            self.vec[6:9] = rot_to_rpy(state["PoseSensor"][:3,:3])
            self.vec[9:12] = state["IMUSensorClean"][1]

            self.bias[0:3] = state["IMUSensor"][3]
            self.bias[3:6] = state["IMUSensor"][2]

            self.mat[:3,:3] = state["PoseSensor"][:3,:3]
            self.mat[:3,3] = state["VelocitySensor"]
            self.mat[:3,4] = state["PoseSensor"][:3,3]

        # Estimated State
        if isinstance(state, SE3[2,6]):
            self.vec[0:3] = state[1]
            self.vec[3:6] = state[0]
            self.vec[6:9] = rot_to_rpy(state.State[:3,:3].copy())

            if last_meas_omega is None:
                raise ValueError("Need a measurement for angular velocity")
            self.vec[9:12] = last_meas_omega - state.Aug[:3]

            self.bias = state.Aug
            self.mat = state.State

        # Commanded State
        if isinstance(state, np.ndarray):
            self.vec = state

    @property
    def data_plot(self):
        return np.append(self.vec[:9], self.bias)

def rot_to_rpy(mat):
    return Rotation.from_matrix(mat).as_euler("xyz")*180/np.pi

def make_route(route, num_seconds):
    # Setup trajectory
    if route == "helix":
        R = 3
        tau = 2
        pos = lambda t: np.array([R*np.cos(t*tau*2*np.pi/num_seconds), R*np.sin(t*tau*2*np.pi/num_seconds), -5-0.05*t])
        rot = lambda t: np.array([0*t, 15-30*t/num_seconds, 170+t*tau*360/num_seconds])
    
    elif route == "wave":
        pos = lambda t: np.array([t/2, 0*t, -2*np.cos(t*4*2*np.pi/num_seconds)-3])
        rot = lambda t: np.array([0*t, -np.arctan(2*2*np.pi/num_seconds*np.sin(t*4*2*np.pi/num_seconds))*180/np.pi, 0*t])
    
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
        vec_pos = np.vectorize(pos, signature='()->(n)')
        pos = lambda t: vec_pos(t).T
        

        def rot(t):
            if t <= q:
                return np.array([0*t, 0*t, 0*t])
            elif q < t and t <= 2*q:
                return np.array([0*t, 0*t, 90+0*t])
            elif 2*q < t and t <= 3*q:
                return np.array([0*t, 0*t, 180+0*t])
            elif 3*q < t:
                return np.array([0*t, 0*t, 270+0*t])
        vec_rot = np.vectorize(rot, signature='()->(n)')
        rot = lambda t: vec_rot(t).T

    return pos, rot