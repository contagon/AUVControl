import numpy as np
from scipy.spatial.transform import Rotation
from inekf import SE3

class State:
    def __init__(self, state, last_meas_omega):
        self.vec = np.zeros(12)
        self.mat = np.eye(5)

        if isinstance(state, dict):
            self.vec[0:3] = state["PoseSensor"][:3,3]
            self.vec[3:6] = state["VelocitySensor"]
            self.vec[6:9] = rot_to_rpy(state["PoseSensor"][:3,:3])
            self.vec[9:12] = state["IMUSensorClean"][1]

            self.mat[:3,:3] = state["PoseSensor"][:3,:3]
            self.mat[:3,3] = state["VelocitySensor"]
            self.mat[:3,4] = state["PoseSensor"][:3,3]

        if isinstance(state, SE3[2,6]):
            self.vec[0:3] = state[1]
            self.vec[3:6] = state[0]
            self.vec[6:9] = rot_to_rpy(state.State[:3,:3].copy())
            self.vec[9:12] = last_meas_omega - state.Aug[:3]

            self.mat = state.State

def rot_to_rpy(mat):
    return Rotation.from_matrix(mat).as_euler("xyz")*180/np.pi