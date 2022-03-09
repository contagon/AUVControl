import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation
import seaborn as sns
sns.set(context='paper', style='whitegrid', font_scale=0.8)

class Plotter:
    def __init__(self):
        self.t = []
        self.data = None
        self.est_data = None

        self.num_row = 5
        self.num_col = 3

        plt.ion()
        self.fig, self.ax = plt.subplots(self.num_row, self.num_col, figsize=(6,8), sharex=True)
        plt.tight_layout()

        self.line_state = [[] for _ in range(self.num_row)]
        self.line_est_state = [[] for _ in range(self.num_row)]
        for i in range(self.num_row):
            for j in range(self.num_col):
                p, = self.ax[i,j].plot([], [], c='r')
                p_est, = self.ax[i,j].plot([], [], c='g')

                self.line_state[i].append(p)
                self.line_est_state[i].append(p_est)
                
        names = ["RPY", "Position", "Velocity", "Bias - Omega", "Bias - Acceleration"]
        for i in range(self.num_row):
            self.ax[i,1].set_title(names[i])

    def add_timestep(self, state, est_state):
        self.t.append(state["t"])

        rpy = self._rot_to_rpy(state["PoseSensor"][:3, :3])
        p = state["PoseSensor"][:3, 3]
        v = state["VelocitySensor"]
        bias = state["IMUSensor"][2:]
        new_state = np.block([[rpy], [p], [v], [bias[0]], [bias[1]]])

        est_rpy = self._rot_to_rpy(est_state.State[:3, :3].copy())
        est_p = est_state.State[:3, 4]
        est_v = est_state.State[:3, 3]
        est_bias = est_state.Aug
        new_est_state = np.block(
            [[est_rpy], [est_p], [est_v], [est_bias[:3]], [est_bias[3:]]]
        )

        if self.data is None:
            self.data = new_state
            self.est_data = new_est_state
        else:
            self.data = np.dstack((self.data, new_state))
            self.est_data = np.dstack((self.est_data, new_est_state))

    def _rot_to_rpy(self, mat):
        return Rotation.from_matrix(mat).as_euler('xyz')

    def update_plots(self):
        for i in range(self.num_row):
            for j in range(self.num_col):
                self.line_state[i][j].set_data(self.t, self.data[i,j])
                self.line_est_state[i][j].set_data(self.t, self.est_data[i,j])

                self.ax[i,j].relim()
                self.ax[i,j].autoscale_view()

        # self.fig.canvas.draw()
        self.fig.canvas.flush_events()