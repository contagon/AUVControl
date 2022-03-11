import numpy as np

class Controller:
    def __init__(self):
        self.cob = np.array([-5.96, 0.29, -1.85]) - np.array([-5.9, 0.46, -2.82])
        self.m = 31.02
        self.V = 0.03554577

        self.thruster_p = np.array([[18.18, -22.14, -4],
                                    [18.18, 22.14, -4],
                                    [-18.18, 22.14, -4], # First element should be -31.43
                                    [-18.18, -22.14, -4], # First element should be -31.43
                                    [20.64, -18.23, -0.21], # First element should be 7.39
                                    [20.64, 18.23, -0.21], # First element should be 7.39
                                    [-20.64, 18.23, -0.21],
                                    [-20.64, -18.23, -0.21]])/100

        self.thruster_d = np.array([[0, 0, 1],
                                    [0, 0, 1],
                                    [0, 0, 1],
                                    [0, 0, 1],
                                    [np.sqrt(2), np.sqrt(2), 0],
                                    [np.sqrt(2), -np.sqrt(2), 0],
                                    [np.sqrt(2), np.sqrt(2), 0],
                                    [np.sqrt(2), -np.sqrt(2), 0]])
        
        self.M = np.zeros((6,8))
        for i in range(8):
            self.M[:3,i] = self.thruster_d[i]
            self.M[3:,i] = -np.cross(self.thruster_d[i], self.thruster_p[i])
        print(self.M)

        self.Minv = self.M.T@np.linalg.inv(self.M@self.M.T)
        # self.Minv = np.linalg.inv(self.M.T@self.M)@self.M.T