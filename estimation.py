import numpy as np
from inekf import InertialProcess, DVLSensor, DepthSensor, GenericMeasureModel
from inekf import SE3, SO3, ERROR, InEKF

class Observer:
    def __init__(self, error=ERROR.RIGHT):
        # set up initial state
        xi = np.array([0, 0, 0,   # rotation
                        0, 0, 0,             # velocity
                        0, 0, -5,     # position
                        0,0,0,0,0,0])               # bias
        s = np.array([0.274156, 0.274156, 0.274156, 
                        1.0, 1.0, 1.0, 
                        0.01, 0.01, 0.01, 
                        0.000025, 0.000025, 0.000025, 
                        0.0025, 0.0025, 0.0025])
        x0 = SE3[2,6](xi, np.diag(s))

        # set up DVL
        dvlR = SO3(np.array([[0.000, -0.995, 0.105],
                        [0.999, 0.005, 0.052],
                        [-0.052, 0.104, 0.993]]))
        dvlT = np.array([-0.17137, 0.00922, -0.33989])
        self.dvl = DVLSensor(dvlR, dvlT)
        self.dvl.setNoise(.0101*2.6, .005*(3.14/180)*np.sqrt(200.0))

        # set up depth sensor
        self.depth = DepthSensor(51.0 * (1.0/100) * (1.0/2))

        # gps sensor
        b = np.array([0,0,0,0,1])
        noise = np.eye(3)*.15
        self.gps = GenericMeasureModel[SE3[2,6]](b, noise, ERROR.LEFT)
        
        # Initialize sensors
        self.iekf = InEKF[InertialProcess](x0, error)
        self.iekf.addMeasureModel('DVL', self.dvl)
        self.iekf.addMeasureModel('Depth', self.depth)
        self.iekf.addMeasureModel('GPS', self.gps)

        self.last_omega = np.zeros(3)

    def predict_imu(self, imu, dt):
        # WHICH ORDER ARE THESE IN?
        self.last_omega = imu[:3]
        return self.iekf.Predict(imu.flatten()[:6], dt)

    def update_gps(self, gps):
        return self.iekf.Update(gps, "GPS")

    def update_depth(self, depth):
        return self.iekf.Update(depth, "Depth")

    def update_dvl(self, dvl):
        dvl = np.append(dvl, self.last_omega)
        return self.iekf.Update(dvl, "DVL")

    def tick(self, state, ts):
        if 'IMUSensor' in state:
            est_state = self.predict_imu(state["IMUSensor"], ts)
        if 'DVLSensor' in state:
            est_state = self.update_dvl(state['DVLSensor'])
        if 'GPSSensor' in state:
            est_state = self.update_gps(state['GPSSensor'])
        if 'DepthSensor' in state:
            est_state = self.update_depth(state["DepthSensor"])
        return est_state


