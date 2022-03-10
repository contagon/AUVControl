import numpy as np
from inekf import (
    DVLSensor,
    DepthSensor,
    GenericMeasureModel,
    MeasureModel,
    InertialProcess,
)
from inekf import SE3, SO3, ERROR, InEKF
from scipy.linalg import expm

# Import parameters from scenario file
from holoocean_config import scenario

sensors_params = scenario["agents"][0]["sensors"]
sensors_params = {d["sensor_type"]: d for d in sensors_params}
sensors_params["CompassSensor"] = sensors_params["OrientationSensor"]
np.set_printoptions(suppress=True, formatter={"float_kind": f"{{:0.2f}}".format})


class CompassSensor(MeasureModel[SE3[2, 6]]):
    def __init__(self, std):
        super().__init__()

        self.error = ERROR.RIGHT

        self.M = np.eye(3) * std**2

        H = np.zeros((3, 15))
        H[1, 2] = -1
        H[2, 1] = 1
        self.H = H

    def processZ(self, z, state):
        new_z = np.zeros(5)
        new_z[:3] = z
        return new_z

    def calcV(self, z, state):
        V = state.State[:3, :] @ z - np.array([1, 0, 0])
        return V


class Observer:
    def __init__(self, error=ERROR.RIGHT):
        # set up initial state
        xi = np.zeros(15)
        xi[8] = -2  # depth

        s = np.zeros(15)
        s[0:3] = 0.1  # rotation
        s[3:6] = 1.0  # position
        s[6:9] = 0.01  # velocity
        s[9:12] = 0.000025  # bias
        s[12:15] = 0.0025
        x0 = SE3[2, 6](xi, np.diag(s))

        # set up DVL
        dvlR = SO3()
        dvlT = np.zeros(3)
        self.dvl = DVLSensor(dvlR, dvlT)
        self.dvl.setNoise(
            sensors_params["DVLSensor"]["configuration"]["VelSigma"],
            sensors_params["IMUSensor"]["configuration"]["AngVelSigma"],
        )

        # set up depth sensor
        self.depth = DepthSensor(
            sensors_params["DepthSensor"]["configuration"]["Sigma"]
        )

        # set up compass sensor
        self.compass = CompassSensor(
            sensors_params["CompassSensor"]["configuration"]["Sigma"]
        )

        # gps sensor
        b = np.array([0, 0, 0, 0, 1])
        noise = np.eye(3) * sensors_params["GPSSensor"]["configuration"]["Sigma"] ** 2
        self.gps = GenericMeasureModel[SE3[2, 6]](b, noise, ERROR.LEFT)

        # Initialize sensors
        self.iekf = InEKF[InertialProcess](x0, error)
        self.iekf.addMeasureModel("DVL", self.dvl)
        self.iekf.addMeasureModel("Depth", self.depth)
        self.iekf.addMeasureModel("GPS", self.gps)
        self.iekf.addMeasureModel("Compass", self.compass)

        self.iekf.pModel.setGyroNoise(
            sensors_params["IMUSensor"]["configuration"]["AngVelSigma"]
        )
        self.iekf.pModel.setAccelNoise(
            sensors_params["IMUSensor"]["configuration"]["AccelSigma"]
        )
        self.iekf.pModel.setGyroBiasNoise(
            sensors_params["IMUSensor"]["configuration"]["AngVelBiasSigma"]
        )
        self.iekf.pModel.setAccelBiasNoise(
            sensors_params["IMUSensor"]["configuration"]["AccelBiasSigma"]
        )

        self.last_omega = np.zeros(3)

    def predict_imu(self, imu, dt):
        a = imu[0]
        omega = imu[1]
        u = np.append(omega, a)
        self.last_omega = omega
        return self.iekf.Predict(u, dt)

    def update_gps(self, gps):
        return self.iekf.Update(gps, "GPS")

    def update_depth(self, depth):
        return self.iekf.Update(depth, "Depth")

    def update_dvl(self, dvl):
        dvl = np.append(dvl, self.last_omega)
        return self.iekf.Update(dvl, "DVL")

    def update_compass(self, compass):
        return self.iekf.Update(compass, "Compass")

    def tick(self, state, ts):
        if "IMUSensor" in state:
            est_state = self.predict_imu(state["IMUSensor"], ts)
        if "DVLSensor" in state:
            est_state = self.update_dvl(state["DVLSensor"])
        if "DepthSensor" in state:
            est_state = self.update_depth(state["DepthSensor"])
        if "GPSSensor" in state:
            est_state = self.update_gps(state["GPSSensor"])
        if "CompassSensor" in state:
            z = np.linalg.inv(state["CompassSensor"])[:3, 0]
            z += np.random.normal(
                0, sensors_params["CompassSensor"]["configuration"]["Sigma"], 3
            )
            est_state = self.update_compass(z)

        return est_state
