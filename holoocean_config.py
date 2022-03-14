scenario = {
    "name": "Hovering",
    "package_name": "Ocean",
    "world": "SimpleUnderwater",
    "main_agent": "auv0",
    "ticks_per_sec": 200,
    "frames_per_sec": False,
    "agents":[
        {
            "agent_name": "auv0",
            "agent_type": "HoveringAUV",
            "sensors": [
                {
                    "sensor_type": "PoseSensor",
                    "socket": "COM"
                },
                {
                    "sensor_type": "VelocitySensor",
                    "socket": "COM"
                },
                {
                    "sensor_type": "IMUSensor",
                    "socket": "COM",
                    "Hz": 200,
                    "configuration": {
                        "AccelSigma": 0.00277,
                        "AngVelSigma": 0.00123,
                        "AccelBiasSigma": 0.0,
                        "AngVelBiasSigma": 0.0,
                        # "AccelBiasSigma": 0.00141,
                        # "AngVelBiasSigma": 0.00388,
                        "ReturnBias": True
                    }
                },
                {
                    "sensor_type": "GPSSensor",
                    "socket": "COM",
                    "Hz": 2,
                    "configuration":{
                        "Sigma": 0.3,
                        "Depth": 3,
                        "DepthSigma": 1
                    }
                },
                {
                    "sensor_type": "OrientationSensor",
                    "sensor_name": "CompassSensor",
                    "socket": "COM",
                    "Hz": 50,
                    "configuration":{
                        "Sigma": 0.05
                    }
                },
                {
                    "sensor_type": "DVLSensor",
                    "socket": "COM",
                    "Hz": 5,
                    "configuration": {
                        "Elevation": 22.5,
                        "VelSigma": 0.02626,
                        "ReturnRange": False,
                        "MaxRange": 50,
                        "RangeSigma": 0.1
                    }
                },
                {
                    "sensor_type": "DepthSensor",
                    "socket": "COM",
                    "Hz": 50,
                    "configuration": {
                        "Sigma": 0.255
                    }
                }
            ],
            "control_scheme": 0,
            "location": [0.0, 0.0, -2.0],
            "rotation": [0.0, 0.0, 0.0]
        }
    ],

    "window_width":  1280,
    "window_height": 720
}
