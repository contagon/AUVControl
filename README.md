# AUVControl
Control of a hovering AUV simulated using HoloOcean. This implements the full robotics stack 
include various algorithms for state estimation, low-level control, and planning. HoloOcean is simple
to use, and the HoveringAUV is easy to control, making this a perfect platform for experimenting and 
testing of various robotics algorithms.

# Python Module
The code is organized into the python module `auv_control`. This consists of 3 submodules - 
`estimation`, `control`, and `planning` - along with various other helper functions. The following algorithms have been implemented:

- Base module
    - State class - wrapper class to standardize how our state is represented.
    - scenario - python dictionary with the example HoloOcean scenario to run.
- Estimation module
    - `InEKF` - Implementation of the invariant EKF with the IMU sensor as the process model
        and GPS, DVL, magnetometer, and pressure sensor as the update step.
- Control Module
    - `LQR` - Simple LQR controller using linearized state dynamics and euler angles. (Simple
    improvement to this would be something using Lie Groups)
- Planning Module
    - `RRT` - Loads up various obstacles in the environment, and plans a trajectory from one side of the environment to the other using RRT.
    - `Traj` - Various predefined trajectories to test controllers on.

# Running

To run the simulation, first install all dependencies
```
pip install -r requirements.txt
```

Then simply run the script
```
python run.py -s -p
```
where `-s` shows the simulation and `-p` shows the plotter. By default these are both off. On first run, it will download the HoloOcean binaries which may take a minute. To see a full list of simulation options, run
```
python run.py -h
```
