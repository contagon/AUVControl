from matplotlib.pyplot import plot
import numpy as np
import holoocean
import json
from tqdm import tqdm

from estimation import Observer
from plotter import Plotter
from controller import Controller
from tools import State
from holoocean_config import scenario

np.set_printoptions(suppress=True, formatter={"float_kind": f"{{:0.2f}}".format})

# Simulation parameters
num_seconds = 50
view = True

# Install simulation environments
if "Ocean" not in holoocean.installed_packages():
    holoocean.install("Ocean")

# Set everything up
observer = Observer()
plotter = Plotter(["True", "Estimated", "Commanded"])
controller = Controller()

# Load in HoloOcean info
ts = 1 / scenario["ticks_per_sec"]
num_ticks = int(num_seconds / ts)

u = np.zeros(8)
x_d = State(np.array([2.0, 2, -2, 0, 0, 0, 0, 20, 45, 0, 0, 0]))
with holoocean.make(scenario_cfg=scenario, show_viewport=view) as env:
    for i in tqdm(range(num_ticks)):
        # Tick environment
        env.act("auv0", u)
        sensors = env.tick()

        # Pluck true state from sensors
        t = sensors["t"]
        true_state = State(sensors)

        # Estimate State
        est_state = observer.tick(sensors, ts)

        # Autopilot Commands
        u = controller.u(est_state, x_d)

        # Update plots
        plotter.add_timestep(t, [true_state, est_state, x_d])
        if i % 200 == 0:
            plotter.update_plots()
