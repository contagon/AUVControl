from matplotlib.pyplot import plot
import numpy as np
import holoocean
import json
from tqdm import tqdm

from estimation import Observer
from plotter import Plotter
from controller import Controller
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
plotter = Plotter()
controller = Controller()

# Load in HoloOcean info
ts = 1 / scenario["ticks_per_sec"]
num_ticks = int(num_seconds / ts)

F = np.array([0, 0, 10, 0, 0, 0])
command = controller.Minv@F

with holoocean.make(scenario_cfg=scenario, show_viewport=view) as env:
    for i in tqdm(range(num_ticks)):
        # Tick environment
        env.act("auv0", command)
        state = env.tick()

        # Estimate State
        est_state = observer.tick(state, ts)

        # Autopilot Commands

        # Update plots
        plotter.add_timestep(state, est_state)
        if i % 200 == 0:
            plotter.update_plots()
