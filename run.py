from matplotlib.pyplot import plot
import numpy as np
import holoocean
import json
from tqdm import tqdm

from estimation import Observer
from plotter import Plotter
from holoocean_config import scenario

# Simulation parameters
num_seconds = 50
view = False

# Set everything up
observer = Observer()
plotter = Plotter()

# Load in HoloOcean info
ts = 1 / scenario["ticks_per_sec"]
num_ticks = int(num_seconds / ts)

command = np.array([-.1, -.1, -.1, -.1, 4.8, 5, 5, 5])*5
# command = np.zeros(8)
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
