import numpy as np
import holoocean
from estimation import Observer
import json
from tqdm import tqdm

# Simulation parameters
num_seconds = 2
view = False

# Set everything up
observer = Observer()

# Load in HoloOcean info
scenario = json.load(open("holoocean_config.json"))
ts = 1 / scenario["ticks_per_sec"]
num_ticks = int(num_seconds / ts)

command = [5, 5, 5, 5, 0, 0, 0, 0]
with holoocean.make(scenario_cfg=scenario, show_viewport=view) as env:
    for i in tqdm(range(num_ticks)):
        # Tick environment
        env.act("auv0", command)
        state = env.tick()

        # Estimate State
        est_state = observer.tick(state, ts)

        # Autopilot Commands

        # Update plots
