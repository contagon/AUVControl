from matplotlib.pyplot import plot
import numpy as np
import holoocean
import json
from tqdm import tqdm

from estimation import Observer
from plotter import Plotter
from controller import Controller
from planner import Planner
from tools import State
from holoocean_config import scenario
import argparse

np.set_printoptions(suppress=True, formatter={"float_kind": f"{{:0.2f}}".format})

def main(num_seconds, show, verbose):
    # Install simulation environments
    if "Ocean" not in holoocean.installed_packages():
        holoocean.install("Ocean")

    # Load in HoloOcean info
    ts = 1 / scenario["ticks_per_sec"]
    num_ticks = int(num_seconds / ts)

    # Set everything up
    observer = Observer()
    plotter = Plotter(["True", "Estimated", "Desired"])
    controller = Controller()

    # Setup trajectory
    planner = Planner(lambda t: np.array([0.5*t, 5+0*t, -5+0*t]),
                        lambda t: np.array([0*t, 0*t, 3*t]))

    u = np.zeros(8)
    with holoocean.make(scenario_cfg=scenario, show_viewport=show, verbose=verbose) as env:
        planner.draw_traj(env, num_seconds)

        for i in tqdm(range(num_ticks)):
            # Tick environment
            env.act("auv0", u)
            sensors = env.tick()

            # Pluck true state from sensors
            t = sensors["t"]
            true_state = State(sensors)

            # Estimate State
            est_state = observer.tick(sensors, ts)

            # Path planner
            des_state = planner.tick(t)

            # Autopilot Commands
            u = controller.u(est_state, des_state)

            # Update visualization
            plotter.add_timestep(t, [true_state, est_state, des_state])
            if i % 100 == 0:
                plotter.update_plots()
                planner.draw_step(env, t)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run AUV simulation.')
    parser.add_argument('-s', '--show', action='store_true', help='Show viewport')
    parser.add_argument('-v', '--verbose', action='store_true', help='Print holoocean output')
    parser.add_argument('-n', '--num_seconds', default=50, type=float, help='Length to run simulation for')

    args = parser.parse_args()
    main(**vars(args))