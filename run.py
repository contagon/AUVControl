import numpy as np
import holoocean
from tqdm import tqdm

from estimation import Observer
from plotter import Plotter
from controller import Controller
from planner import Planner
from tools import State, make_route
from holoocean_config import scenario
import argparse

np.set_printoptions(suppress=True, formatter={"float_kind": f"{{:0.2f}}".format})

def main(num_seconds, show, plot, verbose, route):
    # Install simulation environments
    if "Ocean" not in holoocean.installed_packages():
        holoocean.install("Ocean")

    # Load in HoloOcean info
    ts = 1 / scenario["ticks_per_sec"]
    num_ticks = int(num_seconds / ts)

    # Set everything up
    observer = Observer()
    if plot:
        plotter = Plotter(["True", "Estimated", "Desired"])
    controller = Controller()
    pos, rot = make_route(route, num_seconds)
    planner = Planner(pos, rot)

    # Run simulation!
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
            if plot:
                plotter.add_timestep(t, [true_state, est_state, des_state])
                if i % 100 == 0:
                    plotter.update_plots()
            if show:
                if i % 10 == 0:
                    planner.draw_step(env, t, ts*10)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run AUV simulation.')
    parser.add_argument('-s', '--show', action='store_true', help='Show viewport')
    parser.add_argument('-p', '--plot', action='store_true', help='Plot data')
    parser.add_argument('-v', '--verbose', action='store_true', help='Print holoocean output')
    parser.add_argument('-n', '--num_seconds', default=50, type=float, help='Length to run simulation for')
    parser.add_argument('-r', '--route', default="wave", type=str, help='Length to run simulation for')

    args = parser.parse_args()
    main(**vars(args))