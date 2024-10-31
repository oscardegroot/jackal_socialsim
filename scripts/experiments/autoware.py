import sys, os
import numpy as np
import matplotlib.pyplot as plt

sys.path.append("..")
from load_data import get_all_experiments_for_scenario
from table import create_table, load_default_settings
from plot import plot_agent_trajectories, plot_agent_trajectories_for_all_experiments


def one_figure(experiments, base_folder, scenario):
    for experiment in experiments:
                plot_agent_trajectories_for_all_experiments(base_folder, scenario, experiment, translate_to_zero=True,
                                                            xlim=[-50, 2], ylim=[-5, 12])

def figure_per_experiment(experiments, base_folder, scenario):
    # experiments = get_all_experiments_for_scenario(base_folder, scenario)#, filter=["none"])
    for experiment in experiments:
        plot_agent_trajectories(base_folder, scenario, experiment, translate_to_zero=True, xlim=[-2, 50], ylim=[-20, 10])


def table(experiments, base_folder, scenario):

    def caption(num_experiments, num_scenarios):
        scenario_pedestrians = scenario.split('_')[-1].split('.')[0]
        if "empty" in scenario:
             scenario_pedestrians = 0
             
        return f"\\caption{{Scenario: ${scenario_pedestrians}$ randomized pedestrians over $15$ experiments. Reporting task duration, minimum distance to pedestrians, collisions, time-outs (vehicle did not reach the goal in time) and average velocity.}}\\resizebox{{\\textwidth}}{{!}}{{%"

    def add_table_data(table):
        table.add_data("Method", lambda metrics, highlight: table.table_settings["clean_method_name"](metrics["experiment"]), align="l")

        table.add_data("Dur. [s]",
                        lambda metrics, highlight: f'{highlight(np.mean(metrics["metric_duration"]), 1)} ({np.std(metrics["metric_duration"]):.1f})')

        if not "empty" in scenario:
            table.add_data("Min Dist. [m]",
                            lambda metrics, highlight: f'{highlight(np.mean(metrics["obstacle_clearance"]["min"]), 2)} ({np.std(metrics["obstacle_clearance"]["min"]):.2f})')
        
        
        table.add_data("Collisions",
                        lambda metrics, highlight: f'{highlight(np.sum([1 if m > 0 else 0 for m in metrics["metric_collisions"]]), 0)}')
        # table.add_data("Mean Clearance. [m]",
                    #    lambda metrics, highlight: f'{highlight(np.mean(metrics["obstacle_clearance"]["mean"]), 2)} ({np.std(metrics["obstacle_clearance"]["mean"]):.2f})')

        table.add_data("Timeouts",
                        lambda metrics, highlight: f'{highlight(np.sum([1 if m > 0 else 0 for m in metrics["metric_timeout"]]), 0)}')

        table.add_data("Avg. Velocity [m/s]",
                        lambda metrics, highlight: f'{highlight(np.mean(metrics["velocity"]["mean"]), 2)}', highlight_select=max)
        # table.add_data("Runtime (Max) [ms]",
                    #    lambda metrics, highlight: f'{highlight(metrics["runtime"]["mean"] * 1000., 0)} ({metrics["runtime"]["max"] * 1000.:.0f})')

    settings = load_default_settings([scenario])
    settings["caption"] = caption
    settings["add_table_data"] = add_table_data

    create_table(base_folder, scenario, experiments, settings, verbose=False)

if __name__ == '__main__':
    choice = sys.argv[1]

    scenario = sys.argv[2]
    base_folder="/workspaces/autoware_iv/src/universe/external/planning/jackal_socialsim/"
    experiments = get_all_experiments_for_scenario(base_folder, scenario)
    print(experiments)

    if choice.lower() == "trajectories":
        one_figure(experiments, base_folder, scenario)

    if choice.lower() == "single_trajectory":
        figure_per_experiment(experiments, base_folder, scenario)

    if choice.lower() == "table":
         table(experiments, base_folder, scenario)

    if choice.lower() == "all":
        one_figure(experiments, base_folder, scenario)
        figure_per_experiment(experiments, base_folder, scenario)
        table(experiments, base_folder, scenario)
        



        
