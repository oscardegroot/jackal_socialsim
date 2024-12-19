import sys, os
import numpy as np
import matplotlib.pyplot as plt

sys.path.append("..")
from load_data import get_all_experiments_for_scenario
from table import create_table, load_default_settings
from plot import plot_agent_trajectories, plot_agent_trajectories_for_all_experiments
from plot import compare_control_signals
from config import Config



def one_figure(experiments, base_folder, scenario, config):
    for experiment in experiments:
                plot_agent_trajectories_for_all_experiments(base_folder, scenario, experiment, config, translate_to_zero=True,
                                                            xlim=[-50, 2], ylim=[-5, 12])

def figure_per_experiment(experiments, base_folder, scenario, config):
    # experiments = get_all_experiments_for_scenario(base_folder, scenario)#, filter=["none"])
    for experiment in experiments:
        plot_agent_trajectories(base_folder, scenario, experiment, config, translate_to_zero=True, 
                                xlim=[-2, 32], ylim=[-15, 10], remove_first=True)


def control_signals(experiments, base_folder, scenario):
    #  for experiment in experiments:
    styles = {"autoware": "-", "lmpcc": "-", "tmpc": "-", "tmpcnf": "--"}
    colors = {"autoware": "C2", "lmpcc": "C3", "tmpc": "C0", "tmpcnf": "C0"}
    compare_control_signals(base_folder, scenario, experiments, styles=styles, colors=colors, remove_first=True)


def table(experiments, base_folder, scenario, config):

    def caption(num_experiments, num_scenarios):
        scenario_pedestrians = scenario.split('_')[-1].split('.')[0]
        if "empty" in scenario:
             scenario_pedestrians = 0
             
        return f"\\caption{{Scenario: ${scenario_pedestrians}$ randomized pedestrians over ${num_experiments}$ experiments. Reporting task duration, minimum distance to pedestrians, collisions, time-outs (vehicle did not reach the goal in time) and average velocity.}}\\resizebox{{\\textwidth}}{{!}}{{%"

    def add_table_data(table):
        # pass
        table.add_data("Method", lambda metrics, highlight: table.table_settings["clean_method_name"](metrics["experiment"]), align="l")

        table.add_data("Dur. [s]",
                        lambda metrics, highlight: f'{highlight(np.mean(metrics["metric_duration"]), 1)} ({np.std(metrics["metric_duration"]):.1f})')

        # if not "empty" in scenario:
            # table.add_data("Min Dist. [m]",
                            # lambda metrics, highlight: f'{highlight(np.mean(metrics["obstacle_clearance"]["min"]), 2)} ({np.std(metrics["obstacle_clearance"]["min"]):.2f})')
        
        
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

    def clean_method_name(method_name):
        if method_name == "lmpcc":
            return "LMPCC"
        elif method_name == "autoware":
            return "Autoware"
        elif method_name == "tmpc":
            return "T-MPC\\texttt{++}"
        elif method_name == "tmpcnf":
            return "T-MPC\\texttt{++} (w/o fallback)"


    settings = load_default_settings([scenario])
    settings["caption"] = caption
    settings["add_table_data"] = add_table_data
    settings["clean_method_name"] = clean_method_name

    create_table(base_folder, scenario, experiments, settings, config)

if __name__ == '__main__':
    choice = sys.argv[1]

    scenario = sys.argv[2]
    config = Config()
    base_folder="/workspaces/autoware_iv/src/universe/external/planning/jackal_socialsim/"
    experiments = get_all_experiments_for_scenario(base_folder, scenario, config)
    print(f"Experimental results found for methods: {experiments}")

    if choice.lower() == "trajectories":
        one_figure(experiments, base_folder, scenario, config)

    if choice.lower() == "single_trajectory":
        config = Config(remove_first=True)
        figure_per_experiment(experiments, base_folder, scenario, config)

    if choice.lower() == "table":
         table(experiments, base_folder, scenario, config)

    # if choice.lower() == "combined_table":
        #  table(["empty.xml", "random_2.xml", "random_4.xml"], base_folder, scenario)

    if choice.lower() == "control_signals":
         control_signals(experiments, base_folder, scenario)

    if choice.lower() == "all":
        one_figure(experiments, base_folder, scenario, config)
        figure_per_experiment(experiments, base_folder, scenario, config)
        table(experiments, base_folder, scenario, config)
        



        
