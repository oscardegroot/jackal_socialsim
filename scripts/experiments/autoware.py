import sys, os
import numpy as np
import matplotlib.pyplot as plt

sys.path.append("..")
from load_data import get_all_experiments_for_scenario
from table import create_table, load_default_settings
from plot import plot_agent_trajectories, plot_agent_trajectories_for_all_experiments


if __name__ == '__main__':
    choice = sys.argv[1]

    scenario = sys.argv[2]
    base_folder="/workspaces/autoware_iv/src/universe/external/planning/jackal_socialsim/"

    if choice.lower() == "trajectories":
        experiments = get_all_experiments_for_scenario(base_folder, scenario, filter=["none"])
        for experiment in experiments:
            t_final = -5
            # if "path" in experiment:
                # t_final = -230
            plot_agent_trajectories_for_all_experiments(base_folder, scenario, experiment, t_final=t_final)

#     if choice.lower() == "single_trajectory":
#         experiments = get_all_experiments_for_scenario(base_folder, scenario, filter=["none"])
#         for experiment in experiments:
#             plot_agent_trajectories(base_folder, scenario, experiment, xlim=[-2, 38.], ylim=[-20., 15.], translate_to_zero=True)


#     if choice.lower() == "follow_trajectories":
#         fig = plt.figure()
#         ax = plt.gca()

#         experiments = get_all_experiments_for_scenario(base_folder, "follow_scenario", filter=["none"])
#         for idx, experiment in enumerate(experiments):
#             plot_agent_trajectories(base_folder, scenario, experiment, 
#                                     color_idx=0, external_ax=ax, 
#                                     xlim=[0, 20], ylim=[-5, 12.5],
#                                     marker=['*', 's'][idx])

#     if choice.lower() == "table":

#         def caption(num_experiments, num_scenarios):
#             return f"\\caption{{{scenario}.}}\\resizebox{{\\textwidth}}{{!}}{{%"

#         def add_table_data(table):
#             table.add_data("Method", lambda metrics, highlight: table.table_settings["clean_method_name"](metrics["experiment"]), align="l")

#             table.add_data("Dur. [s]",
#                            lambda metrics, highlight: f'{highlight(np.mean(metrics["metric_duration"]), 1)} ({np.std(metrics["metric_duration"]):.1f})')

#             table.add_data("Min Dist. [m]",
#                            lambda metrics, highlight: f'{highlight(np.mean(metrics["obstacle_clearance"]["min"]), 2)} ({np.std(metrics["obstacle_clearance"]["min"]):.2f})')
            
            
#             table.add_data("Collisions",
#                            lambda metrics, highlight: f'{highlight(np.sum([1 if m > 0 else 0 for m in metrics["metric_collisions"]]), 0)}')
#             # table.add_data("Mean Clearance. [m]",
#                         #    lambda metrics, highlight: f'{highlight(np.mean(metrics["obstacle_clearance"]["mean"]), 2)} ({np.std(metrics["obstacle_clearance"]["mean"]):.2f})')

#             table.add_data("Timeouts",
#                            lambda metrics, highlight: f'{highlight(np.sum([1 if m > 0 else 0 for m in metrics["metric_timeout"]]), 0)}')

#             table.add_data("Avg. Velocity [m/s]",
#                            lambda metrics, highlight: f'{np.mean(metrics["velocity"]["mean"]):.2f}')
#             # table.add_data("Runtime (Max) [ms]",
#                         #    lambda metrics, highlight: f'{highlight(metrics["runtime"]["mean"] * 1000., 0)} ({metrics["runtime"]["max"] * 1000.:.0f})')

#         experiments = get_all_experiments_for_scenario(base_folder, scenario, filter=["none"])
#         print(experiments)

#         settings = load_default_settings([scenario])
#         settings["caption"] = caption
#         settings["add_table_data"] = add_table_data

#         create_table(base_folder, scenario, experiments, settings, verbose=False)
