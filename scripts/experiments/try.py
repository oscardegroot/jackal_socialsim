import sys, os
import numpy as np

sys.path.append("..")
from load_data import get_all_experiments_for_scenario
from table import create_table, load_default_settings


if __name__ == '__main__':
    choice = sys.argv[1]

    if choice.lower() == "table":

        def caption(num_experiments, num_scenarios):
            return f"\\caption{{Comparison table.}}\\resizebox{{\\textwidth}}{{!}}{{%"

        def add_table_data(table):
            table.add_data("Method", lambda metrics, highlight: table.table_settings["clean_method_name"](metrics["experiment"]), align="l")

            table.add_data("Dur. [s]",
                           lambda metrics, highlight: f'{highlight(np.mean(metrics["metric_duration"]), 1)} ({np.std(metrics["metric_duration"]):.1f})')

            table.add_data("Min Dist. [m]",
                           lambda metrics, highlight: f'{highlight(np.mean(metrics["obstacle_clearance"]["min"]), 2)} ({np.std(metrics["obstacle_clearance"]["min"]):.2f})')
            
            
            table.add_data("Collisions",
                           lambda metrics, highlight: f'{highlight(np.sum([1 if m > 0 else 0 for m in metrics["metric_collisions"]]), 0)}')
            # table.add_data("Mean Clearance. [m]",
                        #    lambda metrics, highlight: f'{highlight(np.mean(metrics["obstacle_clearance"]["mean"]), 2)} ({np.std(metrics["obstacle_clearance"]["mean"]):.2f})')

            table.add_data("Avg. Velocity [m/s]",
                           lambda metrics, highlight: f'{highlight(np.mean(metrics["velocity"]["mean"]), 2)}')
            # table.add_data("Runtime (Max) [ms]",
                        #    lambda metrics, highlight: f'{highlight(metrics["runtime"]["mean"] * 1000., 0)} ({metrics["runtime"]["max"] * 1000.:.0f})')

        scenario = sys.argv[2]
        base_folder="/workspace/src/LLM-Navigation/"
        experiments = get_all_experiments_for_scenario(base_folder, scenario, filter=["none"])
        print(experiments)
        # experiments = [
        #     "Follow the path.",
        #     "Follow the path. You are driving in a factory.",
        #     "Follow the path. The robot must navigate through a hospital.",
        #     "Follow the path. Drive slowly.",
        #     "Follow the path. Drive fast.",
        #     "Follow the path and try to keep a distance of 1m from pedestrians."
        # ]#
        
        settings = load_default_settings([scenario])
        settings["caption"] = caption
        settings["add_table_data"] = add_table_data

        create_table(base_folder, scenario, experiments, settings, verbose=False)
