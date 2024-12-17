import sys, os
import numpy as np
import matplotlib.pyplot as plt

# sys.path.append("./../")
sys.path.append(os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..')))
from load_data import get_all_experiments_for_scenario
from table import create_table, load_default_settings
from plot import plot_agent_trajectories, plot_agent_trajectories_for_all_experiments
from plot import compare_control_signals
from config import Config

def custom_min(values):
    return min([v for v in values if v != "-"])

def value_or_dash(values, value_func):
    if "-" in values:
        return "-"
    else:
        return value_func(values)

    # return value if value != "-" else 10000

def learned_initial_guess_table(experiments, base_folder, scenario):

    def caption(num_experiments, num_scenarios):
    
        # scenario_pedestrians = scenario.split('_')[-1].split('.')[0]
        # if "empty" in scenario:
        #      scenario_pedestrians = 0
             
        return f"\\caption{{Ablation study for using learned initial guesses for global guidance trajectories. Solver convergence statistics, feasibility and inference times are presented.}}\\resizebox{{\\textwidth}}{{!}}{{%"

    def add_table_data(table):
        table.add_data("Method", lambda metrics, highlight: table.table_settings["clean_method_name"](metrics["experiment"]), align="l")

        # table.add_data("Dur. [s]",
                        # lambda metrics, highlight: f'{highlight(np.mean(metrics["metric_duration"]), 1)} ({np.std(metrics["metric_duration"]):.1f})')


        table.add_data("res eq", 
                       lambda metrics, highlight: f'{highlight(np.mean(metrics["res_eq"]), 2)} ({np.std(metrics["res_eq"]):.2f})', highlight_select=min)
        table.add_data("res ineq", 
                       lambda metrics, highlight: f'{highlight(np.mean(metrics["res_ineq"]), 2)} ({np.std(metrics["res_ineq"]):.2f})', highlight_select=min)
        table.add_data("res stat",
                        lambda metrics, highlight: f'{highlight(np.mean(metrics["res_stat"]), 2)} ({np.std(metrics["res_stat"]):.2f})', highlight_select=min)

        table.add_data("feasible", 
                       lambda metrics, highlight: f'{highlight((metrics["feasible_perc"]), 0)}', highlight_select=max)
        table.add_data("Stop early", 
                       lambda metrics, highlight: f'{highlight((metrics["stop_early_perc"]), 0)}', highlight_select=min)
        
        table.add_data("Runtime (Max) [ms]",
                       lambda metrics, highlight: f'{highlight(np.mean(metrics["runtime"]) * 1000., 0)} ({np.std(metrics["runtime"])* 1000.:.0f})')
        
        std_lambda = lambda x: f'{np.std(x)* 1000.:.1f}'
        table.add_data("Inference Time (Max) [ms]",
                       lambda metrics, highlight: f'{highlight(value_or_dash(metrics["inference_time"], lambda x: np.mean(x) * 1000.), 1)} ({value_or_dash(metrics["inference_time"], std_lambda)})',
                       highlight_select=custom_min)

    def clean_method_name(method_name):
        if method_name == "lmpcc":
            return "LMPCC"
        return method_name


    settings = load_default_settings([scenario])
    settings["caption"] = caption
    settings["add_table_data"] = add_table_data
    settings["clean_method_name"] = clean_method_name
    settings["output_name"] = "initial_guess_ablation"
    settings["method_order_list"] = ["regular", "rk4", "euler"]
    print("Creating table")
    comparison_metrics = create_table(base_folder, scenario, experiments, settings, config)




if __name__ == '__main__':
    choice = sys.argv[1]
    scenario = sys.argv[2]

    config = Config(load_json=False)

    base_folder="/workspaces/autoware_iv/experiments"
    print(f"Looking for files in {base_folder}/data/{scenario}")
    experiments = get_all_experiments_for_scenario(base_folder, scenario, config)
    print(experiments)


    if choice.lower() == "table"  or choice.lower() == "all":
        learned_initial_guess_table(experiments, base_folder, scenario)
        



        
