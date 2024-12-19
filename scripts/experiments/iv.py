import sys, os
import numpy as np
import matplotlib.pyplot as plt

# sys.path.append("./../")
sys.path.append(os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..')))
from load_data import get_all_experiments_for_scenario
from table import create_table, load_default_settings
from plot import plot_agent_trajectories, plot_agent_trajectories_for_all_experiments
from plot import compare_control_signals, plot_duration
from config import Config

def custom_min(values):
    return min([v for v in values if v != "-"])

def value_or_dash(values, value_func):
    if "-" in values:
        return "-"
    else:
        return value_func(values)

def learned_initial_guess_table(experiments, base_folder, scenario):

    def caption(num_experiments, num_scenarios):
             
        return f"\\caption{{Ablation study for using learned initial guesses for global guidance trajectories. Results are presented as ``mean (std)\'\' unless denoted otherwise and show the residuals for equality (Eq.), inequality (Ineq.) and stationarity (Stat.), the percentage of infeasible guided optimization problems and the inference time of the learned guess. Bold values denote best performance.}}\\resizebox{{\\columnwidth}}{{!}}{{%"

    def add_table_data(table):
        table.add_data("Method", lambda metrics, highlight: table.table_settings["clean_method_name"](metrics["experiment"]), align="l")

        table.add_data("Dur. [s]",
                        lambda metrics, highlight: f'{highlight(np.mean(metrics["metric_duration"]), 1)} ({np.std(metrics["metric_duration"]):.1f})')
                        # lambda metrics, highlight: f'{highlight(np.percentile(metrics["metric_duration"], 50), 1)} ({np.percentile(metrics["metric_duration"], 90):.1f})')
        # table.add_data("Collisions",
                        # lambda metrics, highlight: f'{highlight(np.sum([1 if m > 0 else 0 for m in metrics["metric_collisions"]]), 0)}')
        # table.add_data("Avg. Velocity [m/s]",
                        # lambda metrics, highlight: f'{highlight(np.mean(metrics["velocity"]["mean"]), 2)}', highlight_select=max)

        table.add_data("Res. Eq.", 
                       lambda metrics, highlight: f'{highlight(np.mean(metrics["res_eq"]), 3)} ({np.std(metrics["res_eq"]):.3f})', highlight_select=min)
        table.add_data("Res. Ineq.", 
                       lambda metrics, highlight: f'{highlight(np.mean(metrics["res_ineq"]), 3)} ({np.std(metrics["res_ineq"]):.3f})', highlight_select=min)
        table.add_data("Res. Stat.",
                        lambda metrics, highlight: f'{highlight(np.mean(metrics["res_stat"]), 3)} ({np.std(metrics["res_stat"]):.3f})', highlight_select=min)
        # table.add_data("Obj.",
                        # lambda metrics, highlight: f'{highlight(np.mean(metrics["cost"]), 3)} ({np.std(metrics["cost"]):.3f})', highlight_select=min)

        table.add_data("Inf. [\\%]", 
                       lambda metrics, highlight: f'{highlight((100-metrics["feasible_perc"]), 0)}', highlight_select=min)
        # table.add_data("Infeasible [\\%]", 
                    #    lambda metrics, highlight: f'{highlight((100-metrics["planner_feasible_percentage"]), 2)}', highlight_select=min)
        # table.add_data("Stop early", 
                    #    lambda metrics, highlight: f'{highlight((metrics["stop_early_perc"]), 0)}', highlight_select=min)
        # table.add_data("Iterations [#]", 
                    #    lambda metrics, highlight: f'{highlight(np.mean(metrics["num_iterations"]), 0)} ({np.std(metrics["num_iterations"]):.0f})', highlight_select=min)
        
        # table.add_data("Runtime (Max) [ms]",
                    #    lambda metrics, highlight: f'{highlight(np.mean(metrics["runtime"]) * 1000., 0)} ({np.max(metrics["runtime"])* 1000.:.0f})')
        
        std_lambda = lambda x: f'{np.std(x)* 1000.:.1f}'
        # table.add_data("Inference Time (Max) [ms]",
                    #    lambda metrics, highlight: f'{highlight(value_or_dash(metrics["inference_time"], lambda x: np.mean(x) * 1000.), 1)} ({value_or_dash(metrics["inference_time"], std_lambda)})',
                    #    highlight_select=custom_min)

    def clean_method_name(method_name):
        if method_name == "euler8":
            return "Learned Guess"
        elif method_name == "regular":
            return "Baseline"
        return method_name

    include_methods = ["euler8", "regular"]
    experiments = [e for e in experiments if e in include_methods]

    settings = load_default_settings([scenario])
    settings["caption"] = caption
    settings["add_table_data"] = add_table_data
    settings["clean_method_name"] = clean_method_name
    settings["output_name"] = "initial_guess_ablation"
    settings["method_order_list"] = ["regular", "rk4", "euler"]
    print("Creating table")
    comparison_metrics = create_table(base_folder, scenario, experiments, settings, config)
    # from statistic_tests import test_significance
    # test_significance(comparison_metrics, "euler", "metric_duration", verbose=True)
    # test_significance(comparison_metrics, "euler", "res_stat", verbose=True)
    # test_significance(comparison_metrics, "euler", "res_eq", verbose=True)

def benchmark_table(experiments, base_folder, scenario):

    def caption(num_experiments, num_scenarios):             
        return f"\\caption{{Ablation study for using learned initial guesses for global guidance trajectories. Solver convergence statistics, feasibility and inference times are presented.}}\\resizebox{{\\textwidth}}{{!}}{{%"

    def add_table_data(table):
        table.add_data("Method", lambda metrics, highlight: table.table_settings["clean_method_name"](metrics["experiment"]), align="l")

        table.add_data("Dur. [s]",
                        lambda metrics, highlight: f'{highlight(np.mean(metrics["metric_duration"]), 1)} ({np.std(metrics["metric_duration"]):.1f})')
        table.add_data("Collisions",
                        lambda metrics, highlight: f'{highlight(np.sum([1 if m > 0 else 0 for m in metrics["metric_collisions"]]), 0)}')
        table.add_data("Avg. Velocity [m/s]",
                        lambda metrics, highlight: f'{highlight(np.mean(metrics["velocity"]["mean"]), 2)}', highlight_select=max)
        
        # table.add_data("Runtime (Max) [ms]",
                    #    lambda metrics, highlight: f'{highlight(np.mean(metrics["runtime"]) * 1000., 0)} ({np.std(metrics["runtime"])* 1000.:.0f})')

    def clean_method_name(method_name):
        return method_name


    settings = load_default_settings([scenario])
    settings["caption"] = caption
    settings["add_table_data"] = add_table_data
    settings["clean_method_name"] = clean_method_name
    settings["output_name"] = "benchmark"
    # settings["method_order_list"] = ["regular", "rk4", "euler"]
    print("Creating table")
    comparison_metrics = create_table(base_folder, scenario, experiments, settings, config)

def plot_experiment_duration(base_folder, scenario, experiment, config):
    plot_duration(base_folder, scenario, experiment, config)

if __name__ == '__main__':
    choice = sys.argv[1]
    scenario = sys.argv[2]


    base_folder="/workspaces/autoware_iv/experiments"

    def get_experiments(base_folder_in, scenario, config):
        print(f"Looking for files in {base_folder_in}/data/{scenario}")
        experiments = get_all_experiments_for_scenario(base_folder_in, scenario, config)
        print(f"Found results for methods: {experiments}")
        return experiments

    if choice.lower() == "table"  or choice.lower() == "all":
        config = Config(load_json=False)
        experiments = get_experiments(base_folder, scenario, config)
        learned_initial_guess_table(experiments, base_folder, scenario)

    if choice.lower() == "benchmark":
        config = Config()
        base_folder = "/workspaces/autoware_iv/src/universe/external/planning/jackal_socialsim"
        experiments = get_experiments(base_folder, scenario, config)
        benchmark_table(experiments, base_folder, scenario)
        
    if choice.lower() == "duration":
        config = Config(load_json=False)
        experiments = get_experiments(base_folder, scenario, config)
        plot_experiment_duration(base_folder, scenario, experiments, config)


        
