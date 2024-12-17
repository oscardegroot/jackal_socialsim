
import sys
import numpy as np

from load_data import load_experiment_data, remove_experiments_with_timeout
from config import Config

safe_radius = 0.3 + 0.325 # NOTE: planner uses 0.4, for safety

def add_mean_min_max_std(metrics, experiment_data, key, metric_name):
    metrics[metric_name] = dict()
    metrics[metric_name]["mean"] = []
    metrics[metric_name]["min"] = []
    metrics[metric_name]["max"] = []
    metrics[metric_name]["std"] = []

    if len(experiment_data[0][key]) == 0:
        return

    for e in experiment_data:
        metrics[metric_name]["mean"].append(np.mean(e[key]))
        # print(e[key])
        metrics[metric_name]["min"].append(np.min(e[key]))
        metrics[metric_name]["max"].append(np.max(e[key]))
        metrics[metric_name]["std"].append(np.std(e[key]))

def add_metric_to_data(metrics, experiment_data, metric_name):
    for i, e in enumerate(experiment_data):
        experiment_data[i][metric_name] = metrics[metric_name][i]


def load_data_metrics(metrics, experiment_data):
    # Initialize lists
    for key, value in experiment_data[0].items():
        if "metric" in key:
            metrics[key] = []

    for e in experiment_data:
        for key, value in e.items():
            if "metric" in key:
                metrics[key].append(value)

def add_data_into_metrics(metrics, experiment_data, key, rename=None):
    if rename is None:
        rename = key

    metrics[rename] = []
    for e in experiment_data:
        metrics[rename] += e[key]
    

def add_num_iterations_to_data(experiment_data):
    for e in experiment_data:
        e["num_iterations"] = len(e["runtime_control_loop"])

def add_num_experiments(metrics, experiment_data):
    metrics["num_experiments"] = len(experiment_data)

def add_num_obstacles(metrics, experiment_data):
    metrics["num_obstacles"] = []
    for e in experiment_data:
        for i in range(100):
            if not f"obstacle_{i}_pos" in e.keys():
                metrics["num_obstacles"].append(i)
                break
    
    add_metric_to_data(metrics, experiment_data, "num_obstacles")

def add_obstacle_clearance(metrics, experiment_data):
    if "pos" not in experiment_data[0].keys():
        return
    
    assert "num_obstacles" in metrics.keys(), "Number of obstacles should be computed before computing the clearance"

    for e in experiment_data:
        e["obstacle_clearance"] = []
        for t in range(e["num_iterations"]):
            pos = np.array(e["pos"][t])

            min_clearance = 1e9
            for i in range(e["num_obstacles"]):
                # print(e[f"obstacle_{i}_pos"][t])
                if len(e[f"obstacle_{i}_pos"]) <= t:
                    continue
                obs_pos = np.array(e[f"obstacle_{i}_pos"][t])
                clearance = np.linalg.norm(pos - obs_pos) - safe_radius
                min_clearance = min(min_clearance, clearance)
            
            if min_clearance != 1e9:
                e["obstacle_clearance"].append(min_clearance)

    add_mean_min_max_std(metrics, experiment_data, "obstacle_clearance", "obstacle_clearance")

def add_average_velocity(metrics, experiment_data):
    if "v" in experiment_data[0].keys():
        add_mean_min_max_std(metrics, experiment_data, "v", "velocity")

def add_solver_stats(metrics, experiment_data):

    if "res_eq_1" not in experiment_data[0].keys():
        print("Solver stats are not in this data file")

    def get_all_valid(experiment_data, key):
        result = []
        for e in experiment_data:
            result += [v for v in e[key] if v != -1 and v < 10.]
        return result
    
    def get_count(experiment_data, key, count_if=1):
        result = []
        all = []
        for e in experiment_data:
            result += [v for v in e[key] if v == count_if]
            all += e[key]
        return len(result), len(all)

    num_solvers = 0
    for i in range(1,100):
        if f"res_eq_{i}" not in experiment_data[0].keys():
            num_solvers = i
            break

    res_eq = []
    res_ineq = []
    res_stat = []

    feasible_count = feasible_total = 0
    stop_early_count = stop_early_total = 0
    for i in range(1, num_solvers):
        res_eq += get_all_valid(experiment_data, f"res_eq_{i}")
        res_ineq += get_all_valid(experiment_data, f"res_ineq_{i}")
        res_stat += get_all_valid(experiment_data, f"res_stat_{i}")

        f, total = get_count(experiment_data, f"feasible_{i}")
        feasible_count += f
        feasible_total += total

        s, stotal = get_count(experiment_data, f"stop_early_{i}")
        stop_early_count += s
        stop_early_total += stotal
    

    metrics["res_eq"] = res_eq
    metrics["res_ineq"] = res_ineq
    metrics["res_stat"] = res_stat

    metrics["feasible_perc"] = float(feasible_count) / float(feasible_total) * 100.
    metrics["stop_early_perc"] = float(stop_early_count) / float(stop_early_total) * 100.


def add_inference_time(metrics, experiment_data):

    if "runtime_learned_guess" not in experiment_data[0].keys():
        metrics["inference_time"] = "-"
    else:
        add_data_into_metrics(metrics, experiment_data, key="runtime_learned_guess", rename="inference_time")


def compute_metrics(base_folder, scenario, experiment, config):    
    # Call the main function with the given inputs
    experiment_data = load_experiment_data(base_folder, scenario, experiment, config)

    metrics = dict()
    metrics["scenario"] = scenario
    metrics["experiment"] = experiment

    load_data_metrics(metrics, experiment_data)

    timeouts = [False] + [v > 37. for v in metrics["metric_duration"]]
    experiment_data = remove_experiments_with_timeout(experiment_data, timeouts)

    metrics["metric_duration"] = [v for v in metrics["metric_duration"] if v <= 37.]

    add_num_iterations_to_data(experiment_data)

    add_num_experiments(metrics, experiment_data)
    add_num_obstacles(metrics, experiment_data)

    add_average_velocity(metrics, experiment_data)
    add_obstacle_clearance(metrics, experiment_data)

    add_solver_stats(metrics, experiment_data)
    add_inference_time(metrics, experiment_data)

    add_data_into_metrics(metrics, experiment_data, key="runtime_control_loop", rename="runtime")

    if config.verbose:
        for name, value in metrics.items():
            print(f"{name}: {value}")
    
    return metrics, experiment_data

if __name__ == "__main__":
    # Example usage
    base_folder = sys.argv[1]
    scenario = sys.argv[2]
    experiment = sys.argv[3]

    config = Config(load_json=False, verbose=True)

    compute_metrics(base_folder, scenario, experiment, config)
    