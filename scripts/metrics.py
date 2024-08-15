
import sys
import numpy as np

from load_data import load_experiment_data

safe_radius = 0.3 + 0.325 # NOTE: planner uses 0.4, for safety

def add_mean_min_max_std(metrics, experiment_data, key, metric_name):
    metrics[metric_name] = dict()
    metrics[metric_name]["mean"] = []
    metrics[metric_name]["min"] = []
    metrics[metric_name]["max"] = []
    metrics[metric_name]["std"] = []

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

def add_num_iterations_to_data(experiment_data):
    for e in experiment_data:
        e["num_iterations"] = len(e["v"])

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
    add_mean_min_max_std(metrics, experiment_data, "v", "velocity")

# def save_metrics(metrics):

def compute_metrics(base_folder, scenario, experiment, verbose=False):
    # Call the main function with the given inputs
    experiment_data = load_experiment_data(base_folder, scenario, experiment, verbose=verbose)

    metrics = dict()
    metrics["scenario"] = scenario
    metrics["experiment"] = experiment

    load_data_metrics(metrics, experiment_data)

    add_num_iterations_to_data(experiment_data)

    add_num_experiments(metrics, experiment_data)
    add_num_obstacles(metrics, experiment_data)

    add_average_velocity(metrics, experiment_data)
    add_obstacle_clearance(metrics, experiment_data)

    if verbose:
        for name, value in metrics.items():
            print(f"{name}: {value}")
    
    return metrics, experiment_data

if __name__ == "__main__":
    # Example usage
    base_folder = sys.argv[1]
    scenario = sys.argv[2]
    experiment = sys.argv[3]

    compute_metrics(base_folder, scenario, experiment, verbose=True)
    