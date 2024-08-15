import sys

from metrics import compute_metrics

def compute_comparison_metrics(base_folder, scenario, experiments, verbose=False):

    comparison_metrics = []
    
    for experiment in experiments:
        new_metrics, _ = compute_metrics(base_folder, scenario, experiment, verbose)
        comparison_metrics.append(new_metrics)
    
    return comparison_metrics

if __name__ == "__main__":
    # Example usage
    base_folder = sys.argv[1]
    scenario = sys.argv[2]
    experiments = [
        "follow_the_path",
        "try"
    ]

    comparison_metrics = compute_comparison_metrics(base_folder, scenario, experiments, verbose=False)
    for metric in comparison_metrics.keys():
        print(comparison_metrics[metric])

