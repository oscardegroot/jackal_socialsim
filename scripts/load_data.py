import json
import os, sys
import glob

def is_continuous_data(data_name):
    return "metric" not in data_name and "scenario" not in data_name and "experiment" not in data_name

def load_data(base_folder, scenario, experiment):
    # Construct the file path
    file_path = f"{base_folder}/data/{scenario}/{experiment}.json"
    
    # Read the JSON file into a dictionary
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"The file {file_path} does not exist.")
    
    with open(file_path, 'r') as file:
        data = json.load(file)
    
    return data

def split_data(data, remove_first=True):
    # Extract the reset values from the dictionary
    reset_values = data["reset"]
    
    # Initialize a list to store the split datasets
    datasets = []
    
    # Initialize the starting index
    start_index = 0
    
    # Split the data based on the reset values
    for reset in reset_values:
        end_index = reset
        subset = {key: value[start_index:end_index] for key, value in data.items() if key != "reset" and is_continuous_data(key)}
        datasets.append(subset)
        start_index = end_index

    # Insert metrics for each simulation (one entry per simulation)
    for key, value in data.items():
        if "metric" in key:
            for v in range(len(value)):
                datasets[v][key] = value[v]

    if remove_first:
        datasets.pop(0)

    # for dataset in datasets
    
    return datasets

def load_experiment_data(base_folder, scenario, experiment, verbose=False):
    if verbose:
        print(f"Loading data for {scenario} Experiment: {experiment}")

    # Load data from the specified JSON file
    data = load_data(base_folder, scenario, experiment)

    # Split the data into datasets
    experiments = split_data(data)
    
    # Output the results (just for demonstration purposes)
    if verbose:
        for i, experiment in enumerate(experiments):
            print(f"Experiment {i+1}:")
            for key, value in experiment.items():
                if type(value) == list:
                    print(f"  {key}: {len(value)} entries")
                else:
                    print(f"  {key}: {value}")
    
    assert len(experiment) > 0, "Data is empty"

    return experiments

def get_all_experiments_for_scenario(base_folder, scenario, filter=[]):
    folder_path = f"{base_folder}/data/{scenario}/"

    # Use glob to find all .json files in the folder
    json_files = glob.glob(os.path.join(folder_path, '*.json'))

    # Create a list of file names without extensions
    return [os.path.splitext(os.path.basename(file))[0] for file in json_files if file not in filter]


if __name__ == "__main__":
    # Example usage
    base_folder = sys.argv[1]
    scenario = sys.argv[2]
    experiment = sys.argv[3]
    
    # Call the main function with the given inputs
    datasets = load_experiment_data(base_folder, scenario, experiment, verbose=True)
