import json
import os, sys
import glob

from config import Config

def is_continuous_data(data_name):
    return "metric" not in data_name and "scenario" not in data_name and "experiment" not in data_name

# Default format: .json
def load_data(base_folder, scenario, experiment):
    # Construct the file path
    file_path = f"{base_folder}/data/{scenario}/{experiment}.json"
    
    # Read the JSON file into a dictionary
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"The file {file_path} does not exist.")
    
    print(f"Loading {file_path}")
    with open(file_path, 'r') as file:
        data = json.load(file)
    
    return data

# Legacy format: .txt
def load_txt_data(base_folder, scenario, experiment):
    # Construct the file path
    file_path = f"{base_folder}/data/{scenario}/{experiment}.txt"
    
    # Read the JSON file into a dictionary
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"The file {file_path} does not exist.")
    
    data = parse_txt_to_dict(file_path)
    return data
    


def split_data(data, remove_first=True):
    # Extract the reset values from the dictionary
    reset_values = data["reset"]
    
    # Initialize a list to store the split datasets
    datasets = []
    
    # Initialize the starting index
    start_index = 1
    
    # Split the data based on the reset values
    # (was reset_values[1:])
    for reset in reset_values:
        end_index = int(reset)
        # print(f"Cutting data from {start_index} to {end_index}")
        subset = {key: value[start_index:end_index] for key, value in data.items() if key != "reset" and is_continuous_data(key)}
        datasets.append(subset)
        start_index = end_index

    print(f"Splitting data into {len(datasets)} experiments")

    # Insert metrics for each simulation (one entry per simulation)
    for key, value in data.items():
        if "metric" in key:
            if len(value) > len(datasets):
                print(f"Metric {key} contains more values than there were experiments.")
                continue
            for v in range(1, len(value)): # Skip the first
                datasets[v-1][key] = value[v]

    if remove_first:
        datasets.pop(0)
    
    return datasets

def load_experiment_data(base_folder, scenario, experiment, config):

    if config.verbose:
        print(f"Loading data for {scenario} Experiment: {experiment}")

    # Load data from the specified JSON file
    if config.load_json:
        data = load_data(base_folder, scenario, experiment)
    else:
        data = load_txt_data(base_folder, scenario, experiment)

    # Split the data into datasets
    experiments = split_data(data, remove_first=config.remove_first)
    print(f"Split data into {len(experiments)} experiments")
    # Output the results (just for demonstration purposes)
    if config.verbose:
        for i, experiment in enumerate(experiments):
            print(f"Experiment {i+1}:")
            for key, value in experiment.items():
                if type(value) == list:
                    print(f"  {key}: {len(value)} entries")
                else:
                    print(f"  {key}: {value}")
    
    assert len(experiment) > 0, "Data is empty"

    return experiments

def remove_experiments_with_timeout(data, timeout_data):
    data_out = []
    for i in range(len(data)):
        if not timeout_data[i]:
            data_out.append(data[i])
        else:
            print(f"Timeout on experiment {i}")
            # print(data[i]["feasible_1"])
    return data_out


def get_all_experiments_for_scenario(base_folder, scenario, config, filter=[]):
    folder_path = f"{base_folder}/data/{scenario}/"

    # Use glob to find all .json files in the folder
    if config.load_json:
        files = glob.glob(os.path.join(folder_path, '*.json'))
    else:
        files = glob.glob(os.path.join(folder_path, '*.txt'))

    # Create a list of file names without extensions
    return [os.path.splitext(os.path.basename(file))[0] for file in files if file not in filter]


def parse_txt_to_dict(file_path):
    data_dict = {}

    with open(file_path, 'r') as file:
        lines = file.readlines()

    i = 0
    while i < len(lines):
        line = lines[i].strip()

        if ":" in line:
            # Extract the name, values per entry, and number of entries
            name, metadata = line.split(":")
            name = name.strip()
            metadata_parts = metadata.strip().split()

            if len(metadata_parts) == 2:
                try:
                    values_per_entry = int(metadata_parts[0])
                    num_entries = int(metadata_parts[1])
                except ValueError:
                    raise ValueError(f"Invalid metadata format: {metadata.strip()}")

                # Read the entries
                entries = []
                for _ in range(num_entries):
                    i += 1
                    if i < len(lines):
                        entry = lines[i].strip().split()[:values_per_entry]
                        entry = [float(e) for e in entry]
                        if len(entry) == 1:
                            entry = entry[0]
                        entries.append(entry)

                # Store in the dictionary
                data_dict[name] = entries
        
        i += 1

    return data_dict

if __name__ == "__main__":
    # Example usage
    base_folder = sys.argv[1]
    scenario = sys.argv[2]
    experiment = sys.argv[3]
    
    # Call the main function with the given inputs
    datasets = load_experiment_data(base_folder, scenario, experiment, verbose=True, load_txt=True)
