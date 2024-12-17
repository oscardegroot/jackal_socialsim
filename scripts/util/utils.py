
import numpy as np
import math
from copy import deepcopy

def rotation_matrix(psi):
    return np.array([[np.cos(psi), -np.sin(psi)],
                     [np.sin(psi),  np.cos(psi)]])

def translate_positions_to_zero_the_same_way(experiment_data, num_obstacles, rotate=0, t_start=0, t_final=-1):

    e = experiment_data[0]
    pos_data = e["pos"][t_start:t_final]
    for cur_t in range(len(pos_data)):
        if np.linalg.norm(np.array(pos_data[cur_t+1]) - np.array(pos_data[cur_t])) > 1e-1:
            t = cur_t
            break

    zero = np.array(pos_data[0])
    psi = math.atan2(pos_data[t+1][1] - pos_data[t][1], pos_data[t+1][0] - pos_data[t][0])
    R = rotation_matrix(-psi + rotate)

    for e_idx, e in enumerate(experiment_data):    
        experiment_data[e_idx]["pos"] = [np.dot(R, e- zero) for e in experiment_data[e_idx]["pos"]]
        
        for i in range(num_obstacles):
            experiment_data[e_idx][f"obstacle_{i}_pos"] = [np.dot(R, e- zero) for e in experiment_data[e_idx][f"obstacle_{i}_pos"]]

def translate_positions_to_zero(experiment_data, num_obstacles, rotate=0, t_start=0, t_final=-1, all_experiments_the_same=False):

    if all_experiments_the_same:
        translate_positions_to_zero_the_same_way(experiment_data, num_obstacles, rotate, t_start, t_final)
        return

    for e_idx, e in enumerate(experiment_data):
        if type(t_start) == list:
            t_start_cur = t_start[e_idx]
        else:
            t_start_cur = t_start
        pos_data = e["pos"][t_start_cur:t_final]
        for cur_t in range(len(pos_data)):
            if np.linalg.norm(np.array(pos_data[cur_t+1]) - np.array(pos_data[cur_t])) > 1e-1:
                t = cur_t
                break
    
        zero = np.array(pos_data[0])
        psi = math.atan2(pos_data[t+1][1] - pos_data[t][1], pos_data[t+1][0] - pos_data[t][0])
        R = rotation_matrix(-psi + rotate)

        experiment_data[e_idx]["pos"] = [np.dot(R, e- zero) for e in experiment_data[e_idx]["pos"]]
        
        for i in range(num_obstacles):
            experiment_data[e_idx][f"obstacle_{i}_pos"] = [np.dot(R, e- zero) for e in experiment_data[e_idx][f"obstacle_{i}_pos"]]
        
        # plt.ylim([-4, 4])

def detect_start_times(experiment_data, num_obstacles):

    start_times = []
    for e_idx, e in enumerate(experiment_data):
        if num_obstacles == 0:
            start_times.append(0)
            continue

        # For all obstacles -> Find where their position jumps
        max_start_time = 0
        for i in range(num_obstacles):
            obs_pos = deepcopy(e[f"obstacle_{i}_pos"])
            prev_pos = obs_pos[0]
            t_test = 0
            while prev_pos[0] == -1000000000.0: # Filter for cases where the obstacle is missing!
                t_test += 1
                prev_pos = obs_pos[t_test]
            # print(f"valid at t = {t_test}, value = {prev_pos}")
            for cur_t in range(t_test+1, round(len(obs_pos) / 2)): # The switch should happen at the start somewhere
                # Filter for cases where the obstacle is missing!
                if(prev_pos[0] == -1000000000.0 or obs_pos[cur_t][0] == -1000000000.0):
                    continue

                if np.linalg.norm(np.array(prev_pos) - np.array(obs_pos[cur_t])) > 2.:
                    if cur_t > max_start_time:
                        max_start_time = cur_t # We keep only the latest time
                    break
                prev_pos = deepcopy(obs_pos[cur_t])
        start_times.append(max_start_time)
    return start_times
        
