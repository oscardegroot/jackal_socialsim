
import numpy as np
import math

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
        pos_data = e["pos"][t_start:t_final]
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

        obs_pos = experiment_data[e_idx][f"obstacle_{0}_pos"]
        prev_pos = obs_pos[0]
        for cur_t in range(1, len(obs_pos)):
            if np.linalg.norm(np.array(prev_pos) - np.array(obs_pos[cur_t])) > 5.:
                start_times.append(cur_t + 4)
                break
    return start_times
        
