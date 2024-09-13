
import numpy as np
import math

def rotation_matrix(psi):
    return np.array([[np.cos(psi), -np.sin(psi)],
                     [np.sin(psi),  np.cos(psi)]])

def translate_positions_to_zero(experiment_data, num_obstacles, rotate=0):

    for e_idx, e in enumerate(experiment_data):
        for cur_t in range(len(e["pos"])):
            if np.linalg.norm(np.array(e["pos"][cur_t+1]) - np.array(e["pos"][cur_t])) > 1e-1:
                t = cur_t
                break
    
        zero = np.array(e["pos"][0])
        psi = math.atan2(e["pos"][t+1][1] - e["pos"][t][1], e["pos"][t+1][0] - e["pos"][t][0])
        R = rotation_matrix(-psi + rotate)

        experiment_data[e_idx]["pos"] = [np.dot(R, e- zero) for e in experiment_data[e_idx]["pos"]]
        
        for i in range(num_obstacles):
            experiment_data[e_idx][f"obstacle_{i}_pos"] = [np.dot(R, e- zero) for e in experiment_data[e_idx][f"obstacle_{i}_pos"]]
        
        # plt.ylim([-4, 4])