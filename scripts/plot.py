import os, sys

import numpy as np
import matplotlib.pyplot as plt

from metrics import compute_metrics

fig_width = 7.02625 # /textwidth in latex in inches

def save_figure_as(fig, figure_folder, figure_name, ratio=1.0, verbose=True, save_pdf= True, save_png=True, dpi=None, **kwargs):
    fig.set_size_inches(fig_width, fig_width*ratio)

    os.makedirs(os.path.dirname(f'{figure_folder}{figure_name}'), exist_ok=True)

    if save_pdf:
        fig.savefig(f'{figure_folder}{figure_name}.pdf', bbox_inches='tight', **kwargs)

    if save_png:
        fig.savefig(f'{figure_folder}{figure_name}.png', bbox_inches='tight', dpi=dpi, **kwargs)

    if verbose:
        print(f'Saved figure as: {figure_folder}{figure_name}.pdf')

def new_plot(ax=None):
    if ax is None:
        fig = plt.figure(tight_layout=True)
        return fig, plt.gca()
    else:
        return plt.gcf(), ax

def plot_trajectory(trajectory, ax=None, t_start=0, t_final=-1, show_trace=True, show_markers=True, **kwargs):
    
    fig, ax = new_plot(ax)

    trajectory = trajectory[t_start:t_final]
    trajectory = np.reshape(trajectory, (-1, 2))

    if show_markers:
        temp_kwargs = kwargs

        if 'label' not in temp_kwargs.keys():
            temp_kwargs['label'] = '_nolegend_'

        ax.scatter(trajectory[:, 0], trajectory[:, 1], alpha=0.3, s=50, marker='.', **temp_kwargs)

    if show_trace:
        # Plot continuous sequences only
        sequences = []
        t_start = 0
        for t in range(len(trajectory[:, 0]) - 1):
            if np.linalg.norm(trajectory[t+1, :] - trajectory[t, :]) > 1.:
                sequences.append(trajectory[t_start:t, :])
                t_start = t+1
        sequences.append(trajectory[t_start:, :])
        for sequence in sequences:
            ax.plot(sequence[:, 0], sequence[:, 1], alpha=0.7, linestyle='-', linewidth=3.0, **kwargs)

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")

    return fig, ax


if __name__ == "__main__":
    # Example usage
    base_folder = sys.argv[1]
    scenario = sys.argv[2]
    experiment = sys.argv[3]

    metrics, experiment_data = compute_metrics(base_folder, scenario, experiment, verbose=False)

    for e_idx, e in enumerate(experiment_data):
        fig, ax = plot_trajectory(e["pos"], t_final=-80)
        for i in range(metrics["num_obstacles"][0]):
            fig, ax = plot_trajectory(e[f"obstacle_{i}_pos"], t_final=-80, ax=ax, color="C2", show_markers=False)
        plt.ylim([-4, 4])
        save_figure_as(fig, base_folder + f"figures/{scenario}/{experiment}/", f"trajectories_{e_idx}", save_pdf=False)
    


    