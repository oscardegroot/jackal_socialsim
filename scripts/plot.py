import os, sys

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import seaborn as sns

from metrics import compute_metrics
from load_data import load_experiment_data
from util.utils import translate_positions_to_zero, detect_start_times

fig_width = 7.02625 # /textwidth in latex in inches

vehicle_color = [255/ 256., 111./256., 97./256.]
ped_color = [13/256., 102./256., 100./256.]

def set_font(latex=True, fontsize=20):
    plt.rcdefaults()

    if latex:
        # plt.rcParams['text.usetex'] = True
        matplotlib.rcParams['mathtext.fontset'] = 'stix'
        matplotlib.rcParams['font.family'] = 'STIXGeneral'
        # matplotlib.pyplot.title(r'ABC123 vs $\mathrm{ABC123}^{123}$')
        font = {
            # 'family' : 'serif',
            # 'serif' : ["Computer Modern Roman"],
            'weight' : 'bold',
            'size'   : fontsize}
    else:
        font = {
            # 'family' : 'serif',
            # 'serif' : ["Computer Modern Serif"],
            'weight' : 'bold',
            'size'   : fontsize}
    plt.rc('font', **font)


def save_figure_as(fig, figure_folder, figure_name, ratio=1.0, verbose=True, save_pdf= True, save_png=True, dpi=None, **kwargs):
    fig.set_size_inches(fig_width, fig_width*ratio)

    os.makedirs(os.path.dirname(f'{figure_folder}{figure_name}'), exist_ok=True)

    if save_pdf:
        fig.savefig(f'{figure_folder}{figure_name}.pdf', bbox_inches='tight', **kwargs)

    if save_png:
        fig.savefig(f'{figure_folder}{figure_name}.png', bbox_inches='tight', dpi=dpi, **kwargs)

    if verbose:
        print(f'Saved figure as: {figure_folder}{figure_name}.png')

def new_plot(ax=None):

    if ax is None:
        fig = plt.figure(tight_layout=True)
        return fig, plt.gca()
    else:
        return plt.gcf(), ax

def plot_environment(ax):

    lw = 3.
    ax.plot([-3, 10], [3, 3], 'k-', linewidth=lw)
    ax.plot([-3, 10], [-3, -3], 'k-', linewidth=lw)

    ax.plot([16, 23], [3, 3], 'k-', linewidth=lw)
    ax.plot([16, 23], [-3, -3], 'k-', linewidth=lw)

    ax.plot([16, 16], [3, 23], 'k-', linewidth=lw)
    ax.plot([10, 10], [3, 23], 'k-', linewidth=lw)    
    
    ax.plot([16, 16], [-3, -23], 'k-', linewidth=lw)
    ax.plot([10, 10], [-3, -23], 'k-', linewidth=lw)

    ax.plot([0., 23], [0., 0.], 'k-.', linewidth=lw)

    sns.despine(offset=15)

def compare_control_signals(base_folder, scenario, experiments, styles='-', colors=None, remove_first=True, verbose=False):

    if type(styles) == str:
        s = styles
        styles = dict()
        for experiment in experiments:
            styles[experiment] = s

    if colors is None:
        colors = dict()
        for i in range(len(experiments)):
            colors[experiments[i]] = f'C{i}'

    data = dict()
    for experiment in experiments:
        data[experiment] = load_experiment_data(base_folder, scenario, experiment, 
                                           verbose=verbose, remove_first=remove_first)
        num_experiments = len(data[experiment])

    for idx in range(num_experiments):
        fig = plt.figure()
        ax = plt.gca()
        t_max = 0
        for e_idx, experiment in enumerate(experiments):
            t_max = max(t_max, plot_control_signal(ax, data[experiment][idx]["v"], experiment, 
                                                   colors[experiment], styles[experiment]))
            
        ax.plot([0, t_max], [2.3, 2.3], 'k--', linewidth=2.)
        plt.xlabel("Time [s]")
        plt.ylabel("Velocity [m/s]")
        plt.legend()
        plt.tight_layout()
        ax.set_xlim(left=0)
        ax.set_ylim(bottom=0)

        save_figure_as(fig, base_folder + f"figures/{scenario}/control/", f"v_{idx}", save_pdf=False, ratio=0.25)


def plot_control_signal(ax, signal, label, color, style="-"):

    def get_start_time(data):
        # First remove the start
        for t in range(len(data)):
            if data[t] <= 1e-4:
                start_time = t
                break

        max_t = 120
        for t in range(start_time + 1, max_t):
            
            if data[t-1] > 0.1 and data[t] <= 1e-3:
                start_time = t
                break

        for t in range(start_time, len(data)):
            if data[t] > 1e-2:
                return t

    start_time = get_start_time(signal)
    
    t_range =  [float(x)*0.05 for x in range(len(signal[start_time:]))]
    ax.plot(t_range, signal[start_time:], style, color=color, label=label)
    return t_range[-1]


def plot_trajectory(trajectory, ax=None, t_start=0, t_final=-1, show_trace=True, show_markers=True, marker='.', rotate=False, marker_size=150, alpha=1,**kwargs):
    fig, ax = new_plot(ax)

    trajectory = trajectory[t_start:t_final]
    trajectory = np.reshape(trajectory, (-1, 2))

    if show_markers:
        temp_kwargs = kwargs

        if 'label' not in temp_kwargs.keys():
            temp_kwargs['label'] = '_nolegend_'
        
        # Transparancy
        color_cycle = plt.cm.get_cmap('tab10', 10)  # 'tab10' is the default color cycle with 10 colors

        if 'color' in temp_kwargs.keys():
            if "C" in temp_kwargs['color']:
                color = int(temp_kwargs['color'].split("C")[1])
                base_color = color_cycle(color)  # Get 'C0' in RGBA format
            else:
                base_color = temp_kwargs['color']
                if len(base_color) == 3:
                    base_color.append(1.)

            c_save = temp_kwargs['color']
            del temp_kwargs['color']
        else:
            base_color = color_cycle(0)  # Get 'C0' in RGBA format

        N = len(trajectory[:, 0])
        alphas = np.linspace(0.01, alpha, N)
        alphas = [a**2 for a in alphas]
        colors = np.zeros((N, 4))  # Nx4 array for RGBA values
        for i in range(N):
            colors[i, :] = base_color  # Set the base color (same for all points)
            colors[i, 3] = alphas[-i]   # Vary the alpha (transparency)

        ax.scatter(trajectory[:, 0], trajectory[:, 1], color=colors, s=marker_size, marker=marker, **temp_kwargs)
        temp_kwargs['color'] = c_save

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
            ax.plot(sequence[:, 0], sequence[:, 1], alpha=0.7, linestyle='-', linewidth=2.0, **kwargs)

    # Plot the start position
    ax.plot(trajectory[0, 0], trajectory[0, 1], 'kx', markersize=10, mew=2.0)

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_aspect('equal', 'box')

    return fig, ax

def plot_agent_trajectories_for_all_experiments(base_folder, scenario, experiment, color_idx=0,
                                                 external_ax=None, xlim=None, ylim=None, t_final=-18, translate_to_zero=False, **kwargs):
    set_font(latex=True, fontsize=22)
    name = experiment.split(" ")[-1].split(".")[0].lower()
    metrics, experiment_data = compute_metrics(base_folder, scenario, experiment, verbose=False)

    start_times = detect_start_times(experiment_data, metrics["num_obstacles"][0])
    t_start = start_times[0]
    if translate_to_zero:
        translate_positions_to_zero(experiment_data, metrics["num_obstacles"][0], 
                                    t_start=t_start, t_final=t_final,rotate = np.pi/2., all_experiments_the_same=True)

    # All together
    # t_final = -1
    if external_ax is None:
        fig = plt.figure()
        ax = plt.gca()
    else:
        ax = external_ax
    
    #Deep Teal: rgb(13, 102, 100)
    #Coral Pink: rgb(255, 111, 97)
    # vehicle_color = f"C{color_idx}"
    # ped_color = "C2"


    for e_idx, e in enumerate(experiment_data):
        # fig, ax = plot_trajectory(e["pos"], ax=ax, t_start=t_start, t_final=t_final, color=f"C{color_idx}", **kwargs)
        fig, ax = plot_trajectory(e["pos"], ax=ax, t_start=t_start, t_final=t_final, color=vehicle_color, show_trace=False, **kwargs)
        # for i in range(metrics["num_obstacles"][0]):
            # fig, ax = plot_trajectory(e[f"obstacle_{i}_pos"], t_start=t_start, ax=ax, color=ped_color, show_trace=False, **kwargs)
        # plt.ylim([-4, 4])

    # plot_environment(ax)

    if xlim is not None:
        ax.set_xlim(xlim)
    if ylim is not None:
        ax.set_ylim(ylim)
    plt.locator_params(nbins=2)

    save_figure_as(fig, base_folder + f"figures/{scenario}/{experiment}/", f"trajectories_{name}", save_pdf=True)

def plot_agent_trajectories(base_folder, scenario, experiment, 
                            color_idx=0, external_ax=None, xlim=None, ylim=None, 
                            translate_to_zero=False, t_start=0, t_final=-1, **kwargs):

    metrics, experiment_data = compute_metrics(base_folder, scenario, experiment, verbose=False, **kwargs)

    start_times = detect_start_times(experiment_data, metrics["num_obstacles"][0])

    if translate_to_zero:
        translate_positions_to_zero(experiment_data, metrics["num_obstacles"][0], t_start=start_times, rotate = -np.pi/2.)

    # NOTE Figure per experiment
    t_final = t_final
    # t_start = t_start
    for e_idx, e in enumerate(experiment_data):
        t_start = start_times[e_idx]
        fig, ax = plot_trajectory(e["pos"], t_start=t_start, t_final=t_final, color=vehicle_color, show_trace=False, marker_size=600)
        for i in range(metrics["num_obstacles"][0]):
            fig, ax = plot_trajectory(e[f"obstacle_{i}_pos"], t_start=t_start, t_final=t_final, ax=ax, color=ped_color, show_trace=False, show_markers=True)
        # plt.ylim([-4, 4])
        if xlim is not None:
            ax.set_xlim(xlim)
        if ylim is not None:
            ax.set_ylim(ylim)
        save_figure_as(fig, base_folder + f"figures/{scenario}/{experiment}/", f"trajectories_{e_idx}", save_pdf=False)

if __name__ == "__main__":
    # Example usage
    base_folder = sys.argv[1]
    scenario = sys.argv[2]
    experiment = sys.argv[3]
    plot_agent_trajectories(base_folder, scenario, experiment)


    
    


    