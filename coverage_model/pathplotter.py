import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib.colors import ListedColormap
from matplotlib.widgets import Button

def plot_interactive_paths(G, uav_paths, uav_covered_nodes, sink, Rs, Rc,
                           Nx, Ny, O_lin=None, aux_tensor=None):
    """
    Interactive plot: UAV paths, coverage, and (optional) battery levels.
    """
    if O_lin is None:
        O_lin = []

    # Helper: infer T from paths
    def get_T():
        if not uav_paths:
            return 0
        return max((len(path) for path in uav_paths.values() if path), default=0)

    T = get_T()
    if T == 0:
        print("Nothing to plot: all UAV paths are empty.")
        return

    # ---------- Extract battery levels if aux_tensor provided ----------
    battery_levels = {}
    if aux_tensor is not None:
        for n in range(len(uav_paths)):
            battery_levels[n] = [aux_tensor[t, n, 0] for t in range(T)]

    # ---------- Setup figure ----------
    uav_colors = ['red', 'blue', 'green', 'orange', 'purple', 'teal', 'magenta', 'brown']
    fig = plt.figure(figsize=(18, 10))
    gs = fig.add_gridspec(2, 3, width_ratios=[1, 1, 0.12],
                          height_ratios=[1, 0.5], wspace=0.25, hspace=0.35)

    ax1 = fig.add_subplot(gs[0, 0])  # main grid view
    ax2 = fig.add_subplot(gs[0, 1])  # coverage summary
    ax_batt = fig.add_subplot(gs[1, 0:2])  # NEW: battery levels over time
    ax_legend_pad = fig.add_subplot(gs[:, 2])
    ax_legend_pad.axis('off')

    # Buttons
    btn_ax_prev = fig.add_axes([0.15, 0.02, 0.1, 0.05])
    btn_ax_next = fig.add_axes([0.27, 0.02, 0.1, 0.05])
    button_prev = Button(btn_ax_prev, '⟵ Prev')
    button_next = Button(btn_ax_next, 'Next ⟶')

    state = {'t': 0}

    # ---------- Draw function ----------
    def render_at_t(t_idx):
        # (reuse your grid + coverage render code for ax1, ax2...)

        # --- Battery levels plot ---
        ax_batt.clear()
        ax_batt.set_title("Battery Levels over Time", fontsize=13, fontweight='bold')
        ax_batt.set_xlabel("Time step", fontweight='bold')
        ax_batt.set_ylabel("Battery level", fontweight='bold')
        ax_batt.set_xlim(0, T-1)

        if aux_tensor is not None:
            for n, levels in battery_levels.items():
                color = uav_colors[n % len(uav_colors)]
                ax_batt.plot(range(T), levels, '-o', color=color, label=f'UAV {n+1}')
                # highlight current t
                ax_batt.plot(t_idx, levels[t_idx], 's', color=color, markersize=10,
                             markeredgecolor='black', markeredgewidth=1.3)

            ax_batt.legend(loc='upper right', fontsize=9)

        fig.canvas.draw_idle()

    # ---------- Interaction handlers ----------
    def on_next(event=None):
        if state['t'] < T-1:
            state['t'] += 1
            render_at_t(state['t'])

    def on_prev(event=None):
        if state['t'] > 0:
            state['t'] -= 1
            render_at_t(state['t'])

    def on_key(event):
        if event.key in ('right', 'd', 'n'):
            on_next()
        elif event.key in ('left', 'a', 'p'):
            on_prev()

    button_next.on_clicked(on_next)
    button_prev.on_clicked(on_prev)
    fig.canvas.mpl_connect('key_press_event', on_key)

    # ---------- Initial render ----------
    render_at_t(0)
    plt.show()
