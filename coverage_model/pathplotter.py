import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib.widgets import Button

def plot_interactive_paths(G, uav_paths, uav_covered_nodes, sink, Rs, Rc,
                           Nx, Ny, O_lin=None, aux_tensor=None):
    """
    Interactive plot for UAV paths, coverage, and optional battery levels.
    """
    if O_lin is None: O_lin = []

    def get_T():
        return max((len(p) for p in uav_paths.values() if p), default=0)

    T = get_T()
    if T == 0:
        print("Nothing to plot: all UAV paths are empty.")
        return

    battery_levels = {}
    if aux_tensor is not None and aux_tensor.ndim == 3:
        for n in range(len(uav_paths)):
            battery_levels[n] = [aux_tensor[t, n, 0] for t in range(T)]
    else:
        battery_levels = {n: [] for n in range(len(uav_paths))}

    uav_colors = ['red', 'blue', 'green', 'orange', 'purple', 'brown', 'pink', 'cyan']
    fig = plt.figure(figsize=(16, 8))
    gs = fig.add_gridspec(2, 2, width_ratios=[1, 1], height_ratios=[1, 0.4])
    ax_grid = fig.add_subplot(gs[0, :])
    ax_batt = fig.add_subplot(gs[1, :])
    
    btn_ax_prev = fig.add_axes([0.35, 0.02, 0.1, 0.05]); button_prev = Button(btn_ax_prev, '⟵ Prev')
    btn_ax_next = fig.add_axes([0.55, 0.02, 0.1, 0.05]); button_next = Button(btn_ax_next, 'Next ⟶')

    state = {'t': 0}
    sink_y, sink_x = np.unravel_index(sink, (Ny, Nx))

    def render_at_t(t_idx):
        ax_grid.clear()
        ax_grid.set_title(f'UAV Positions and Coverage at Time: {t_idx}', fontsize=14, fontweight='bold')
        
        # --- Grid and Obstacles ---
        ax_grid.set_xticks(np.arange(-0.5, Nx, 1), minor=True)
        ax_grid.set_yticks(np.arange(-0.5, Ny, 1), minor=True)
        ax_grid.grid(which='minor', color='black', linestyle='-', linewidth=1)
        ax_grid.set_xticks(np.arange(0, Nx, 1)); ax_grid.set_yticks(np.arange(0, Ny, 1))
        ax_grid.set_xlim(-0.5, Nx - 0.5); ax_grid.set_ylim(-0.5, Ny - 0.5)
        ax_grid.set_aspect('equal', adjustable='box')
        ax_grid.invert_yaxis()

        for obs_lin in O_lin:
            obs_y, obs_x = np.unravel_index(obs_lin, (Ny, Nx))
            ax_grid.add_patch(patches.Rectangle((obs_x-0.5, obs_y-0.5), 1, 1, facecolor='gray', alpha=0.8, hatch='/'))
        ax_grid.plot(sink_x, sink_y, 'h', markersize=15, color='gold', markeredgecolor='black', label='Sink')

        # --- UAV Paths and Coverage ---
        for n, path in uav_paths.items():
            color = uav_colors[n % len(uav_colors)]
            # Draw full path faintly
            path_coords = [np.unravel_index(p, (Ny, Nx)) for p in path if p is not None]
            if path_coords:
                py, px = zip(*path_coords)
                ax_grid.plot(px, py, '-', color=color, alpha=0.3, linewidth=2)
            
            # Draw current position and coverage
            if t_idx < len(path) and path[t_idx] is not None:
                pos_y, pos_x = np.unravel_index(path[t_idx], (Ny, Nx))
                
                # Plot coverage area
                for covered_node_lin in uav_covered_nodes[n][t_idx]:
                    cy, cx = np.unravel_index(covered_node_lin, (Ny, Nx))
                    ax_grid.add_patch(patches.Rectangle((cx-0.5, cy-0.5), 1, 1, facecolor=color, alpha=0.15))

                ax_grid.plot(pos_x, pos_y, 'o', markersize=12, color=color, markeredgecolor='black', label=f'UAV {n+1}')
        
        ax_grid.legend(loc='upper right')

        # --- Battery Plot ---
        ax_batt.clear()
        ax_batt.set_title("Battery Levels over Time", fontweight='bold')
        ax_batt.set_xlabel("Time step"); ax_batt.set_ylabel("Battery level")
        ax_batt.set_xlim(-0.5, T - 0.5)
        if battery_levels and any(battery_levels.values()):
            ymin = min(min(v) for v in battery_levels.values() if v)
            ymax = max(max(v) for v in battery_levels.values() if v)
            ax_batt.set_ylim(ymin * 0.95, ymax * 1.05)

        for n, levels in battery_levels.items():
            if len(levels) == T:
                color = uav_colors[n % len(uav_colors)]
                ax_batt.plot(range(T), levels, '-o', color=color, label=f'UAV {n+1}')
                ax_batt.plot(t_idx, levels[t_idx], 's', color='yellow', markersize=10, markeredgecolor='black')
        ax_batt.legend(loc='upper right')
        ax_batt.grid(True, linestyle='--', alpha=0.6)

        fig.canvas.draw_idle()

    def on_next(event=None):
        if state['t'] < T - 1: state['t'] += 1; render_at_t(state['t'])
    def on_prev(event=None):
        if state['t'] > 0: state['t'] -= 1; render_at_t(state['t'])
    def on_key(event):
        if event.key in ('right', 'd', ' '): on_next()
        elif event.key in ('left', 'a', 'p'): on_prev()

    button_next.on_clicked(on_next); button_prev.on_clicked(on_prev)
    fig.canvas.mpl_connect('key_press_event', on_key)
    render_at_t(0)
    plt.tight_layout(rect=[0, 0.1, 1, 1])
    plt.savefig('plot.png')