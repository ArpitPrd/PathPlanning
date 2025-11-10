import numpy as np
import matplotlib
# Force a backend that works well.
# You can also try 'Qt5Agg' if you have PyQt5 installed
try:
    matplotlib.use('Qt5Agg') 
except ImportError:
    print("TkAgg backend not found, trying Qt5Agg...")
    try:
        matplotlib.use('Qt5Agg')
    except ImportError:
        print("Qt5Agg backend not found, falling back to Agg (non-interactive).")
        matplotlib.use('Agg')

import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib.widgets import Button

def _draw_base_grid(ax, title, Nx, Ny, O_lin, sink_x, sink_y):
    """Helper function to draw the static grid, obstacles, and sink."""
    ax.clear()
    ax.set_title(title, fontsize=12, fontweight='bold')
    
    # Grid setup
    ax.set_xticks(np.arange(-0.5, Nx, 1), minor=True)
    ax.set_yticks(np.arange(-0.5, Ny, 1), minor=True)
    ax.grid(which='minor', color='black', linestyle='-', linewidth=1)
    ax.set_xticks(np.arange(0, Nx, 1))
    ax.set_yticks(np.arange(0, Ny, 1))
    ax.set_xlim(-0.5, Nx - 0.5)
    ax.set_ylim(-0.5, Ny - 0.5)
    ax.set_aspect('equal', adjustable='box')
    ax.invert_yaxis() # (0,0) is top-left

    # Obstacles
    for obs_lin in O_lin:
        obs_y, obs_x = np.unravel_index(obs_lin, (Ny, Nx))
        ax.add_patch(patches.Rectangle((obs_x-0.5, obs_y-0.5), 1, 1, facecolor='gray', alpha=0.8, hatch='/'))
    
    # Sink
    ax.plot(sink_x, sink_y, 'h', markersize=15, color='gold', markeredgecolor='black', label='Sink')

def plot_interactive_paths(G, uav_paths, uav_covered_nodes, sink, Rs, Rc,Nx, Ny, O_lin=None, aux_tensor=None, config_path="config.json", x_charge_matrix=None):
    """
    Interactive plot for UAV paths, coverage, and optional battery levels.
    Computes battery consumption if not provided using:
    b_{n,k+1} = b_{n,k} - b_mov(if moved) - b_steady(if steady) + M(if at sink)
    """
    import json
    if O_lin is None: O_lin = []

    # --- Load config for battery parameters ---
    try:
        with open(config_path, 'r') as f:
            cfg = json.load(f)
        batt_cfg = cfg.get("battery_", cfg.get("battery", {}))
        b_mov = batt_cfg.get("b_mov", 2.0)
        b_steady = batt_cfg.get("b_steady", 0.1)
        b_full = batt_cfg.get("b_full", 100.0)
        M = batt_cfg.get("ebase", 5.0)
    except Exception as e:
        print(f"[Warning] Could not load battery config ({e}), using defaults.")
        b_mov, b_steady, b_full, M = 2.0, 0.1, 100.0, 5.0

    def get_T():
        return max((len(p) for p in uav_paths.values() if p), default=0)

    T = get_T()
    if T == 0:
        print("Nothing to plot: all UAV paths are empty.")
        return

    # --- Determine sink linear index ---
    if isinstance(sink, (int, np.integer)):
        sink_lin = sink
    else:
        sink_y, sink_x = sink[0] - 1, sink[1] - 1
        sink_lin = np.ravel_multi_index((sink_y, sink_x), (Ny, Nx))

    # --- Compute or use given battery levels ---
    battery_levels = {}
    if aux_tensor is not None and aux_tensor.ndim == 3 and aux_tensor.shape[0] == T:
        for n in range(len(uav_paths)):
            if n < aux_tensor.shape[1]:
                battery_levels[n] = [aux_tensor[t, n, 0] for t in range(T)]
        print("[Info] Using solver-provided battery data (aux_tensor).")
    else:
        print("[Info] Computing battery levels dynamically using solver xnk (if provided).")
        for n, path in uav_paths.items():
            b = [b_full]
            for t in range(T - 1):
                if path[t] is None or path[t + 1] is None:
                    b.append(b[-1])
                    continue

                moved = path[t] != path[t + 1]
                xnk = 0
                if x_charge_matrix is not None:
                    # Use solver-provided xnk
                    xnk = 1 if x_charge_matrix[t, n] > 0.5 else 0
                else:
                    # Fallback heuristic: at sink
                    xnk = 1 if path[t] == sink_lin else 0

                delta = - (b_mov if moved else b_steady)
                if xnk == 1:
                    delta += M

                new_b = max(0.0, min(b_full, b[-1] + delta))
                b.append(new_b)
            battery_levels[n] = b

    # --- Compute cumulative coverage sets (unchanged) ---
    cumulative_coverage_sets = []
    current_total_coverage = set()
    for t in range(T):
        for n in uav_covered_nodes:
            if t < len(uav_covered_nodes[n]):
                coverage_data = uav_covered_nodes[n][t]
                if isinstance(coverage_data, (list, set, np.ndarray)):
                    current_total_coverage.update(coverage_data)
        current_total_coverage.discard(sink_lin)
        cumulative_coverage_sets.append(current_total_coverage.copy())

    total_coverable_cells = (Nx * Ny) - len(O_lin)
    if sink_lin not in O_lin:
        total_coverable_cells -= 1
    total_coverable_cells = max(1, total_coverable_cells)

    uav_colors = ['red', 'blue', 'green', 'orange', 'purple', 'brown', 'pink', 'cyan']
    fig = plt.figure(figsize=(16, 9))
    gs = fig.add_gridspec(2, 2, width_ratios=[1, 1], height_ratios=[0.6, 0.4])
    ax_grid_pos = fig.add_subplot(gs[0, 0])
    ax_grid_cov = fig.add_subplot(gs[0, 1])
    ax_batt = fig.add_subplot(gs[1, :])

    btn_ax_prev = fig.add_axes([0.35, 0.01, 0.1, 0.04]); button_prev = Button(btn_ax_prev, '⟵ Prev')
    btn_ax_next = fig.add_axes([0.55, 0.01, 0.1, 0.04]); button_next = Button(btn_ax_next, 'Next ⟶')

    state = {'t': 0}

    def render_at_t(t_idx):
        _draw_base_grid(ax_grid_pos, f'UAV Positions at Time: {t_idx}', Nx, Ny, O_lin, *np.unravel_index(sink_lin, (Ny, Nx)))
        for n, path in uav_paths.items():
            color = uav_colors[n % len(uav_colors)]
            path_coords = [np.unravel_index(p, (Ny, Nx)) for p in path if p is not None]
            if path_coords:
                py, px = zip(*path_coords)
                ax_grid_pos.plot(px, py, '-', color=color, alpha=0.3, linewidth=2)
            if t_idx < len(path) and path[t_idx] is not None:
                pos_y, pos_x = np.unravel_index(path[t_idx], (Ny, Nx))
                ax_grid_pos.plot(pos_x, pos_y, 'o', markersize=12, color=color, markeredgecolor='black', label=f'UAV {n+1}')
        ax_grid_pos.legend(loc='upper right')

        # Cumulative coverage
        _draw_base_grid(ax_grid_cov, f'Cumulative Coverage up to Time: {t_idx}', Nx, Ny, O_lin, *np.unravel_index(sink_lin, (Ny, Nx)))
        cells_to_plot = cumulative_coverage_sets[t_idx]
        for lin_idx in cells_to_plot:
            cov_y, cov_x = np.unravel_index(lin_idx, (Ny, Nx))
            ax_grid_cov.add_patch(patches.Rectangle((cov_x-0.5, cov_y-0.5), 1, 1, facecolor='green', alpha=0.4))
        percent = (len(cells_to_plot) / total_coverable_cells) * 100
        ax_grid_cov.text(0.02, 0.98, f'Coverage: {percent:.1f}%', transform=ax_grid_cov.transAxes,
                         fontsize=12, fontweight='bold', ha='left', va='top',
                         bbox=dict(boxstyle='round,pad=0.3', fc='white', alpha=0.8))

        # Battery plot
        ax_batt.clear()
        ax_batt.set_title("Battery Levels over Time", fontweight='bold')
        ax_batt.set_xlabel("Time step")
        ax_batt.set_ylabel("Battery level")
        ax_batt.set_xlim(-0.5, T - 0.5)
        ax_batt.grid(True, linestyle='--', alpha=0.6)
        for n, levels in battery_levels.items():
            color = uav_colors[n % len(uav_colors)]
            ax_batt.plot(range(len(levels)), levels, '-o', color=color, label=f'UAV {n+1}')
            if t_idx < len(levels):
                ax_batt.plot(t_idx, levels[t_idx], 's', color='yellow', markersize=10, markeredgecolor='black')
        ax_batt.legend(loc='upper right')
        fig.canvas.draw_idle()

    def on_next(event=None):
        if state['t'] < T - 1:
            state['t'] += 1
            render_at_t(state['t'])

    def on_prev(event=None):
        if state['t'] > 0:
            state['t'] -= 1
            render_at_t(state['t'])

    def on_key(event):
        if event.key in ('right', 'd', ' '): on_next()
        elif event.key in ('left', 'a'): on_prev()

    button_next.on_clicked(on_next)
    button_prev.on_clicked(on_prev)
    fig.canvas.mpl_connect('key_press_event', on_key)
    render_at_t(0)
    fig.subplots_adjust(left=0.05, right=0.95, bottom=0.1, top=0.93, hspace=0.3, wspace=0.2)
    plt.savefig('plot.png')
    try:
        plt.show()
    except Exception as e:
        print(f"Could not open interactive window: {e}")
        print("Plot saved to plot.png")
