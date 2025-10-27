import matplotlib.animation as animation
import matplotlib
# Set a backend that supports UI
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import patches
from matplotlib.widgets import Button


def plot_interactive_paths(G, uav_paths, uav_covered_nodes, sink, Rs, Rc,
                           Nx, Ny, O_lin=None, aux_tensor=None):
    """
    Interactive plot for UAV paths, coverage, and optional battery levels.

    Left top panel: UAV positions, paths, per-step coverage shading (per UAV).
    Right top panel: CUMULATIVE coverage grid (green = sensed so far).
    Bottom panel: Battery levels over time (if provided via aux_tensor).
    """

    if O_lin is None:
        O_lin = []
    O_lin_set = set(int(x) for x in O_lin)
    sink_lin = int(sink) # Get the linear index of the sink

    # --- Time horizon ---
    def get_T():
        return max((len(p) for p in uav_paths.values() if p), default=0)

    T = get_T()
    if T == 0:
        print("Nothing to plot: all UAV paths are empty.")
        return

    # --- Battery extraction (if provided) ---
    battery_levels = {}
    if aux_tensor is not None and getattr(aux_tensor, "ndim", None) == 3:
        # Ensure aux_tensor has enough time steps and UAVs
        T_aux, N_aux, _ = aux_tensor.shape
        for n in range(len(uav_paths)):
            if n < N_aux:
                 # Only read up to the shortest of T or T_aux
                battery_levels[n] = [aux_tensor[t, n, 0] for t in range(min(T, T_aux))]
    else:
        battery_levels = {n: [] for n in range(len(uav_paths))}

    # --- Precompute cumulative coverage over time (union across UAVs) ---
    total_cells = Nx * Ny
    
    # *** FIX HERE: `total_valid` now excludes obstacles AND the sink ***
    valid_cells_to_cover = set(range(total_cells)) - O_lin_set - {sink_lin}
    total_valid = len(valid_cells_to_cover)
    
    covered_cumulative = []
    covered_so_far = set()
    for t in range(T):
        # Add coverage from all UAVs at this timestep
        for n in uav_paths.keys():
            # Guard for ragged inputs
            if n in uav_covered_nodes and t < len(uav_covered_nodes[n]):
                nodes_t = uav_covered_nodes[n][t]
                if nodes_t is None:
                    continue
                for lin in nodes_t:
                    lin = int(lin)
                    # Only count cells that are part of the valid set
                    if lin in valid_cells_to_cover:
                        covered_so_far.add(lin)
        covered_cumulative.append(set(covered_so_far))  # store a copy

    # --- Sink (y,x) from linear index ---
    sink_y, sink_x = np.unravel_index(sink_lin, (Ny, Nx))

    # --- Colors/style ---
    uav_colors = ['red', 'blue', 'green', 'orange', 'purple', 'brown', 'pink', 'cyan']

    # --- Figure layout: top row = [paths | cumulative coverage], bottom row = battery ---
    fig = plt.figure(figsize=(16, 8))
    gs = fig.add_gridspec(2, 2, height_ratios=[1, 0.4], width_ratios=[1, 1])

    ax_grid = fig.add_subplot(gs[0, 0])  # left top: UAV positions & per-step coverage
    ax_cov  = fig.add_subplot(gs[0, 1])  # right top: cumulative coverage
    ax_batt = fig.add_subplot(gs[1, :])  # bottom: battery

    # --- UI Buttons ---
    btn_ax_prev = fig.add_axes([0.35, 0.02, 0.1, 0.05]); button_prev = Button(btn_ax_prev, '⟵ Prev')
    btn_ax_next = fig.add_axes([0.55, 0.02, 0.1, 0.05]); button_next = Button(btn_ax_next, 'Next ⟶')

    state = {'t': 0}

    # --- Render one time step ---
    def render_at_t(t_idx):
        ax_grid.clear()
        ax_cov.clear()
        ax_batt.clear()

        # ===== Left panel: UAV positions & per-step coverage =====
        ax_grid.set_title(f'UAV Positions and Coverage at Time: {t_idx}', fontsize=14, fontweight='bold')
        # Grid styling
        ax_grid.set_xticks(np.arange(-0.5, Nx, 1), minor=True)
        ax_grid.set_yticks(np.arange(-0.5, Ny, 1), minor=True)
        ax_grid.grid(which='minor', color='black', linestyle='-', linewidth=1)
        ax_grid.set_xticks(np.arange(0, Nx, 1)); ax_grid.set_yticks(np.arange(0, Ny, 1))
        ax_grid.set_xlim(-0.5, Nx - 0.5); ax_grid.set_ylim(-0.5, Ny - 0.5)
        ax_grid.set_aspect('equal', adjustable='box'); ax_grid.invert_yaxis()

        # Obstacles (gray) and Sink (gold)
        for obs_lin in O_lin_set:
            oy, ox = np.unravel_index(obs_lin, (Ny, Nx))
            ax_grid.add_patch(patches.Rectangle((ox - 0.5, oy - 0.5), 1, 1, facecolor='gray', alpha=0.8, hatch='/'))
        ax_grid.plot(sink_x, sink_y, 'h', markersize=15, color='gold', markeredgecolor='black', label='Sink')

        # UAVs: full path (faint), per-step coverage (light fill), current position (marker)
        for n, path in uav_paths.items():
            color = uav_colors[n % len(uav_colors)]
            # Full path faintly
            path_coords = [np.unravel_index(p, (Ny, Nx)) for p in path if p is not None]
            if path_coords:
                py, px = zip(*path_coords)
                ax_grid.plot(px, py, '-', color=color, alpha=0.3, linewidth=2)

            # Per-step coverage shading for this UAV at t
            if n in uav_covered_nodes and t_idx < len(uav_covered_nodes[n]):
                for covered_node_lin in (uav_covered_nodes[n][t_idx] or []):
                    cy, cx = np.unravel_index(int(covered_node_lin), (Ny, Nx))
                    ax_grid.add_patch(patches.Rectangle((cx - 0.5, cy - 0.5), 1, 1, facecolor=color, alpha=0.15))

            # Current position marker
            if t_idx < len(path) and path[t_idx] is not None:
                pos_y, pos_x = np.unravel_index(int(path[t_idx]), (Ny, Nx))
                ax_grid.plot(pos_x, pos_y, 'o', markersize=12, color=color, markeredgecolor='black', label=f'UAV {n+1}')

        ax_grid.legend(loc='upper right')

        # ===== Right panel: CUMULATIVE coverage grid =====
        # Grid styling
        ax_cov.set_xticks(np.arange(-0.5, Nx, 1), minor=True)
        ax_cov.set_yticks(np.arange(-0.5, Ny, 1), minor=True)
        ax_cov.grid(which='minor', color='black', linestyle='-', linewidth=1)
        ax_cov.set_xticks(np.arange(0, Nx, 1)); ax_cov.set_yticks(np.arange(0, Ny, 1))
        ax_cov.set_xlim(-0.5, Nx - 0.5); ax_cov.set_ylim(-0.5, Ny - 0.5)
        ax_cov.set_aspect('equal', adjustable='box'); ax_cov.invert_yaxis()

        # Obstacles (gray)
        for obs_lin in O_lin_set:
            oy, ox = np.unravel_index(obs_lin, (Ny, Nx))
            ax_cov.add_patch(patches.Rectangle((ox - 0.5, oy - 0.5), 1, 1, facecolor='gray', alpha=0.8, hatch='/'))
        # Sink (gold outline)
        ax_cov.add_patch(patches.Rectangle((sink_x-0.5, sink_y-0.5), 1, 1, facecolor='none', edgecolor='gold', linewidth=2, hatch='xx'))


        # Cumulative coverage until t
        covered_now = covered_cumulative[t_idx]
        for lin in covered_now:
            ry, rx = np.unravel_index(int(lin), (Ny, Nx))
            ax_cov.add_patch(patches.Rectangle((rx - 0.5, ry - 0.5), 1, 1, facecolor=(0.0, 0.7, 0.0), alpha=0.6))

        # Coverage percentage (using corrected total_valid)
        pct = (100.0 * len(covered_now) / max(1, total_valid))
        ax_cov.set_title(f'Cumulative Sensed Coverage = {pct:.2f}%  '
                         f'({len(covered_now)}/{total_valid})',
                         fontsize=14, fontweight='bold')

        # Also print to console
        # print(f"t={t_idx} | Coverage: {pct:.2f}% ({len(covered_now)}/{total_valid})")

        # ===== Bottom panel: Battery =====
        ax_batt.set_title("Battery Levels over Time", fontweight='bold')
        ax_batt.set_xlabel("Time step"); ax_batt.set_ylabel("Battery level")
        ax_batt.set_xlim(-0.5, T - 0.5)
        if battery_levels and any(battery_levels.values()):
            vals = [v for v in battery_levels.values() if v]
            if vals:
                all_vals = [item for sublist in vals for item in sublist]
                if all_vals:
                    ymin, ymax = min(all_vals), max(all_vals)
                    if ymin != ymax:
                        # Ensure ylim includes 0 and 100 if they are close
                        plot_min = min(0, ymin * 1.1)
                        plot_max = max(100, ymax * 1.1)
                        ax_batt.set_ylim(plot_min , plot_max)
                    else:
                        ax_batt.set_ylim(ymin - 1, ymin + 1)
                        
        for n, levels in battery_levels.items():
            if t_idx < len(levels): # Plot full line up to T
                color = uav_colors[n % len(uav_colors)]
                ax_batt.plot(range(len(levels)), levels, '-o', color=color, label=f'UAV {n+1}')
                # Plot current time marker
                ax_batt.plot(t_idx, levels[t_idx], 's', color='yellow', markersize=10, markeredgecolor='black')
        ax_batt.legend(loc='upper right')
        ax_batt.grid(True, linestyle='--', alpha=0.6)

        fig.canvas.draw_idle()

    # --- Button & keyboard handlers ---
    def on_next(event=None):
        if state['t'] < T - 1:
            state['t'] += 1
            render_at_t(state['t'])

    def on_prev(event=None):
        if state['t'] > 0:
            state['t'] -= 1
            render_at_t(state['t'])

    def on_key(event):
        if event.key in ('right', 'd', ' '):
            on_next()
        elif event.key in ('left', 'a', 'p'):
            on_prev()

    # --- Wire up and show ---
    button_next.on_clicked(on_next)
    button_prev.on_clicked(on_prev)
    fig.canvas.mpl_connect('key_press_event', on_key)

    render_at_t(0)
    plt.tight_layout(rect=[0, 0.1, 1, 1])
    plt.show()
    return render_at_t, T


def animate_paths(G, uav_paths, uav_covered_nodes, sink, Rs, Rc,
                  Nx, Ny, O_lin=None, aux_tensor=None,
                  filename="uav_animation.mp4", fps=2):
    """
    Save an animation of UAV paths, coverage, and optional battery levels.

    Args:
        filename: output filename (.mp4 or .gif)
        fps: frames per second
    """
    if O_lin is None:
        O_lin = []
    O_lin_set = set(int(x) for x in O_lin)
    sink_lin = int(sink) # Get the linear index of the sink

    # --- Time horizon ---
    def get_T():
        return max((len(p) for p in uav_paths.values() if p), default=0)

    T = get_T()
    if T == 0:
        print("Nothing to animate: all UAV paths are empty.")
        return

    # --- Battery extraction ---
    battery_levels = {}
    if aux_tensor is not None and getattr(aux_tensor, "ndim", None) == 3:
        T_aux, N_aux, _ = aux_tensor.shape
        for n in range(len(uav_paths)):
            if n < N_aux:
                battery_levels[n] = [aux_tensor[t, n, 0] for t in range(min(T, T_aux))]
    else:
        battery_levels = {n: [] for n in range(len(uav_paths))}

    # --- Precompute cumulative coverage ---
    total_cells = Nx * Ny
    
    # *** FIX HERE: `total_valid` now excludes obstacles AND the sink ***
    valid_cells_to_cover = set(range(total_cells)) - O_lin_set - {sink_lin}
    total_valid = len(valid_cells_to_cover)

    covered_cumulative = []
    covered_so_far = set()
    for t in range(T):
        for n in uav_paths.keys():
            if n in uav_covered_nodes and t < len(uav_covered_nodes[n]):
                nodes_t = uav_covered_nodes[n][t]
                if nodes_t is None:
                    continue
                for lin in nodes_t:
                    lin = int(lin)
                    if lin in valid_cells_to_cover:
                        covered_so_far.add(lin)
        covered_cumulative.append(set(covered_so_far))

    # --- Sink coords ---
    sink_y, sink_x = np.unravel_index(sink_lin, (Ny, Nx))

    # --- Colors ---
    uav_colors = ['red', 'blue', 'green', 'orange', 'purple', 'brown', 'pink', 'cyan']

    # --- Figure layout ---
    fig = plt.figure(figsize=(16, 8))
    gs = fig.add_gridspec(2, 2, height_ratios=[1, 0.4], width_ratios=[1, 1])
    ax_grid = fig.add_subplot(gs[0, 0])
    ax_cov  = fig.add_subplot(gs[0, 1])
    ax_batt = fig.add_subplot(gs[1, :])

    # --- Rendering function for animation ---
    def render_at_t(t_idx):
        ax_grid.clear()
        ax_cov.clear()
        ax_batt.clear()

        # ===== UAV positions & coverage =====
        ax_grid.set_title(f'UAV Positions and Coverage at Time: {t_idx}', fontsize=14, fontweight='bold')
        ax_grid.set_xticks(np.arange(-0.5, Nx, 1), minor=True)
        ax_grid.set_yticks(np.arange(-0.5, Ny, 1), minor=True)
        ax_grid.grid(which='minor', color='black', linestyle='-', linewidth=1)
        ax_grid.set_xlim(-0.5, Nx - 0.5); ax_grid.set_ylim(-0.5, Ny - 0.5)
        ax_grid.set_aspect('equal', adjustable='box'); ax_grid.invert_yaxis()

        for obs_lin in O_lin_set:
            oy, ox = np.unravel_index(obs_lin, (Ny, Nx))
            ax_grid.add_patch(patches.Rectangle((ox - 0.5, oy - 0.5), 1, 1,
                                                facecolor='gray', alpha=0.8, hatch='/'))
        ax_grid.plot(sink_x, sink_y, 'h', markersize=15, color='gold', markeredgecolor='black', label='Sink')

        for n, path in uav_paths.items():
            color = uav_colors[n % len(uav_colors)]
            path_coords = [np.unravel_index(p, (Ny, Nx)) for p in path if p is not None]
            if path_coords:
                py, px = zip(*path_coords)
                ax_grid.plot(px, py, '-', color=color, alpha=0.3, linewidth=2)

            if n in uav_covered_nodes and t_idx < len(uav_covered_nodes[n]):
                for covered_node_lin in (uav_covered_nodes[n][t_idx] or []):
                    cy, cx = np.unravel_index(int(covered_node_lin), (Ny, Nx))
                    ax_grid.add_patch(patches.Rectangle((cx - 0.5, cy - 0.5), 1, 1,
                                                        facecolor=color, alpha=0.15))

            if t_idx < len(path) and path[t_idx] is not None:
                pos_y, pos_x = np.unravel_index(int(path[t_idx]), (Ny, Nx))
                ax_grid.plot(pos_x, pos_y, 'o', markersize=12, color=color,
                             markeredgecolor='black', label=f'UAV {n+1}')
        ax_grid.legend(loc='upper right')

        # ===== Cumulative coverage =====
        ax_cov.set_xticks(np.arange(-0.5, Nx, 1), minor=True)
        ax_cov.set_yticks(np.arange(-0.5, Ny, 1), minor=True)
        ax_cov.grid(which='minor', color='black', linestyle='-', linewidth=1)
        ax_cov.set_xlim(-0.5, Nx - 0.5); ax_cov.set_ylim(-0.5, Ny - 0.5)
        ax_cov.set_aspect('equal', adjustable='box'); ax_cov.invert_yaxis()

        for obs_lin in O_lin_set:
            oy, ox = np.unravel_index(obs_lin, (Ny, Nx))
            ax_cov.add_patch(patches.Rectangle((ox - 0.5, oy - 0.5), 1, 1,
                                               facecolor='gray', alpha=0.8, hatch='/'))
        # Sink (gold outline)
        ax_cov.add_patch(patches.Rectangle((sink_x-0.5, sink_y-0.5), 1, 1, facecolor='none', edgecolor='gold', linewidth=2, hatch='xx'))

        covered_now = covered_cumulative[t_idx]
        for lin in covered_now:
            ry, rx = np.unravel_index(int(lin), (Ny, Nx))
            ax_cov.add_patch(patches.Rectangle((rx - 0.5, ry - 0.5), 1, 1,
                                               facecolor=(0.0, 0.7, 0.0), alpha=0.6))
        pct = (100.0 * len(covered_now) / max(1, total_valid))
        ax_cov.set_title(f'Cumulative Sensed Coverage = {pct:.2f}%  '
                         f'({len(covered_now)}/{total_valid})',
                         fontsize=14, fontweight='bold')

        # ===== Battery =====
        ax_batt.set_title("Battery Levels over Time", fontweight='bold')
        ax_batt.set_xlabel("Time step"); ax_batt.set_ylabel("Battery level")
        ax_batt.set_xlim(-0.5, T - 0.5)
        vals = [v for v in battery_levels.values() if v]
        if vals:
            all_vals = [item for sublist in vals for item in sublist]
            if all_vals:
                ymin, ymax = min(all_vals), max(all_vals)
                if ymin != ymax:
                    plot_min = min(0, ymin * 1.1)
                    plot_max = max(100, ymax * 1.1)
                    ax_batt.set_ylim(plot_min , plot_max)
                else:
                    ax_batt.set_ylim(ymin - 1, ymin + 1)
                    
        for n, levels in battery_levels.items():
            if t_idx < len(levels):
                color = uav_colors[n % len(uav_colors)]
                ax_batt.plot(range(len(levels)), levels, '-o', color=color, label=f'UAV {n+1}')
                ax_batt.plot(t_idx, levels[t_idx], 's', color='yellow',
                             markersize=10, markeredgecolor='black')
        ax_batt.legend(loc='upper right')
        ax_batt.grid(True, linestyle='--', alpha=0.6)

    # --- Build animation ---
    print(f"Building animation ({T} frames)...")
    ani = animation.FuncAnimation(fig, render_at_t, frames=T, interval=1000/fps, repeat=False, blit=False)

    # Save as video or gif
    try:
        if filename.endswith(".mp4"):
            ani.save(filename, writer="ffmpeg", fps=fps)
        elif filename.endswith(".gif"):
            ani.save(filename, writer="pillow", fps=fps)
        else:
            raise ValueError("Unsupported output format. Use .mp4 or .gif")
        print(f"Animation saved to {filename}")
    except Exception as e:
        print(f"\nCould not save animation. Error: {e}")
        print("Please ensure 'ffmpeg' (for .mp4) or 'pillow' (for .gif) is installed.")
    
    plt.close(fig)