import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import patches
from matplotlib.widgets import Button
from collections import deque

def get_bfs_path(start, end, Nx, Ny, obstacles):
    """
    Finds the shortest path between start and end (linear indices)
    avoiding obstacles using BFS. Returns list of linear indices.
    """
    if start == end:
        return [start]
    
    # Check validity
    if start is None or end is None:
        return []

    # Queue: (current_node, path_so_far)
    queue = deque([(start, [start])])
    visited = {start}
    
    # Convert linear to grid for coordinate math
    target_y, target_x = np.unravel_index(end, (Ny, Nx))
    
    while queue:
        curr, path = queue.popleft()
        if curr == end:
            return path
        
        cy, cx = np.unravel_index(curr, (Ny, Nx))
        
        # Neighbors: Up, Down, Left, Right
        for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            ny, nx = cy + dy, cx + dx
            
            # Bounds check
            if 0 <= ny < Ny and 0 <= nx < Nx:
                n_lin = int(ny * Nx + nx)
                
                # Check obstacles (allow end node even if it technically overlaps for visualization sake)
                if (n_lin not in obstacles or n_lin == end) and n_lin not in visited:
                    visited.add(n_lin)
                    queue.append((n_lin, path + [n_lin]))
    
    # Fallback to straight line if no path found (e.g. enclosed or invalid)
    return [start, end]

def plot_interactive_paths(G, uav_paths, uav_covered_nodes, sink, Rs, Rc,
                           Nx, Ny, O_lin=None, aux_tensor=None):
    """
    Interactive plot for UAV paths (BFS visualization), coverage, and battery.
    Now includes a dashed circle representing the Communication Radius (Rc).
    """
    if O_lin is None:
        O_lin = []
    O_lin_set = set(int(x) for x in O_lin)

    # --- Time horizon ---
    def get_T():
        return max((len(p) for p in uav_paths.values() if p), default=0)

    T = get_T()
    if T == 0:
        print("Nothing to plot: all UAV paths are empty.")
        return

    # --- Battery extraction ---
    consumption = 0
    battery_levels = {}
    if aux_tensor is not None and getattr(aux_tensor, "ndim", None) == 3:
        for n in range(len(uav_paths)):
            battery_levels[n] = [aux_tensor[t, n, 0] for t in range(T)]
            consumption += battery_levels[n][0] - battery_levels[n][T-1]
    else:
        battery_levels = {n: [] for n in range(len(uav_paths))}
    print(f"BATTERY {consumption}")

    # --- Precompute cumulative coverage ---
    total_cells = Nx * Ny
    total_valid = total_cells - len(O_lin_set)
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
                    if 0 <= lin < total_cells and lin not in O_lin_set:
                        covered_so_far.add(lin)
        covered_cumulative.append(set(covered_so_far))

    # --- PRECOMPUTE BFS VISUAL PATHS ---
    visual_paths_coords = {}
    
    for n, path in uav_paths.items():
        full_bfs_path = []
        valid_path = [p for p in path if p is not None]
        
        if len(valid_path) > 0:
            full_bfs_path.append(valid_path[0])
            for i in range(len(valid_path) - 1):
                start_node = valid_path[i]
                end_node = valid_path[i+1]
                segment = get_bfs_path(start_node, end_node, Nx, Ny, O_lin_set)
                full_bfs_path.extend(segment[1:])
        
        coords = [np.unravel_index(p, (Ny, Nx)) for p in full_bfs_path]
        if coords:
            visual_paths_coords[n] = list(zip(*coords))
        else:
            visual_paths_coords[n] = ([], [])

    # --- Sink coords ---
    sink_y, sink_x = np.unravel_index(int(sink), (Ny, Nx))

    # --- Colors/style ---
    uav_colors = ['red', 'blue', 'green', 'orange', 'purple', 'brown', 'pink', 'cyan']

    # --- Figure setup ---
    fig = plt.figure(figsize=(16, 8))
    gs = fig.add_gridspec(2, 2, height_ratios=[1, 0.4], width_ratios=[1, 1])

    ax_grid = fig.add_subplot(gs[0, 0])
    ax_cov  = fig.add_subplot(gs[0, 1])
    ax_batt = fig.add_subplot(gs[1, :])

    # --- UI Buttons ---
    btn_ax_prev = fig.add_axes([0.35, 0.02, 0.1, 0.05]); button_prev = Button(btn_ax_prev, '⟵ Prev')
    btn_ax_next = fig.add_axes([0.55, 0.02, 0.1, 0.05]); button_next = Button(btn_ax_next, 'Next ⟶')

    state = {'t': 0}

    # --- Render function ---
    def render_at_t(t_idx):
        ax_grid.clear()
        ax_cov.clear()
        ax_batt.clear()

        # ===== Left panel: UAV positions & paths =====
        ax_grid.set_title(f'UAV Positions (BFS Path) at Time: {t_idx}', fontsize=14, fontweight='bold')
        ax_grid.set_xticks(np.arange(-0.5, Nx, 1), minor=True)
        ax_grid.set_yticks(np.arange(-0.5, Ny, 1), minor=True)
        ax_grid.grid(which='minor', color='black', linestyle='-', linewidth=1)
        ax_grid.set_xticks(np.arange(0, Nx, 1)); ax_grid.set_yticks(np.arange(0, Ny, 1))
        ax_grid.set_xlim(-0.5, Nx - 0.5); ax_grid.set_ylim(-0.5, Ny - 0.5)
        ax_grid.set_aspect('equal', adjustable='box'); ax_grid.invert_yaxis()

        # Obstacles
        for obs_lin in O_lin_set:
            oy, ox = np.unravel_index(obs_lin, (Ny, Nx))
            ax_grid.add_patch(patches.Rectangle((ox - 0.5, oy - 0.5), 1, 1, facecolor='gray', alpha=0.8, hatch='/'))
        ax_grid.plot(sink_x, sink_y, 'h', markersize=15, color='gold', markeredgecolor='black', label='Sink')

        # UAVs
        for n, path in uav_paths.items():
            color = uav_colors[n % len(uav_colors)]
            
            # Plot Pre-calculated BFS Path
            if n in visual_paths_coords:
                py, px = visual_paths_coords[n]
                ax_grid.plot(px, py, '-', color=color, alpha=0.4, linewidth=2)

            # Per-step coverage shading
            if n in uav_covered_nodes and t_idx < len(uav_covered_nodes[n]):
                for covered_node_lin in (uav_covered_nodes[n][t_idx] or []):
                    # Check if covered node is an obstacle before drawing
                    if int(covered_node_lin) not in O_lin_set:
                        cy, cx = np.unravel_index(int(covered_node_lin), (Ny, Nx))
                        ax_grid.add_patch(patches.Rectangle((cx - 0.5, cy - 0.5), 1, 1, facecolor=color, alpha=0.15))

            # Current position and Communication Circle
            if t_idx < len(path) and path[t_idx] is not None:
                pos_y, pos_x = np.unravel_index(int(path[t_idx]), (Ny, Nx))
                
                # 1. Plot the UAV dot
                ax_grid.plot(pos_x, pos_y, 'o', markersize=12, color=color, markeredgecolor='black', label=f'UAV {n+1}')
                
                # 2. Plot the Communication Radius (dashed circle)
                #    Rc must be valid (e.g. > 0) to make sense, but Circle handles 0 fine (it just becomes invisible)
                comm_circle = patches.Circle(
                    (pos_x, pos_y), 
                    radius=Rc, 
                    edgecolor=color, 
                    facecolor='none', 
                    linestyle='--', 
                    linewidth=1.5,
                    alpha=0.6
                )
                ax_grid.add_patch(comm_circle)

        ax_grid.legend(loc='upper right')

        # ===== Right panel: Coverage =====
        ax_cov.set_xticks(np.arange(-0.5, Nx, 1), minor=True)
        ax_cov.set_yticks(np.arange(-0.5, Ny, 1), minor=True)
        ax_cov.grid(which='minor', color='black', linestyle='-', linewidth=1)
        ax_cov.set_xticks(np.arange(0, Nx, 1)); ax_cov.set_yticks(np.arange(0, Ny, 1))
        ax_cov.set_xlim(-0.5, Nx - 0.5); ax_cov.set_ylim(-0.5, Ny - 0.5)
        ax_cov.set_aspect('equal', adjustable='box'); ax_cov.invert_yaxis()

        for obs_lin in O_lin_set:
            oy, ox = np.unravel_index(obs_lin, (Ny, Nx))
            ax_cov.add_patch(patches.Rectangle((ox - 0.5, oy - 0.5), 1, 1, facecolor='gray', alpha=0.8, hatch='/'))

        covered_now = covered_cumulative[t_idx]
        for lin in covered_now:
            ry, rx = np.unravel_index(int(lin), (Ny, Nx))
            ax_cov.add_patch(patches.Rectangle((rx - 0.5, ry - 0.5), 1, 1, facecolor=(0.0, 0.7, 0.0), alpha=0.6))

        pct = (100.0 * len(covered_now) / max(1, total_valid-1))
        ax_cov.set_title(f'Cumulative Coverage = {pct:.2f}% ({len(covered_now)}/{total_valid-1})', fontsize=14, fontweight='bold')
        
        print(f"t={t_idx} | Coverage: {pct:.2f}%")

        # ===== Bottom panel: Battery =====
        ax_batt.set_title("Battery Levels over Time", fontweight='bold')
        ax_batt.set_xlabel("Time step"); ax_batt.set_ylabel("Battery level")
        ax_batt.set_xlim(-0.5, T - 0.5)
        vals = [v for v in battery_levels.values() if v]
        if vals:
            ymin = min(min(v) for v in vals if v)
            ymax = max(max(v) for v in vals if v)
            if ymin != ymax:
                ax_batt.set_ylim(ymin * 0.95, ymax * 1.05)
        for n, levels in battery_levels.items():
            if len(levels) == T:
                color = uav_colors[n % len(uav_colors)]
                ax_batt.plot(range(T), levels, '-o', color=color, label=f'UAV {n+1}')
                ax_batt.plot(t_idx, levels[t_idx], 's', color='yellow', markersize=10, markeredgecolor='black')
        ax_batt.legend(loc='upper right')
        ax_batt.grid(True, linestyle='--', alpha=0.6)

        fig.canvas.draw_idle()

    # --- Handlers ---
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

    button_next.on_clicked(on_next)
    button_prev.on_clicked(on_prev)
    fig.canvas.mpl_connect('key_press_event', on_key)

    # render_at_t(0)
    # plt.tight_layout(rect=[0, 0.1, 1, 1])
    # plt.show()
    return render_at_t, T


def animate_paths(G, uav_paths, uav_covered_nodes, sink, Rs, Rc,
                  Nx, Ny, O_lin=None, aux_tensor=None,
                  filename="uav_animation.mp4", fps=2):
    """
    Save an animation of UAV paths (BFS visualization), coverage, and battery.
    Now includes a dashed circle representing the Communication Radius (Rc).
    """
    if O_lin is None:
        O_lin = []
    O_lin_set = set(int(x) for x in O_lin)

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
        for n in range(len(uav_paths)):
            battery_levels[n] = [aux_tensor[t, n, 0] for t in range(T)]
    else:
        battery_levels = {n: [] for n in range(len(uav_paths))}

    # --- Precompute cumulative coverage ---
    total_cells = Nx * Ny
    total_valid = total_cells - len(O_lin_set)
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
                    if 0 <= lin < total_cells and lin not in O_lin_set:
                        covered_so_far.add(lin)
        covered_cumulative.append(set(covered_so_far))

    # --- PRECOMPUTE BFS VISUAL PATHS ---
    visual_paths_coords = {}
    for n, path in uav_paths.items():
        full_bfs_path = []
        valid_path = [p for p in path if p is not None]
        if len(valid_path) > 0:
            full_bfs_path.append(valid_path[0])
            for i in range(len(valid_path) - 1):
                start_node = valid_path[i]
                end_node = valid_path[i+1]
                segment = get_bfs_path(start_node, end_node, Nx, Ny, O_lin_set)
                full_bfs_path.extend(segment[1:])
        
        coords = [np.unravel_index(p, (Ny, Nx)) for p in full_bfs_path]
        if coords:
            visual_paths_coords[n] = list(zip(*coords))
        else:
            visual_paths_coords[n] = ([], [])

    # --- Sink coords ---
    sink_y, sink_x = np.unravel_index(int(sink), (Ny, Nx))
    uav_colors = ['red', 'blue', 'green', 'orange', 'purple', 'brown', 'pink', 'cyan']

    # --- Figure layout ---
    fig = plt.figure(figsize=(16, 8))
    gs = fig.add_gridspec(2, 2, height_ratios=[1, 0.4], width_ratios=[1, 1])
    ax_grid = fig.add_subplot(gs[0, 0])
    ax_cov  = fig.add_subplot(gs[0, 1])
    ax_batt = fig.add_subplot(gs[1, :])

    def render_at_t(t_idx):
        ax_grid.clear()
        ax_cov.clear()
        ax_batt.clear()

        # ===== UAV positions & BFS Paths =====
        ax_grid.set_title(f'UAV Positions (BFS Path) at Time: {t_idx}', fontsize=14, fontweight='bold')
        ax_grid.set_xticks(np.arange(-0.5, Nx, 1), minor=True)
        ax_grid.set_yticks(np.arange(-0.5, Ny, 1), minor=True)
        ax_grid.grid(which='minor', color='black', linestyle='-', linewidth=1)
        ax_grid.set_xticks(np.arange(0, Nx, 1)); ax_grid.set_yticks(np.arange(0, Ny, 1))
        ax_grid.set_xlim(-0.5, Nx - 0.5); ax_grid.set_ylim(-0.5, Ny - 0.5)
        ax_grid.set_aspect('equal', adjustable='box'); ax_grid.invert_yaxis()

        for obs_lin in O_lin_set:
            oy, ox = np.unravel_index(obs_lin, (Ny, Nx))
            ax_grid.add_patch(patches.Rectangle((ox - 0.5, oy - 0.5), 1, 1, facecolor='gray', alpha=0.8, hatch='/'))
        ax_grid.plot(sink_x, sink_y, 'h', markersize=15, color='gold', markeredgecolor='black', label='Sink')

        for n, path in uav_paths.items():
            color = uav_colors[n % len(uav_colors)]
            
            # Plot BFS Path
            if n in visual_paths_coords:
                py, px = visual_paths_coords[n]
                ax_grid.plot(px, py, '-', color=color, alpha=0.4, linewidth=2)

            if n in uav_covered_nodes and t_idx < len(uav_covered_nodes[n]):
                for covered_node_lin in (uav_covered_nodes[n][t_idx] or []):
                    # Check if covered node is an obstacle before drawing
                    if int(covered_node_lin) not in O_lin_set:
                        cy, cx = np.unravel_index(int(covered_node_lin), (Ny, Nx))
                        ax_grid.add_patch(patches.Rectangle((cx - 0.5, cy - 0.5), 1, 1, facecolor=color, alpha=0.15))

            if t_idx < len(path) and path[t_idx] is not None:
                pos_y, pos_x = np.unravel_index(int(path[t_idx]), (Ny, Nx))
                
                # 1. Plot UAV Dot
                ax_grid.plot(pos_x, pos_y, 'o', markersize=12, color=color, markeredgecolor='black', label=f'UAV {n+1}')
                
                # 2. Plot Communication Circle
                comm_circle = patches.Circle(
                    (pos_x, pos_y), 
                    radius=Rc, 
                    edgecolor=color, 
                    facecolor='none', 
                    linestyle='--', 
                    linewidth=1.5,
                    alpha=0.6
                )
                ax_grid.add_patch(comm_circle)
                
        ax_grid.legend(loc='upper right')

        # ===== Cumulative coverage =====
        ax_cov.set_xticks(np.arange(-0.5, Nx, 1), minor=True)
        ax_cov.set_yticks(np.arange(-0.5, Ny, 1), minor=True)
        ax_cov.grid(which='minor', color='black', linestyle='-', linewidth=1)
        ax_cov.set_xticks(np.arange(0, Nx, 1)); ax_cov.set_yticks(np.arange(0, Ny, 1))
        ax_cov.set_xlim(-0.5, Nx - 0.5); ax_cov.set_ylim(-0.5, Ny - 0.5)
        ax_cov.set_aspect('equal', adjustable='box'); ax_cov.invert_yaxis()

        for obs_lin in O_lin_set:
            oy, ox = np.unravel_index(obs_lin, (Ny, Nx))
            ax_cov.add_patch(patches.Rectangle((ox - 0.5, oy - 0.5), 1, 1, facecolor='gray', alpha=0.8, hatch='/'))

        covered_now = covered_cumulative[t_idx]
        for lin in covered_now:
            ry, rx = np.unravel_index(int(lin), (Ny, Nx))
            ax_cov.add_patch(patches.Rectangle((rx - 0.5, ry - 0.5), 1, 1, facecolor=(0.0, 0.7, 0.0), alpha=0.6))
        pct = (100.0 * len(covered_now) / max(1, total_valid-1))
        ax_cov.set_title(f'Cumulative Coverage = {pct:.2f}% ({len(covered_now)}/{total_valid-1})', fontsize=14, fontweight='bold')

        # ===== Battery =====
        ax_batt.set_title("Battery Levels over Time", fontweight='bold')
        ax_batt.set_xlabel("Time step"); ax_batt.set_ylabel("Battery level")
        ax_batt.set_xlim(-0.5, T - 0.5)
        vals = [v for v in battery_levels.values() if v]
        if vals:
            ymin = min(min(v) for v in vals if v)
            ymax = max(max(v) for v in vals if v)
            if ymin != ymax:
                ax_batt.set_ylim(ymin * 0.95, ymax * 1.05)
        for n, levels in battery_levels.items():
            if len(levels) == T:
                color = uav_colors[n % len(uav_colors)]
                ax_batt.plot(range(T), levels, '-o', color=color, label=f'UAV {n+1}')
                ax_batt.plot(t_idx, levels[t_idx], 's', color='yellow', markersize=10, markeredgecolor='black')
        ax_batt.legend(loc='upper right')
        ax_batt.grid(True, linestyle='--', alpha=0.6)

    ani = animation.FuncAnimation(fig, render_at_t, frames=T, interval=1000/fps, repeat=False)

    if filename.endswith(".mp4"):
        ani.save(filename, writer="ffmpeg", fps=fps)
    elif filename.endswith(".gif"):
        ani.save(filename, writer="pillow", fps=fps)
    else:
        raise ValueError("Unsupported output format. Use .mp4 or .gif")

    plt.close(fig)
    print(f"Animation saved to {filename}")