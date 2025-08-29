import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib.colors import ListedColormap
from matplotlib.widgets import Button

def plot_interactive_paths(G, uav_paths, uav_covered_nodes, sink, Rs, Rc,Nx, Ny, O_lin = None):
    """
    Interactive plot: shows grid, per-UAV positions & paths at time t,
    and coverage summary. Click Next/Prev (or use Left/Right arrows) to step time.

    Args:
        G : (unused here, kept for signature compatibility)
        uav_paths : dict[int -> list[(row,col)]], 1-based coordinates
        uav_covered_nodes :
            - dict[int -> iterable[(row,col)]]  (overall coverage), OR
            - dict[int -> list[ iterable[(row,col)] ]] (per-time coverage)
        sink : (row, col) 1-based
        Rs : sensing radius (shown as circle at UAV's current cell)
        Nx, Ny : grid size
    """
    if(O_lin is None):
        O_lin = []
    # ---------- Helpers ----------
    def get_T():
        """Infer total T from the longest path."""
        if not uav_paths:
            return 0
        return max((len(path) for path in uav_paths.values() if path), default=0)

    
    # --- helper: normalize any node into a 1-based (row, col) tuple ---
    def to_rc1(node):
        """
        Accepts:
        - (r,c) as list/tuple/ndarray (0-based or 1-based)
        - linear index (int, 0-based or 1-based — we’ll try to infer)
        Returns: (r,c) 1-based tuple
        """
        # numpy scalar → python int
        if isinstance(node, (np.integer, int)):
            li = int(node)
            # assume 0-based linear index by default
            r0, c0 = np.unravel_index(li, (Nx, Ny))
            return (r0+1, c0+1)  # to 1-based
        # array-like
        try:
            r, c = int(node[0]), int(node[1])
        except Exception:
            # last resort: treat as scalar linear index
            li = int(node)
            r0, c0 = np.unravel_index(li, (Nx, Ny))
            return (r0+1, c0+1)

        # decide if 0-based or 1-based
        # If clearly in 1..Nx and 1..Ny, keep; else if 0..Nx-1, convert to 1-based
        if 1 <= r <= Nx and 1 <= c <= Ny:
            return (r, c)
        elif 0 <= r < Nx and 0 <= c < Ny:
            return (r+1, c+1)
        else:
            # if out-of-range, clamp to grid (optional) or raise
            r = min(max(r, 1), Nx)
            c = min(max(c, 1), Ny)
            return (r, c)

        # --- rest of your function setup ... ---

    cumulative_covered = set()

    def coverage_at_time(t_idx):
        """
        Returns cumulative coverage up to and including time t_idx.
        Once a node is covered, it stays covered.
        """
        nonlocal cumulative_covered   # let it persist across calls

        # for each UAV, compute coverage at this time
        for uav_id, path in uav_paths.items():
            if not path:
                continue
            # current position (clamp if path shorter than t_idx)
            pos = path[min(t_idx, len(path)-1)]
            r, c = pos

            # add all grid cells within Rs
            for i in range(1, Nx+1):
                for j in range(1, Ny+1):
                    if (i-r)**2 + (j-c)**2 <= Rs**2:
                        cumulative_covered.add((i, j))

        return set(cumulative_covered)

        # … keep the rest of your plotting code unchanged …


    def path_prefix(path, t_idx):
        """Return path up to and including time t_idx."""
        if not path:
            return []
        return path[:min(len(path), t_idx+1)]

    def draw_grid(ax):
        ax.clear()
        # Draw grid cells
        for i in range(1, Nx+1):
            for j in range(1, Ny+1):
                rect = patches.Rectangle((j-0.5, i-0.5), 1, 1,
                                        linewidth=1, edgecolor='gray',
                                        facecolor='lightgray', alpha=0.3)
                ax.add_patch(rect)
                ax.text(j, i, f'({i},{j})', ha='center', va='center', fontsize=8, color='black')

        # Paint obstacles red
        for li in O_lin:
            r0, c0 = np.unravel_index(li, (Nx, Ny))   # 0-based
            r, c = r0+1, c0+1                         # convert to 1-based
            rect = patches.Rectangle((c-0.5, r-0.5), 1, 1,
                                    linewidth=2, edgecolor='red',
                                    facecolor='red', alpha=0.6)
            ax.add_patch(rect)

        # Sink highlight
        sink_rect = patches.Rectangle((sink[1]-0.5, sink[0]-0.5), 1, 1,
                                    linewidth=3, edgecolor='black',
                                    facecolor='yellow', alpha=0.8)
        ax.add_patch(sink_rect)
        ax.text(sink[1], sink[0], 'SINK', ha='center', va='center', fontweight='bold', color='black')

        ax.set_xlim(0.5, Ny+0.5)
        ax.set_ylim(0.5, Nx+0.5)
        ax.set_xlabel('Column (Y)', fontweight='bold')
        ax.set_ylabel('Row (X)', fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')


    # ---------- Setup ----------
    T = get_T()
    if T == 0:
        print("Nothing to plot: all UAV paths are empty.")
        return

    uav_colors = ['red', 'blue', 'green', 'orange', 'purple', 'teal', 'magenta', 'brown']

    fig = plt.figure(figsize=(16, 9))
    gs = fig.add_gridspec(2, 3, width_ratios=[1, 1, 0.12], height_ratios=[1, 0.15], wspace=0.25, hspace=0.35)
    ax1 = fig.add_subplot(gs[0, 0])  # main grid view
    ax2 = fig.add_subplot(gs[0, 1])  # coverage summary
    ax_controls = fig.add_subplot(gs[1, 0:2])  # for labels/buttons background
    ax_legend_pad = fig.add_subplot(gs[:, 2])  # legend / colorbar column
    ax_controls.axis('off')
    ax_legend_pad.axis('off')

    # Buttons
    btn_ax_prev = fig.add_axes([0.15, 0.08, 0.1, 0.06])
    btn_ax_next = fig.add_axes([0.27, 0.08, 0.1, 0.06])
    button_prev = Button(btn_ax_prev, '⟵ Prev')
    button_next = Button(btn_ax_next, 'Next ⟶')

    # State
    state = {'t': 0}

    # ---------- Draw functions ----------
    def render_at_t(t_idx):
        # Plot 1: grid with current positions and paths up to t
        draw_grid(ax1)
        ax1.set_title(f'UAV Paths & Coverage (t = {t_idx+1}/{T})', fontsize=14, fontweight='bold')

        # Draw per-UAV
        for uav_id, path in uav_paths.items():
            color = uav_colors[uav_id % len(uav_colors)]
            prefix = path_prefix(path, t_idx)
            if not prefix:
                continue

            # Path up to t
            path_y = [pos[0] for pos in prefix]
            path_x = [pos[1] for pos in prefix]
            ax1.plot(path_x, path_y, color=color, linewidth=3, marker='o', markersize=6,
                     label=f'UAV {uav_id+1}', alpha=0.9)

            # Direction arrows
            for i in range(len(path_x)-1):
                ax1.annotate('', xy=(path_x[i+1], path_y[i+1]), xytext=(path_x[i], path_y[i]),
                             arrowprops=dict(arrowstyle='->', color=color, lw=2))

            # Current position marker at time t
            curr = prefix[-1]
            ax1.plot(curr[1], curr[0], marker='o', markersize=12,
                     markerfacecolor=color, markeredgecolor='black', markeredgewidth=1.5)
            # Sensing disk (optional visual)
            circ = patches.Circle((curr[1], curr[0]), radius=Rs, fill=False, ls='--', lw=1.5, edgecolor=color, alpha=0.7)
            ax1.add_patch(circ)
            circ_comm = patches.Circle((curr[1], curr[0]),
                           radius=Rc, fill=False,
                           ls=':', lw=1.5,
                           edgecolor=color, alpha=0.5)
            ax1.add_patch(circ_comm)

            # Start/end flags relative to current t
            start = prefix[0]
            ax1.plot(start[1], start[0], marker='s', color=color, markersize=10,
                     markeredgecolor='black', markeredgewidth=1.3)
            # If full path already finished at/ before t, mark end
            if len(prefix) == len(path):
                ax1.plot(curr[1], curr[0], marker='^', color=color, markersize=10,
                         markeredgecolor='black', markeredgewidth=1.3)

        ax1.legend(loc='upper left', bbox_to_anchor=(0, 1.02), ncols=2, fontsize=9)

        # Plot 2: coverage summary (up to time t)
        ax2.clear()
        ax2.set_title('Coverage Summary', fontsize=14, fontweight='bold')
        coverage_matrix = np.zeros((Nx, Ny))
        covered_nodes = coverage_at_time(t_idx)

        for node in covered_nodes:
            if tuple(node) != tuple(sink):
                r, c = node
                if 1 <= r <= Nx and 1 <= c <= Ny:
                    coverage_matrix[r-1, c-1] = 1  # covered

        # sink
        if 1 <= sink[0] <= Nx and 1 <= sink[1] <= Ny:
            coverage_matrix[sink[0]-1, sink[1]-1] = 2

        cmap = ListedColormap(['white', 'lightgreen', 'yellow'])
        im = ax2.imshow(coverage_matrix, cmap=cmap, vmin=0, vmax=2)
        # labels on each cell
        for i in range(Nx):
            for j in range(Ny):
                val = coverage_matrix[i, j]
                if val == 0:
                    text, color = 'Not\nCovered', 'black'
                elif val == 1:
                    text, color = 'Covered', 'darkgreen'
                else:
                    text, color = 'SINK', 'black'
                ax2.text(j, i, text, ha='center', va='center', fontweight='bold', color=color, fontsize=9)

        ax2.set_xticks(range(Ny))
        ax2.set_yticks(range(Nx))
        ax2.set_xticklabels(range(1, Ny+1))
        ax2.set_yticklabels(range(1, Nx+1))
        ax2.set_xlabel('Column (Y)', fontweight='bold')
        ax2.set_ylabel('Row (X)', fontweight='bold')

        # simple legend hint in the right pad
        ax_legend_pad.clear()
        ax_legend_pad.axis('off')
        ax_legend_pad.text(0.0, 0.95, "Legend", fontweight='bold', fontsize=12)
        ax_legend_pad.text(0.0, 0.88, "● UAV position", color='black')
        ax_legend_pad.text(0.0, 0.82, "◼ Start,  ▲ End", color='black')
        ax_legend_pad.text(0.0, 0.76, "--  Sensing radius", color='black')
        ax_legend_pad.text(0.0, 0.66, "Cell colors:", fontweight='bold')
        ax_legend_pad.text(0.0, 0.60, "  white  : Not covered")
        ax_legend_pad.text(0.0, 0.54, "  lightgreen : Covered")
        ax_legend_pad.text(0.0, 0.48, "  yellow : Sink")

        # coverage stats up to t
        total_nodes = Nx * Ny - 1  # exclude sink
        # do not count sink in covered_nodes
        covered_no_sink = {node for node in covered_nodes if tuple(node) != tuple(sink)}
        covered_count = len(covered_no_sink)
        pct = 100.0 * covered_count / max(1, total_nodes)
        ax_controls.clear()
        ax_controls.axis('off')
        ax_controls.text(0.01, 0.7, f"Time: {t_idx+1}/{T}", fontsize=12, fontweight='bold')
        ax_controls.text(0.01, 0.45, f"Covered (excl. sink): {covered_count}/{total_nodes}  ({pct:.2f}%)", fontsize=11)
        ax_controls.text(0.01, 0.20, f"Sensing radius: {Rs}", fontsize=11)

        fig.canvas.draw_idle()

    # ---------- Interactions ----------
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

    # Initial render
    render_at_t(0)
    plt.show()
