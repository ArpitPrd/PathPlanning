import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib.colors import ListedColormap
from matplotlib.widgets import Button
from battery_model import E_MAX, E_STEADY, E_MOVEMENT, E_TURN, battery_next











def plot_interactive_paths(G, uav_paths, uav_covered_nodes, sink, Rs, Rc,Nx, Ny, O_lin = None, battery_traces = None, position_traces=None):
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
    
    def draw_battery_panel(t_idx):
        """
        Draw a right-hand legend plus a per-UAV battery panel with adaptive spacing.
        Expects:
        - battery_traces[uav_id] = np.array of length T+1 (E[0] at t=0)
        - uav_paths dict keys are the UAV ids to display
        """
        ax_legend_pad.clear()
        ax_legend_pad.axis('off')

        # --- Static legend (top) ---
        ax_legend_pad.text(0.0, 0.95, "Legend", fontweight='bold', fontsize=12)
        ax_legend_pad.text(0.0, 0.88, "● UAV position", color='black')
        ax_legend_pad.text(0.0, 0.82, "◼ Start,  ▲ End", color='black')
        ax_legend_pad.text(0.0, 0.76, "--  Sensing radius", color='black')
        ax_legend_pad.text(0.0, 0.66, "Cell colors:", fontweight='bold')
        ax_legend_pad.text(0.0, 0.60, "  white  : Not covered")
        ax_legend_pad.text(0.0, 0.54, "  lightgreen : Covered")
        ax_legend_pad.text(0.0, 0.48, "  yellow : Sink")

        # --- Battery legend header + constants ---
        ax_legend_pad.text(0.0, 0.38, "Battery", fontweight='bold', fontsize=12)
        ax_legend_pad.text(0.0, 0.33, f"Steady: -{E_STEADY:.1f}%", fontsize=9)
        ax_legend_pad.text(0.0, 0.28, f"Move:   -{E_MOVEMENT:.1f}%", fontsize=9)
        ax_legend_pad.text(0.0, 0.23, f"Turn:   -{E_TURN:.1f}%", fontsize=9)

        # If no traces, show hint and bail
        if not battery_traces:
            ax_legend_pad.text(0.0, 0.17, "(no battery_traces)", fontsize=9, color='gray')
            return

        # --- Adaptive layout for bars (avoids overlap) ---
        uav_ids = sorted([k for k in uav_paths.keys() if k in battery_traces])
        n_uav = len(uav_ids)
        if n_uav == 0:
            ax_legend_pad.text(0.0, 0.17, "(no UAVs to display)", fontsize=9, color='gray')
            return

        # Reserve vertical space for bars only (below constants block)
        y_top, y_bottom = 0.20, 0.02             # normalized axes coords
        span = max(1e-6, y_top - y_bottom)
        dy = span / n_uav                        # row height per UAV
        bar_h = min(0.035, 0.6 * dy)             # bar thickness
        bar_w = 0.95                              # full track width in axes coords
        label_fs = 10

        for i, uav_id in enumerate(uav_ids):
            E = battery_traces[uav_id]
            # UI shows step t_idx (0..T-1) -> take E at t_idx+1 (cap at end)
            e_now = float(E[min(t_idx + 1, len(E) - 1)])

            # Row center and bar bottom
            y_c = y_top - (i + 0.5) * dy
            y0 = y_c - bar_h / 2

            # Label slightly above the bar
            ax_legend_pad.text(0.0, y_c + bar_h * 0.9,
                            f"UAV {uav_id+1}: {e_now:4.1f}%",
                            fontsize=label_fs, va='bottom')

            # Background track
            ax_legend_pad.add_patch(
                patches.Rectangle((0.0, y0), bar_w, bar_h,
                                transform=ax_legend_pad.transAxes,
                                facecolor='#dddddd', edgecolor='black', lw=0.6)
            )
            # Filled portion (green)
            w = max(0.0, min(bar_w * (e_now / 100.0), bar_w))
            ax_legend_pad.add_patch(
                patches.Rectangle((0.0, y0), w, bar_h,
                                transform=ax_legend_pad.transAxes,
                                facecolor='green', edgecolor='none', alpha=0.85)
            )

    

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
        draw_battery_panel(t_idx)


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
