import numpy as np
import cplex
from cplex.exceptions import CplexError
import time
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.colors import ListedColormap
from Gpt import communicable_gpt, sensing_gpt

# def sensing_gpt(P_sink, G, sz, Rs):
#     """
#     Python equivalent of the MATLAB Sensing_Gpt function.
    
#     Parameters:
#     -----------
#     P_sink : int
#         Linear index of sink node (row-major order).
#     G : ndarray of shape (N,2)
#         Grid coordinates of valid nodes (row, col).
#     sz : tuple (Nx, Ny)
#         Size of the grid.
#     Rs : float
#         Sensing radius.
    
#     Returns:
#     --------
#     L : int
#         Index of sink in G array.
#     Irs : list of lists
#         Each entry contains the indices of sensing neighbors for that node (excluding sink).
#     Irs_sink : list
#         Sensing neighbors of the sink.
#     """
#     # Convert sink index back to (row, col)
#     sink = np.unravel_index(P_sink, sz)  # gives (row, col)
#     sink = np.array(sink)
    
#     # Find sink in G
#     L = np.where((G == sink).all(axis=1))[0][0]
    
#     Irs = []
#     for p in range(len(G)):
#         Gpt = G[p]
#         neighbors = []
#         for q in range(len(G)):
#             v = (Gpt[0] - G[q,0])**2 + (Gpt[1] - G[q,1])**2
#             if v <= Rs**2:
#                 neighbors.append(G[q])
#         neighbors = np.unique(np.array(neighbors), axis=0)
        
#         # Convert coordinates to linear indices
#         indices = [np.ravel_multi_index((r, c), sz) for r, c in neighbors]
        
#         # Exclude sink index
#         indices = sorted(set(indices) - {P_sink})
#         Irs.append(indices)
    
#     Irs_sink = Irs[L]
    
#     return L, Irs, Irs_sink

def plot_coverage_paths(G, uav_paths, uav_covered_nodes, sink, Rs, Nx, Ny):
    """
    Plot the grid, UAV paths, and coverage areas.
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
    
    # Colors for different UAVs
    uav_colors = ['red', 'blue', 'green', 'orange', 'purple']
    
    # Plot 1: Grid with paths and coverage
    ax1.set_title('UAV Paths and Coverage Areas', fontsize=14, fontweight='bold')
    
    # Draw grid
    for i in range(1, Nx+1):
        for j in range(1, Ny+1):
            rect = patches.Rectangle((j-0.5, i-0.5), 1, 1, linewidth=1, 
                                   edgecolor='gray', facecolor='lightgray', alpha=0.3)
            ax1.add_patch(rect)
            ax1.text(j, i, f'({i},{j})', ha='center', va='center', fontsize=8)
    
    # Highlight sink
    sink_rect = patches.Rectangle((sink[1]-0.5, sink[0]-0.5), 1, 1, 
                                 linewidth=3, edgecolor='black', facecolor='yellow', alpha=0.8)
    ax1.add_patch(sink_rect)
    ax1.text(sink[1], sink[0], 'SINK', ha='center', va='center', fontweight='bold')
    
    # Plot coverage areas for each UAV
    for uav_id, covered_nodes in uav_covered_nodes.items():
        if covered_nodes:
            color = uav_colors[uav_id % len(uav_colors)]
            for node in covered_nodes:
                if tuple(node) != tuple(sink):  # Don't cover the sink
                    coverage_rect = patches.Rectangle((node[1]-0.5, node[0]-0.5), 1, 1,
                                                    linewidth=2, edgecolor=color, 
                                                    facecolor=color, alpha=0.3)
                    ax1.add_patch(coverage_rect)
    
    # Plot UAV paths
    for uav_id, path in uav_paths.items():
        if path:
            color = uav_colors[uav_id % len(uav_colors)]
            path_y = [pos[0] for pos in path]
            path_x = [pos[1] for pos in path]
            
            # Plot path line
            ax1.plot(path_x, path_y, color=color, linewidth=3, marker='o', 
                    markersize=8, label=f'UAV {uav_id+1}', alpha=0.8)
            
            # Add arrows to show direction
            for i in range(len(path_x)-1):
                ax1.annotate('', xy=(path_x[i+1], path_y[i+1]), xytext=(path_x[i], path_y[i]),
                           arrowprops=dict(arrowstyle='->', color=color, lw=2))
            
            # Mark start and end points
            ax1.plot(path_x[0], path_y[0], marker='s', color=color, markersize=12, 
                    markeredgecolor='black', markeredgewidth=2, label=f'UAV {uav_id+1} Start')
            ax1.plot(path_x[-1], path_y[-1], marker='^', color=color, markersize=12, 
                    markeredgecolor='black', markeredgewidth=2, label=f'UAV {uav_id+1} End')
    
    ax1.set_xlim(0.5, Ny+0.5)
    ax1.set_ylim(0.5, Nx+0.5)
    ax1.set_xlabel('Column (Y)', fontweight='bold')
    ax1.set_ylabel('Row (X)', fontweight='bold')
    ax1.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect('equal')
    
    # Plot 2: Coverage summary
    ax2.set_title('Coverage Summary', fontsize=14, fontweight='bold')
    
    # Create coverage matrix
    coverage_matrix = np.zeros((Nx, Ny))
    
    # Mark covered nodes
    all_covered = set()
    for uav_id, covered_nodes in uav_covered_nodes.items():
        all_covered.update([tuple(node) for node in covered_nodes])
    
    for node in all_covered:
        if tuple(node) != tuple(sink):
            coverage_matrix[node[0]-1, node[1]-1] = 1  # Convert to 0-based indexing
    
    # Mark sink
    coverage_matrix[sink[0]-1, sink[1]-1] = 2
    
    # Create custom colormap
    colors = ['white', 'lightgreen', 'yellow']  # uncovered, covered, sink
    cmap = ListedColormap(colors)
    
    im = ax2.imshow(coverage_matrix, cmap=cmap, vmin=0, vmax=2)
    
    # Add grid and labels
    for i in range(Nx):
        for j in range(Ny):
            if coverage_matrix[i, j] == 0:
                text = 'Not\nCovered'
                color = 'black'
            elif coverage_matrix[i, j] == 1:
                text = 'Covered'
                color = 'darkgreen'
            else:
                text = 'SINK'
                color = 'black'
            
            ax2.text(j, i, text, ha='center', va='center', 
                    fontweight='bold', color=color, fontsize=9)
    
    ax2.set_xticks(range(Ny))
    ax2.set_yticks(range(Nx))
    ax2.set_xticklabels(range(1, Ny+1))
    ax2.set_yticklabels(range(1, Nx+1))
    ax2.set_xlabel('Column (Y)', fontweight='bold')
    ax2.set_ylabel('Row (X)', fontweight='bold')
    
    # Add colorbar
    cbar = plt.colorbar(im, ax=ax2, shrink=0.8)
    cbar.set_ticks([0, 1, 2])
    cbar.set_ticklabels(['Not Covered', 'Covered', 'Sink'])
    
    plt.tight_layout()
    plt.show()
    
    # Print detailed coverage statistics
    total_nodes = Nx * Ny - 1  # Exclude sink
    covered_count = len(all_covered)
    coverage_percentage = (covered_count / total_nodes) * 100
    
    print(f"\n" + "="*50)
    print("COVERAGE STATISTICS")
    print("="*50)
    print(f"Total nodes (excluding sink): {total_nodes}")
    print(f"Nodes covered: {covered_count}")
    print(f"Coverage percentage: {coverage_percentage:.2f}%")
    print(f"Sensing radius: {Rs}")
    
    # Per-UAV statistics
    print(f"\nPer-UAV Coverage:")
    for uav_id in range(len(uav_paths)):
        if uav_paths[uav_id]:
            print(f"  UAV {uav_id+1}: {len(uav_covered_nodes[uav_id])} nodes")
        else:
            print(f"  UAV {uav_id+1}: 0 nodes (no path)")


def coverage_optimize():
    """
    Main coverage optimization function - Python equivalent of the MATLAB code.
    """
    
    # ==============================
    # User defined input
    # ==============================
    Rs = 1  # Sensing Radius
    Rc = 2 * Rs  # Communication Radius
    
    # ==============================
    # Network Grid
    # ==============================
    x = np.arange(1, 7)  # 1:6
    y = np.arange(1, 7)  # 1:6
    b = x
    a = y
    
    # Create meshgrid (equivalent to MATLAB meshgrid)
    A, B = np.meshgrid(a, b)
    Nx = len(x)
    Ny = len(y)
    
    # Create TG matrix (equivalent to [Y(:),X(:)]) - 1-based coordinates
    TG = np.column_stack((A.flatten(), B.flatten()))
    sz = (Nx, Ny)
    
    P_sink = 0  # Index position of sink (0-based in Python)
    row, col = np.unravel_index(P_sink, sz)
    sink = np.array([row + 1, col + 1])  # Convert to 1-based for consistency with TG
    
    # Obstacles (empty in this case)
    Obs = np.array([(2,3),(3,4)]).reshape(-1, 2)  # Empty obstacles with proper shape
    
    
    if(Obs.size > 0):
        O_lin = np.array([np.ravel_multi_index((r-1, c-1), sz) for r, c in Obs], dtype=int)
    else:
        O_lin = np.array([], dtype=int) 

    if Obs.size > 0:
        obs_pairs = set(map(tuple, Obs.astype(int)))
        G_index = np.array([tuple(rc) not in obs_pairs for rc in TG])
        G = TG[G_index]
        # Convert 1-based coordinates to 0-based linear indices
        G1 = np.array([np.ravel_multi_index((r-1, c-1), sz) for r, c in G])
        Cmax = Nx * Ny - len(Obs) - 1
    else:
        G = TG
        # Convert 1-based coordinates to 0-based linear indices
        G1 = np.array([np.ravel_multi_index((r-1, c-1), sz) for r, c in TG])
        Cmax = Nx * Ny - 1
    
    print("Target set G1:", G1)
    
    # ==============================
    # Communicable and Sensing grid points sets
    # ==============================
    # Convert sink back to 0-based for the functions
    P_sink_0based = np.ravel_multi_index((sink[0]-1, sink[1]-1), sz)
    Irc, Irc_sink = communicable_gpt(P_sink_0based, G-1, sz, Rc, O_lin)  # communication matrix (G-1 for 0-based)
    L, Irs, Irs_sink = sensing_gpt(P_sink_0based, G-1, sz, Rs, O_lin)  # sensing matrix (G-1 for 0-based)

    # Filtering out the obstacles from the neighbor sets
    if O_lin.size > 0:
        Irc = [[q for q in Irc[p] if q not in O_lin] for p in range(len(G))]
        Irs = [[q for q in Irs[p] if q not in O_lin] for p in range(len(G))]
        Irc_sink = [q for q in Irc_sink if q not in O_lin]
        # If the center p itself is an obstacle (it won't be, since G excludes obstacles),
        # you could also empty those lists. Left here for completeness:
        # for p in range(len(G)):
        #     if G1[p] in set(O_lin):
        #         Irc[p] = []
        #         Irs[p] = []
        

    
    N = 2  # number of UAVs
    T = 6  # max no. of time steps
    
    # ==============================
    # Objective function
    # ==============================
    f = np.zeros((1 + 2*N*T) * Nx * Ny)
    f[G1] = 1
    f[P_sink] = 0
    
    #set the reward function for the obstacle to zero
    if O_lin.size > 0:
        f[O_lin] = 0
        
        
    
    # ==============================
    # Position Constraints
    # ==============================
    # 1(a) one-UAV-at-single-point
    F = np.zeros((N*T, (1 + 2*N*T)*Nx*Ny))
    k = 0
    for t in range(T):
        for n in range(N):
            f1 = np.zeros((1 + 2*N*T)*Nx*Ny)
            f1[G1 + (1 + T*N)*Nx*Ny + (t*N*Nx*Ny) + (n*Nx*Ny)] = 1
            f1[(1 + T*N)*Nx*Ny + (t*N*Nx*Ny) + (n*Nx*Ny) + P_sink] = 0
            F[k, :] = f1
            k += 1
    
    # 1(b) single-UAV-at-one-pt
    E = np.zeros((T*len(G), (1 + 2*N*T)*Nx*Ny))
    k = 0
    for t in range(T):
        for p in range(len(G)):
            f2 = np.zeros((1 + 2*N*T)*Nx*Ny)
            if p != L:
                for n in range(N):
                    f2[(1 + T*N)*Nx*Ny + (t*N*Nx*Ny) + (n*Nx*Ny) + G1[p]] = 1
            E[k, :] = f2
            k += 1
    
    # ==============================
    # Connectivity Constraint
    # ==============================
    # 2a Sink
    g = np.zeros((T, (1 + 2*N*T)*Nx*Ny))
    k = 0
    for t in range(T):
        g1 = np.zeros((1 + 2*N*T)*Nx*Ny)
        for n in range(N):
            for i in range(len(Irc_sink)):
                g1[Nx*Ny + (N*T*Nx*Ny) + (t*N*Nx*Ny) + (n*Nx*Ny) + Irc_sink[i]] = -1
        g[k, :] = g1
        k += 1
    
    # 2b Inter-UAV
    H = np.zeros((T*(N-1)*len(G), (1 + 2*N*T)*Nx*Ny))
    k = 0
    for t in range(T):
        for n in range(1, N):  # n=2:N in MATLAB becomes n=1:N-1 in Python
            for p in range(len(G)):
                h11 = np.zeros((1 + 2*N*T)*Nx*Ny)
                if p != L:
                    h11[Nx*Ny + T*N*Nx*Ny + (t*N*Nx*Ny) + (n*Nx*Ny) + G1[p]] = 1
                    for i in range(len(Irc[p])):
                        h11[Nx*Ny + (N*T*Nx*Ny) + (t*N*Nx*Ny) + ((n-1)*Nx*Ny) + Irc[p][i]] = -1
                H[k, :] = h11
                k += 1
    
    # ==============================
    # Mobility Constraint
    # ==============================
    I = np.zeros(((T-1)*N*len(G), (1 + 2*N*T)*Nx*Ny))
    k = 0
    for t in range(T-1):
        for n in range(N):
            for p in range(len(G)):
                h12 = np.zeros((1 + 2*N*T)*Nx*Ny)
                if p != L:
                    h12[Nx*Ny + (N*T*Nx*Ny) + ((t+1)*N*Nx*Ny) + (n*Nx*Ny) + G1[p]] = 1
                    for i in range(len(Irc[p])):
                        h12[Nx*Ny + (N*T*Nx*Ny) + (t*N*Nx*Ny) + (n*Nx*Ny) + Irc[p][i]] = -1
                I[k, :] = h12
                k += 1
    
    # ==============================
    # Cell coverage constraint variables
    # ==============================
    # 4(a)
    K = np.zeros((T*N*len(G), (1 + 2*N*T)*Nx*Ny))
    k = 0
    for t in range(T):
        for n in range(N):
            for p in range(len(G)):
                h1 = np.zeros((1 + 2*N*T)*Nx*Ny)
                if p != L:
                    h1[Nx*Ny + (t*N*Nx*Ny) + (n*Nx*Ny) + G1[p]] = 1
                    for i in range(len(Irs[p])):
                        h1[Nx*Ny + (N*T*Nx*Ny) + (t*N*Nx*Ny) + (n*Nx*Ny) + Irs[p][i]] = -1
                K[k, :] = h1
                k += 1
    
    # 4(b)
    J = np.zeros((N*T*len(G), (1 + 2*N*T)*Nx*Ny))
    k = 0
    for t in range(T):
        for n in range(N):
            for q in range(len(G)):
                h2 = np.zeros((1 + 2*N*T)*Nx*Ny)
                h2[G1[q]] = -1
                h2[G1[L]] = 0
                h2[Nx*Ny + (t*N*Nx*Ny) + (n*Nx*Ny) + G1[q]] = 1
                h2[Nx*Ny + (t*N*Nx*Ny) + (n*Nx*Ny) + G1[L]] = 0
                J[k, :] = h2
                k += 1
    
    # 4(c)
    Q = np.zeros((len(G), (1 + 2*N*T)*Nx*Ny))
    k = 0
    for q in range(len(G)):
        h3 = np.zeros((1 + 2*N*T)*Nx*Ny)
        h3[G1[q]] = 1
        h3[G1[L]] = 0
        for t in range(T):
            for n in range(N):
                h3[Nx*Ny + (t*N*Nx*Ny) + (n*Nx*Ny) + G1[q]] = -1
                h3[Nx*Ny + (t*N*Nx*Ny) + (n*Nx*Ny) + G1[L]] = 0
        Q[k, :] = h3
        k += 1
    
    # ==============================
    # Optimization problem setup
    # ==============================
    Z1 = -f  # Objective function
    
    # Inequality constraints
    Aineq = np.vstack([E, g, H, I, J, Q])
    bineq = np.hstack([
        np.ones(E.shape[0]),
        -np.ones(g.shape[0]),
        np.zeros(H.shape[0]),
        np.zeros(I.shape[0]),
        np.zeros(J.shape[0]),
        np.zeros(Q.shape[0])
    ])
    
    # Equality constraints
    Aeq = np.vstack([F, K])
    beq = np.hstack([np.ones(F.shape[0]), np.zeros(K.shape[0])])
    
    # Bounds
    lb = np.zeros((Nx*Ny) + 2*N*T*Nx*Ny)
    ub = np.ones((Nx*Ny) + 2*N*T*Nx*Ny)
    

    # -------- NO-FLY OBSTACLE BOUNDS --------
    # Helper indexers for the 3 blocks in your variable vector
    def idx_y(q):                      # Block A (selection), size Nx*Ny
        return q

    def idx_a(t, n, q):                # Block B (coverage assign), size N*T*Nx*Ny
        # offset_A = Nx*Ny
        return Nx*Ny + (t*N*Nx*Ny) + (n*Nx*Ny) + q

    def idx_x(t, n, q):                # Block C (positions), size N*T*Nx*Ny (binary)
        # offset_A = Nx*Ny; offset_B = N*T*Nx*Ny
        return Nx*Ny + (N*T*Nx*Ny) + (t*N*Nx*Ny) + (n*Nx*Ny) + q

    if O_lin.size > 0:
        for q in O_lin:
            # Never select obstacle cells
            ub[idx_y(q)] = 0.0
            # Never assign coverage to obstacle cells
            for t in range(T):
                for n in range(N):
                    ub[idx_a(t, n, q)] = 0.0
                    # Never occupy obstacle cells (NO-FLY)
                    ub[idx_x(t, n, q)] = 0.0

    # -------- END NO-FLY OBSTACLE BOUNDS --------





    # Variable types (continuous and binary)
    ctype = ['C'] * ((1 + N*T) * Nx * Ny) + ['B'] * (N*T * Nx * Ny)
    
    # ==============================
    # Solve MILP using CPLEX
    # ==============================
    prob = cplex.Cplex()
    prob.objective.set_sense(prob.objective.sense.minimize)
    prob.variables.add(obj=Z1.tolist(), lb=lb.tolist(), ub=ub.tolist(), types=ctype)
    
    # Add inequality constraints
    for i in range(Aineq.shape[0]):
        prob.linear_constraints.add(
            lin_expr=[cplex.SparsePair(ind=list(range(len(Aineq[i]))), val=Aineq[i].tolist())],
            senses=["L"],
            rhs=[bineq[i]]
        )
    
    # Add equality constraints
    for i in range(Aeq.shape[0]):
        prob.linear_constraints.add(
            lin_expr=[cplex.SparsePair(ind=list(range(len(Aeq[i]))), val=Aeq[i].tolist())],
            senses=["E"],
            rhs=[beq[i]]
        )
    
    # Solver options
    prob.parameters.mip.tolerances.mipgap.set(0)
    prob.parameters.timelimit.set(18000)  # 5 hours
    prob.parameters.mip.strategy.search.set(1)
    
    # Solve
    start_time = time.time()
    try:
        prob.solve()
    except CplexError as e:
        print(f"CPLEX Error: {e}")
        return None
    
    Cov_time = time.time() - start_time
    
    # ==============================
    # Extract solution
    # ==============================
    S = np.array(prob.solution.get_values())
    fval = prob.solution.get_objective_value()
    
    # ==============================
    # Result calculations
    # ==============================
    ss = np.round(S, 1)
    c = np.where(ss > 0)[0]
    path_loc = c[c > (1 + T*N) * Nx * Ny]
    U = ss[path_loc]
    V = np.column_stack((path_loc, U))
    
    # Route tracing
    cov_path = np.zeros((N, Nx*Ny*T), dtype=int)
    for n in range(N):
        P1 = []
        for t in range(T):
            for i in range(len(path_loc)):
                Rmin = (1 + T*N)*Nx*Ny + t*N*Nx*Ny + n*Nx*Ny
                Rmax = (1 + T*N)*Nx*Ny + Nx*Ny + t*N*Nx*Ny + n*Nx*Ny
                if (Rmax <= (1 + T*N + T*N)*Nx*Ny and 
                    path_loc[i] > Rmin and path_loc[i] <= Rmax):
                    c1 = path_loc[i]
                    c2 = c1 - ((1 + T*N)*Nx*Ny + t*N*Nx*Ny + n*Nx*Ny)
                    P1.append(c2)
        
        if len(P1) > 0:
            cov_path[n, :len(P1)] = P1
    
    Cov_Percent = (-1) * (fval / Cmax) * 100
    
    # ==============================
    # Display results
    # ==============================
    print(f"Coverage Percentage: {Cov_Percent:.2f}%")
    print(f"Optimization Time: {Cov_time:.2f} seconds")
    print(f"Objective Value: {fval:.4f}")
    print("\n" + "="*50)
    print("UAV PATHS AND COVERAGE ANALYSIS")
    print("="*50)
    
    # Extract and display paths for each UAV
    uav_paths = {}
    uav_covered_nodes = {}
    
    for n in range(N):
        non_zero = cov_path[n, cov_path[n] != 0]
        if len(non_zero) > 0:
            # Convert linear indices to 2D coordinates
            path_coords = []
            for idx in non_zero:
                row, col = np.unravel_index(idx, sz)
                path_coords.append((row+1, col+1))  # Convert to 1-based for display
            
            uav_paths[n] = path_coords
            
            # Find nodes covered by this UAV at each time step
            covered_nodes = set()
            for t in range(T):
                if t < len(path_coords):
                    current_pos = path_coords[t]
                    # Find all nodes within sensing radius
                    for node in G:
                        dist = np.sqrt((current_pos[0] - node[0])**2 + (current_pos[1] - node[1])**2)
                        if dist <= Rs:
                            covered_nodes.add(tuple(node))
            
            uav_covered_nodes[n] = list(covered_nodes)
            
            print(f"\nUAV {n+1}:")
            print(f"  Path: {path_coords}")
            print(f"  Nodes covered: {sorted(uav_covered_nodes[n])}")
            print(f"  Total nodes covered: {len(uav_covered_nodes[n])}")
        else:
            print(f"\nUAV {n+1}: No path assigned")
            uav_paths[n] = []
            uav_covered_nodes[n] = []
    
    # Calculate total unique coverage
    all_covered = set()
    for nodes in uav_covered_nodes.values():
        all_covered.update(nodes)
    
    print(f"\nTotal unique nodes covered: {len(all_covered)}")
    print(f"All covered nodes: {sorted(list(all_covered))}")
    
    # ==============================
    # Visualization
    # ==============================
    plot_coverage_paths(G, uav_paths, uav_covered_nodes, sink, Rs, Nx, Ny)
    
    return {
        'S': S,
        'fval': fval,
        'Cov_time': Cov_time,
        'cov_path': cov_path,
        'Cov_Percent': Cov_Percent,
        'path_loc': path_loc,
        'V': V,
        'uav_paths': uav_paths,
        'uav_covered_nodes': uav_covered_nodes
    }

if __name__ == "__main__":
    result = coverage_optimize()
    if result is not None:
        print("Optimization completed successfully!")
    else:
        print("Optimization failed!")
        
        