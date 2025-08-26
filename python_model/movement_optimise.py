import numpy as np
import time
import cplex
from cplex.exceptions import CplexError
import matplotlib.pyplot as plt
import matplotlib.patches as patches


def communicable_gpt(P_sink, G, sz, Rc):
    """
    Python equivalent of the MATLAB Communicable_Gpt function.
    """
    # Convert sink index back to (row, col) - MATLAB uses 1-based indexing
    sink = np.unravel_index(P_sink-1, sz)  # Convert to 0-based for unravel_index
    sink = np.array([sink[0]+1, sink[1]+1])  # Convert back to 1-based

    Irc = []
    for p in range(len(G)):
        Gpt = G[p]
        neighbors = []
        for q in range(len(G)):
            v = (Gpt[0] - G[q,0])**2 + (Gpt[1] - G[q,1])**2
            if v <= Rc**2:
                neighbors.append(G[q])
        neighbors = np.unique(np.array(neighbors), axis=0)

        # Convert coordinates to linear indices (1-based MATLAB style)
        indices = []
        for r, c in neighbors:
            idx = np.ravel_multi_index((r-1, c-1), sz) + 1  # Convert to 1-based
            indices.append(idx)

        # Exclude sink index
        indices = sorted(set(indices) - {P_sink})
        Irc.append(indices)

    # Find sink in G
    L = None
    for i, node in enumerate(G):
        if np.array_equal(node, sink):
            L = i + 1  # 1-based index
            break
    
    if L is None:
        raise ValueError("Sink not found in G")
    
    Irc_sink = Irc[L-1]  # Convert back to 0-based for list access

    return Irc, Irc_sink

def sensing_gpt(P_sink, G, sz, Rs):
    """
    Python equivalent of the MATLAB Sensing_Gpt function.
    """
    # Convert sink index back to (row, col) - MATLAB uses 1-based indexing
    sink = np.unravel_index(P_sink-1, sz)  # Convert to 0-based for unravel_index
    sink = np.array([sink[0]+1, sink[1]+1])  # Convert back to 1-based

    # Find sink in G
    L = None
    for i, node in enumerate(G):
        if np.array_equal(node, sink):
            L = i + 1  # 1-based index
            break
    
    if L is None:
        raise ValueError("Sink not found in G")

    Irs = []
    for p in range(len(G)):
        Gpt = G[p]
        neighbors = []
        for q in range(len(G)):
            v = (Gpt[0] - G[q, 0])**2 + (Gpt[1] - G[q, 1])**2
            if v <= Rs**2:
                neighbors.append(G[q])
        neighbors = np.unique(np.array(neighbors), axis=0)

        # Convert coordinates to linear indices (1-based MATLAB style)
        indices = []
        for r, c in neighbors:
            idx = np.ravel_multi_index((r-1, c-1), sz) + 1  # Convert to 1-based
            indices.append(idx)

        # Exclude sink index
        indices = sorted(set(indices) - {P_sink})
        Irs.append(indices)

    Irs_sink = Irs[L-1]  # Convert back to 0-based for list access

    return L, Irs, Irs_sink

def plot_movement_paths(G, cov_path, sink, P_sink, Nx, Ny, path_cost):
    """
    Plot the grid and UAV movement paths exactly like MATLAB results.
    """
    fig, ax = plt.subplots(1, 1, figsize=(12, 10))
    
    # Colors for different UAVs
    uav_colors = ['red', 'blue', 'green', 'orange', 'purple']
    
    ax.set_title(f'UAV Movement Paths (Total Cost: {path_cost:.2f})', 
                 fontsize=14, fontweight='bold')
    
    # Draw grid
    for i in range(1, Nx+1):
        for j in range(1, Ny+1):
            rect = patches.Rectangle((j-0.5, i-0.5), 1, 1, linewidth=1, 
                                   edgecolor='gray', facecolor='lightgray', alpha=0.3)
            ax.add_patch(rect)
            # Calculate linear index for display (1-based like MATLAB)
            linear_idx = np.ravel_multi_index((i-1, j-1), (Nx, Ny)) + 1
            ax.text(j, i, f'{linear_idx}', ha='center', va='center', fontsize=8)
    
    # Highlight sink
    sink_row, sink_col = np.unravel_index(P_sink-1, (Nx, Ny))
    sink_row += 1  # Convert to 1-based
    sink_col += 1
    
    sink_rect = patches.Rectangle((sink_col-0.5, sink_row-0.5), 1, 1, 
                                 linewidth=3, edgecolor='black', facecolor='yellow', alpha=0.8)
    ax.add_patch(sink_rect)
    ax.text(sink_col, sink_row, f'SINK\n{P_sink}', ha='center', va='center', fontweight='bold')
    
    # Plot UAV paths
    N = cov_path.shape[0]
    for n in range(N):
        path_indices = cov_path[n, cov_path[n] != 0]  # Get non-zero indices
        
        if len(path_indices) > 0:
            color = uav_colors[n % len(uav_colors)]
            
            # Convert linear indices to coordinates
            path_coords = []
            for idx in path_indices:
                row, col = np.unravel_index(idx-1, (Nx, Ny))  # Convert to 0-based
                path_coords.append((row+1, col+1))  # Convert back to 1-based
            
            if len(path_coords) > 0:
                path_y = [pos[0] for pos in path_coords]
                path_x = [pos[1] for pos in path_coords]
                
                # Plot path line
                ax.plot(path_x, path_y, color=color, linewidth=3, marker='o', 
                       markersize=10, label=f'UAV {n+1}', alpha=0.8)
                
                # Add arrows to show direction
                for i in range(len(path_x)-1):
                    ax.annotate('', xy=(path_x[i+1], path_y[i+1]), xytext=(path_x[i], path_y[i]),
                               arrowprops=dict(arrowstyle='->', color=color, lw=3))
                
                # Mark start and end points
                ax.plot(path_x[0], path_y[0], marker='s', color=color, markersize=15, 
                       markeredgecolor='black', markeredgewidth=2)
                ax.plot(path_x[-1], path_y[-1], marker='^', color=color, markersize=15, 
                       markeredgecolor='black', markeredgewidth=2)
                
                # Add step numbers
                for i, (x, y) in enumerate(zip(path_x, path_y)):
                    ax.text(x+0.2, y+0.2, f't{i+1}', fontsize=10, fontweight='bold', 
                           color=color, bbox=dict(boxstyle='round,pad=0.2', 
                           facecolor='white', alpha=0.8))
    
    ax.set_xlim(0.5, Ny+0.5)
    ax.set_ylim(0.5, Nx+0.5)
    ax.set_xlabel('Column (Y)', fontweight='bold')
    ax.set_ylabel('Row (X)', fontweight='bold')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    
    plt.tight_layout()
    plt.show()

def movement_optimize():
    """
    EXACT Python equivalent of the MATLAB movement optimization code.
    """
    # ==============================
    # User defined input - EXACT MATLAB VALUES
    # ==============================
    Rs = np.sqrt(2)  # Sensing Radius
    Rc = 2 * Rs      # Communication Radius
    cr = 1
    
    # ==============================
    # Network Grid - EXACT MATLAB SETUP
    # ==============================
    x = np.arange(1, 10)  # 1:12
    y = np.arange(1, 10)  # 1:12
    b = x
    a = y
    
    # MATLAB meshgrid equivalent
    X, Y = np.meshgrid(a, b)
    Nx = len(x)
    Ny = len(y)
    
    # TG = [Y(:), X(:)] - MATLAB column stacking
    TG = np.column_stack((Y.flatten(), X.flatten()))
    sz = np.array([Nx, Ny])
    
    P_sink = 79  # EXACT MATLAB VALUE - Index position of sink
    
    # Convert to row, col (MATLAB ind2sub equivalent)
    row, col = np.unravel_index(P_sink-1, sz)  # Convert to 0-based for Python
    sink = np.array([row+1, col+1])  # Convert back to 1-based
    
    Obs = np.array([])  # Empty obstacles
    
    # Process obstacles (same logic as MATLAB)
    if Obs.size > 0:
        # Remove obstacles logic would go here
        pass
    else:
        G = TG
        # G1 = sub2ind(sz, TG(:,1), TG(:,2)) - MATLAB equivalent
        G1 = np.array([np.ravel_multi_index((r-1, c-1), sz) + 1 for r, c in TG])  # 1-based indexing
        Cmax = Nx * Ny - 1
    
    print(f"Target set G1 shape: {G1.shape}")
    print(f"P_sink: {P_sink}")
    print(f"Sink coordinates: {sink}")
    print(f"Grid size: {Nx}x{Ny}")
    
    # ==============================
    # Communicable and Sensing grid points sets
    # ==============================
    Irc, Irc_sink = communicable_gpt(P_sink, G, sz, Rc)
    L, Irs, Irs_sink = sensing_gpt(P_sink, G, sz, Rs)
    
    N = 3   # number of UAVs
    T = 10  # max no. of time steps
    
    print(f"L (sink index in G): {L}")
    print(f"Number of UAVs: {N}")
    print(f"Time steps: {T}")
    
    # ==============================
    # Objective function - EXACT MATLAB CODE
    # ==============================
    ff = np.zeros((1 + 2*N*T) * Nx * Ny)
    
    for t in range(1, T+1):
        for n in range(1, N+1):
            # MATLAB: ff(G1+(1 + T*N)*Nx*Ny + ((t-1)*N*Nx*Ny)+(n-1)*Nx*Ny) = 1
            indices = (G1-1) + (1 + T*N)*Nx*Ny + ((t-1)*N*Nx*Ny) + (n-1)*Nx*Ny  # Convert G1 to 0-based
            ff[indices] = 1
            # MATLAB: ff((1 + T*N)*Nx*Ny + ((t-1)*N*Nx*Ny)+(n-1)*Nx*Ny + P_sink) = 0
            ff[(1 + T*N)*Nx*Ny + ((t-1)*N*Nx*Ny) + (n-1)*Nx*Ny + P_sink-1] = 0  # Convert P_sink to 0-based
    
    # ==============================
    # Position Constraints - EXACT MATLAB CODE
    # ==============================
    # 1(a) one-UAV-at-single-pt
    F = np.zeros((N*T, (1 + 2*N*T)*Nx*Ny))
    k = 0
    for t in range(1, T+1):
        for n in range(1, N+1):
            f1 = np.zeros((1 + 2*N*T)*Nx*Ny)
            indices = (G1-1) + (1 + T*N)*Nx*Ny + ((t-1)*N*Nx*Ny) + (n-1)*Nx*Ny  # Convert to 0-based
            f1[indices] = 1
            f1[(1 + T*N)*Nx*Ny + ((t-1)*N*Nx*Ny) + (n-1)*Nx*Ny + P_sink-1] = 0  # Convert to 0-based
            F[k, :] = f1
            k += 1
    
    # 1(b) single-UAV-at-one-pt
    E = np.zeros((T*len(G), (1 + 2*N*T)*Nx*Ny))
    k = 0
    for t in range(1, T+1):
        for p in range(1, len(G)+1):
            f2 = np.zeros((1 + 2*N*T)*Nx*Ny)
            if p != L:
                for n in range(1, N+1):
                    f2[(1 + T*N)*Nx*Ny + ((t-1)*N*Nx*Ny) + (n-1)*Nx*Ny + G1[p-1]-1] = 1  # Convert to 0-based
            E[k, :] = f2
            k += 1
    
    # ==============================
    # Connectivity Constraint - EXACT MATLAB CODE
    # ==============================
    # 2a Sink
    g = np.zeros((T, (1 + 2*N*T)*Nx*Ny))
    k = 0
    for t in range(1, T+1):
        g1 = np.zeros((1 + 2*N*T)*Nx*Ny)
        for n in range(1, N+1):
            for i in range(len(Irc_sink)):
                g1[(1 + T*N)*Nx*Ny + ((t-1)*N*Nx*Ny) + (n-1)*Nx*Ny + Irc_sink[i]-1] = -1  # Convert to 0-based
        g[k, :] = g1
        k += 1
    
    # 2b Inter-UAV
    H = np.zeros((T*(N-1)*len(G), (1 + 2*N*T)*Nx*Ny))
    k = 0
    for t in range(1, T+1):
        for n in range(2, N+1):
            for p in range(1, len(G)+1):
                h11 = np.zeros((1 + 2*N*T)*Nx*Ny)
                if p != L:
                    h11[Nx*Ny + T*N*Nx*Ny + ((t-1)*N*Nx*Ny) + (n-1)*Nx*Ny + G1[p-1]-1] = 1  # Convert to 0-based
                    for i in range(len(Irc[p-1])):
                        h11[Nx*Ny + (N*T*Nx*Ny) + ((t-1)*N*Nx*Ny) + (n-2)*Nx*Ny + Irc[p-1][i]-1] = -1  # Convert to 0-based
                H[k, :] = h11
                k += 1
    
    # ==============================
    # Mobility Constraint - EXACT MATLAB CODE
    # ==============================
    I = np.zeros(((T-1)*N*len(G), (1 + 2*N*T)*Nx*Ny))
    k = 0
    for t in range(1, T):  # 1:(T-1)
        for n in range(1, N+1):
            for p in range(1, len(G)+1):
                h12 = np.zeros((1 + 2*N*T)*Nx*Ny)
                if p != L:
                    h12[Nx*Ny + (N*T*Nx*Ny) + (t*N*Nx*Ny) + (n-1)*Nx*Ny + G1[p-1]-1] = 1  # Convert to 0-based
                    for i in range(len(Irc[p-1])):
                        h12[Nx*Ny + (N*T*Nx*Ny) + ((t-1)*N*Nx*Ny) + (n-1)*Nx*Ny + Irc[p-1][i]-1] = -1  # Convert to 0-based
                I[k, :] = h12
                k += 1
    
    # ==============================
    # Cell coverage constraint variables - EXACT MATLAB CODE
    # ==============================
    # 4(a)
    K = np.zeros((T*N*len(G), (1 + 2*N*T)*Nx*Ny))
    k = 0
    for t in range(1, T+1):
        for n in range(1, N+1):
            for p in range(1, len(G)+1):
                h1 = np.zeros((1 + 2*N*T)*Nx*Ny)
                if p != L:
                    h1[Nx*Ny + ((t-1)*N*Nx*Ny) + (n-1)*Nx*Ny + G1[p-1]-1] = 1  # Convert to 0-based
                    for i in range(len(Irs[p-1])):
                        h1[Nx*Ny + (N*T*Nx*Ny) + ((t-1)*N*Nx*Ny) + (n-1)*Nx*Ny + Irs[p-1][i]-1] = -1  # Convert to 0-based
                K[k, :] = h1
                k += 1
    
    # 4(b)
    J = np.zeros((N*T*len(G), (1 + 2*N*T)*Nx*Ny))
    k = 0
    for t in range(1, T+1):
        for n in range(1, N+1):
            for q in range(1, len(G)+1):
                h2 = np.zeros((1 + 2*N*T)*Nx*Ny)
                h2[G1[q-1]-1] = -1  # Convert to 0-based
                h2[G1[L-1]-1] = 0   # Convert to 0-based
                h2[Nx*Ny + ((t-1)*N*Nx*Ny) + (n-1)*Nx*Ny + G1[q-1]-1] = 1  # Convert to 0-based
                h2[Nx*Ny + ((t-1)*N*Nx*Ny) + (n-1)*Nx*Ny + G1[L-1]-1] = 0  # Convert to 0-based
                J[k, :] = h2
                k += 1
    
    # 4(c)
    Q = np.zeros((len(G), (1 + 2*N*T)*Nx*Ny))
    k = 0
    for q in range(1, len(G)+1):
        h3 = np.zeros((1 + 2*N*T)*Nx*Ny)
        h3[G1[q-1]-1] = 1  # Convert to 0-based
        h3[G1[L-1]-1] = 0  # Convert to 0-based
        for t in range(1, T+1):
            for n in range(1, N+1):
                h3[Nx*Ny + ((t-1)*N*Nx*Ny) + (n-1)*Nx*Ny + G1[q-1]-1] = -1  # Convert to 0-based
                h3[Nx*Ny + ((t-1)*N*Nx*Ny) + (n-1)*Nx*Ny + G1[L-1]-1] = 0  # Convert to 0-based
        Q[k, :] = h3
        k += 1
    
    # ==============================
    # Desired Coverage Constraint - EXACT MATLAB CODE
    # ==============================
    W = np.zeros((1 + 2*N*T)*Nx*Ny)
    for l in range(len(G)):
        W[G1[l]-1] = -1  # Convert to 0-based
    W[G1[L-1]-1] = 0  # Convert to 0-based
    
    # ==============================
    # Optimization problem - EXACT MATLAB SETUP
    # ==============================
    Z1 = ff  # objective function
    
    # MATLAB: A = [E;F;g;H;I;J;Q;W']
    A = np.vstack([E, F, g, H, I, J, Q, W.reshape(1, -1)])
    
    # MATLAB: b = [ones(size(E,1),1); ones(size(F,1),1); (-1)*ones(size(g,1),1); 
    #              zeros(size(H,1),1); zeros(size(I,1),1); zeros(size(J,1),1); 
    #              zeros(size(Q,1),1); (-1)*cr*Cmax]
    b = np.hstack([
        np.ones(E.shape[0]),
        np.ones(F.shape[0]),
        (-1) * np.ones(g.shape[0]),
        np.zeros(H.shape[0]),
        np.zeros(I.shape[0]),
        np.zeros(J.shape[0]),
        np.zeros(Q.shape[0]),
        np.array([(-1) * cr * Cmax])
    ])
    
    Aineq = A
    bineq = b
    
    # MATLAB: Aeq = [K]
    Aeq = K
    # MATLAB: beq = [zeros(size(K,1),1)]
    beq = np.zeros(K.shape[0])
    
    # ==============================
    # Bounds - EXACT MATLAB SETUP
    # ==============================
    lb1 = np.zeros(Nx*Ny)
    lb2 = np.zeros(N*T*Nx*Ny)
    lb3 = np.zeros(N*T*Nx*Ny)
    lb = np.hstack([lb1, lb2, lb3])
    
    ub1 = np.ones(Nx*Ny)
    ub2 = np.ones(N*T*Nx*Ny)
    ub3 = np.ones(N*T*Nx*Ny)
    ub = np.hstack([ub1, ub2, ub3])
    
    # Variable types - EXACT MATLAB LOGIC
    ctype = []
    # First (1+T*N)*Nx*Ny variables are continuous
    for p in range((1+T*N)*Nx*Ny):
        ctype.append('C')
    # Last T*N*Nx*Ny variables are binary
    for p in range(T*N*Nx*Ny):
        ctype.append('B')
    
    print(f"Problem size: {len(Z1)} variables")
    print(f"Inequality constraints: {Aineq.shape[0]}")
    print(f"Equality constraints: {Aeq.shape[0]}")
    
    # ==============================
    # Solve with CPLEX - EXACT MATLAB OPTIONS
    # ==============================
    try:
        cpx = cplex.Cplex()
        cpx.objective.set_sense(cpx.objective.sense.minimize)
        
        # MATLAB options equivalent
        cpx.parameters.mip.tolerances.mipgap.set(0)  # mipgap = 0
        cpx.parameters.timelimit.set(18000)  # 18000 seconds
        cpx.parameters.mip.strategy.search.set(1)  # strategy search = 1
        
        # Add variables
        cpx.variables.add(
            obj=Z1.tolist(),
            lb=lb.tolist(),
            ub=ub.tolist(),
            types=ctype
        )
        
        # Add inequality constraints
        print("Adding inequality constraints...")
        for row in range(Aineq.shape[0]):
            nonzero_idx = np.nonzero(Aineq[row, :])[0]
            if len(nonzero_idx) > 0:
                coeffs = Aineq[row, nonzero_idx].tolist()
                vars_idx = nonzero_idx.tolist()
                cpx.linear_constraints.add(
                    lin_expr=[cplex.SparsePair(vars_idx, coeffs)],
                    senses=['L'],
                    rhs=[bineq[row]]
                )
        
        # Add equality constraints
        print("Adding equality constraints...")
        for row in range(Aeq.shape[0]):
            nonzero_idx = np.nonzero(Aeq[row, :])[0]
            if len(nonzero_idx) > 0:
                coeffs = Aeq[row, nonzero_idx].tolist()
                vars_idx = nonzero_idx.tolist()
                cpx.linear_constraints.add(
                    lin_expr=[cplex.SparsePair(vars_idx, coeffs)],
                    senses=['E'],
                    rhs=[beq[row]]
                )
        
        print("Starting optimization...")
        start_time = time.time()
        cpx.solve()
        solve_time = time.time() - start_time
        
        status = cpx.solution.get_status()
        print(f"CPLEX status: {status}")
        
        if status in [101, 102, 107]:  # Optimal or feasible
            S = np.array(cpx.solution.get_values())
            fval = cpx.solution.get_objective_value()
            
            print(f"Objective value: {fval:.4f}")
            print(f"Solve time: {solve_time:.2f} seconds")
            
            # ==============================
            # Result calculations - EXACT MATLAB CODE
            # ==============================
            ss = np.round(S, 1)
            c = np.where(ss > 0)[0] + 1  # Convert to 1-based indexing like MATLAB
            path_loc = c[c > (1 + T*N)*Nx*Ny]
            U = ss[path_loc-1]  # Convert back to 0-based for array access
            V = np.column_stack((path_loc, U))
            
            print(f"Active path variables: {len(path_loc)}")
            
            # ==============================
            # Route Tracing - EXACT MATLAB CODE
            # ==============================
            max_path_length = T  # Maximum possible path length
            cov_path = np.zeros((N, max_path_length), dtype=int)
            
            for n in range(1, N+1):  # MATLAB: n=1:N
                P1 = []
                for t in range(1, T+1):  # MATLAB: t=1:T
                    for i in range(len(path_loc)):
                        Rmin = (1 + T*N)*Nx*Ny + (t-1)*N*Nx*Ny + (n-1)*Nx*Ny
                        Rmax = (1 + T*N)*Nx*Ny + Nx*Ny + (t-1)*N*Nx*Ny + (n-1)*Nx*Ny
                        
                        if (Rmax <= (1 + T*N + T*N)*Nx*Ny and 
                            path_loc[i] > Rmin and path_loc[i] <= Rmax):
                            c1 = path_loc[i]
                            c2 = c1 - ((1 + T*N)*Nx*Ny + (t-1)*N*Nx*Ny + (n-1)*Nx*Ny)
                            P1.append(c2)
                
                if P1:
                    # Fill only up to the length of P1 or max_path_length
                    fill_length = min(len(P1), max_path_length)
                    cov_path[n-1, :fill_length] = P1[:fill_length]
            
            print("Coverage paths (cov_path):")
            print(cov_path)
            
            # ==============================
            # Cost calculation - EXACT MATLAB CODE (COMPLETED)
            # ==============================
            cost = 0
            for r in range(N):  # MATLAB: r=1:N
                p1 = cov_path[r, :]
                p2 = np.where(p1 != 0)[0]  # MATLAB: find(p1)
                ind = p1[p2]
                
                if len(ind) == 1:
                    continue  # MATLAB: break
                elif len(ind) > 1:
                    # MATLAB: [row, col] = ind2sub(sz, ind)
                    rows = []
                    cols = []
                    for idx in ind:
                        row, col = np.unravel_index(idx-1, sz)  # Convert to 0-based
                        rows.append(row+1)  # Convert back to 1-based
                        cols.append(col+1)
                    
                    rows = np.array(rows)
                    cols = np.array(cols)
                    
                    # Calculate path cost
                    for k in range(len(ind)-1):
                        cost += np.sqrt((rows[k+1] - rows[k])**2 + (cols[k+1] - cols[k])**2)
            
            print(f"Total path cost: {cost:.4f}")
            
            # ==============================
            # Visualization - Plot results
            # ==============================
            plot_movement_paths(G, cov_path, sink, P_sink, Nx, Ny, cost)
            
            # ==============================
            # Return results
            # ==============================
            return {
                'objective_value': fval,
                'solve_time': solve_time,
                'total_cost': cost,
                'coverage_paths': cov_path,
                'solution_vector': S,
                'grid_size': (Nx, Ny),
                'sink_position': P_sink,
                'num_uavs': N,
                'time_steps': T
            }
        
        else:
            print(f"Optimization failed with status: {status}")
            return None
    
    except CplexError as exc:
        print(f"CPLEX Error: {exc}")
        return None
    except Exception as e:
        print(f"Error during optimization: {e}")
        return None

def main():
    """
    Main function to run the UAV movement optimization.
    """
    print("Starting UAV Movement Optimization...")
    print("="*50)
    
    # Check if CPLEX is available
    try:
        import cplex
        print("CPLEX is available")
    except ImportError:
        print("ERROR: CPLEX is not installed. Please install IBM CPLEX Optimization Studio.")
        return
    
    # Run the optimization
    results = movement_optimize()
    
    if results is not None:
        print("="*50)
        print("OPTIMIZATION COMPLETED SUCCESSFULLY!")
        print("="*50)
        print(f"Objective Value: {results['objective_value']:.4f}")
        print(f"Total Path Cost: {results['total_cost']:.4f}")
        print(f"Solve Time: {results['solve_time']:.2f} seconds")
        print(f"Grid Size: {results['grid_size'][0]}x{results['grid_size'][1]}")
        print(f"Number of UAVs: {results['num_uavs']}")
        print(f"Time Steps: {results['time_steps']}")
        print(f"Sink Position: {results['sink_position']}")
        print("\nCoverage Paths:")
        for i, path in enumerate(results['coverage_paths']):
            non_zero_path = path[path != 0]
            if len(non_zero_path) > 0:
                print(f"UAV {i+1}: {non_zero_path}")
            else:
                print(f"UAV {i+1}: No movement")
    else:
        print("="*50)
        print("OPTIMIZATION FAILED!")
        print("="*50)

if __name__ == "__main__":
    main()