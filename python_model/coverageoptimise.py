import numpy as np
import cplex
from cplex.exceptions import CplexError
import time
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.colors import ListedColormap
from matplotlib.widgets import Button
from Gpt import communicable_gpt, sensing_gpt
from pathplotter import plot_interactive_paths
from battery_model import battery_next, compute_battery_for_paths, print_battery_and_motion



def coverage_optimize():
    """
    Main coverage optimization function - Python equivalent of the MATLAB code.
    """
    
    # ==============================
    # User defined input
    # ==============================
    Rs = 1  # Sensing Radius
    Rc = 4 * Rs  # Communication Radius
    
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
    Obs = np.array([(3,3), (5,3)]).reshape(-1, 2)  # Empty obstacles with proper shape
    
    
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
    T = 8  # max no. of time steps
    
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
    battery_traces, position_traces = compute_battery_for_paths(
    uav_paths, sink, T, E0=100.0, battery_fn=battery_next, pad_mode="hold")
    print_battery_and_motion(battery_traces, position_traces)

    
    plot_interactive_paths(G, uav_paths, uav_covered_nodes, sink, Rs, Rc, Nx, Ny, O_lin, battery_traces, position_traces)




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