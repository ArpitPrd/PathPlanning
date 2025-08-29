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
import argparse


def sz_to_XY(row_size, col_size):
    """
    general purpose meshgrid maker

    need a, b to be arrays
    """
    x = np.arange(1, row_size+1)  # 1:6
    y = np.arange(1, col_size+1)  # 1:6
    
    return np.meshgrid(y, x)

def i_to_ij(index, sz):
    """
    convert lin length to row, col
    """
    return np.unravel_index(index, sz)

def ij_to_i(cod, sz):
    """
    convert coordinates to lin length wrt sz
    """
    # TODO
    return np.ravel_multi_index(cod, sz)

def XY_to_xy(A:np.array, B:np.array) -> np.array:
    """
    A, B must be obtained in the format from np.meshgrid to obtain all the coordis
    """
    return np.column_stack((A.flatten(), B.flatten()))

def make_grid(row_size, col_size):
    """
    prepares complete meshgrid and returns the gird points

    different from mesh grid because it returns XY
    """
    
    A, B = sz_to_XY(row_size, col_size)
    return XY_to_xy(A, B)

def pos_sink(row_sink, col_sink, sz):
    """
    prepares the sink wrt to the grid
    """
    P_sink = ij_to_i((row_sink, col_sink), sz)
    sink = np.array([row_sink + 1, col_sink + 1])
    return P_sink, sink

def pos_obs(coords_obs, sz=None):
    """
    positions the obstructions in the grid wrt to the size
    """
    return np.array(coords_obs).reshape(-1, len(coords_obs[0]))

def coverage_optimize(
        sensing_radius:int, 
        comm_radius:int,
        col_size:int,
        row_size:int,
        N:int,
        T:int,
        row_sink:int,
        col_sink:int,
        coords_obs:list
    ) -> dict:
    """
    Main coverage optimization function - Python equivalent of the MATLAB code.
    """
    

    sz = (row_size, col_size)

    TG = make_grid(row_size, col_size)

    P_sink, sink = pos_sink(row_sink-1, col_sink-1, sz)
    
    # Obstacles (empty in this case)
    obs = pos_obs(coords_obs, sz)
    
    if(obs.size > 0):
        O_lin = np.array([np.ravel_multi_index((r-1, c-1), sz) for r, c in obs], dtype=int)
    else:
        O_lin = np.array([], dtype=int) 

    if obs.size > 0:
        obs_pairs = set(map(tuple, obs.astype(int)))
        G_index = np.array([tuple(comm_radius) not in obs_pairs for comm_radius in TG])
        G = TG[G_index]
        # Convert 1-based coordinates to 0-based linear indices
        G1 = np.array([np.ravel_multi_index((r-1, c-1), sz) for r, c in G])
        Cmax = row_size * col_size - len(obs) - 1
    else:
        G = TG
        # Convert 1-based coordinates to 0-based linear indices
        G1 = np.array([np.ravel_multi_index((r-1, c-1), sz) for r, c in TG])
        Cmax = row_size * col_size - 1
    
    print("Target set G1:", G1)
    
    # ==============================
    # Communicable and Sensing grid points sets
    # ==============================
    # Convert sink back to 0-based for the functions
    P_sink_0based = np.ravel_multi_index((sink[0]-1, sink[1]-1), sz) # need to change this
    Irc, Irc_sink = communicable_gpt(P_sink_0based, G-1, sz, comm_radius, O_lin)  # communication matrix (G-1 for 0-based)
    L, Irs, Irs_sink = sensing_gpt(P_sink_0based, G-1, sz, sensing_radius, O_lin)  # sensing matrix (G-1 for 0-based)

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
    
    # ==============================
    # Objective function
    # ==============================
    f = np.zeros((1 + 2*N*T) * row_size * col_size)
    f[G1] = 1
    f[P_sink] = 0
    
    #set the reward function for the obstacle to zero
    if O_lin.size > 0:
        f[O_lin] = 0
        
        
    
    # ==============================
    # Position Constraints
    # ==============================
    # 1(a) one-UAV-at-single-point
    F = np.zeros((N*T, (1 + 2*N*T)*row_size*col_size))
    k = 0
    for t in range(T):
        for n in range(N):
            f1 = np.zeros((1 + 2*N*T)*row_size*col_size)
            f1[G1 + (1 + T*N)*row_size*col_size + (t*N*row_size*col_size) + (n*row_size*col_size)] = 1
            f1[(1 + T*N)*row_size*col_size + (t*N*row_size*col_size) + (n*row_size*col_size) + P_sink] = 0
            F[k, :] = f1
            k += 1
    
    # 1(b) single-UAV-at-one-pt
    E = np.zeros((T*len(G), (1 + 2*N*T)*row_size*col_size))
    k = 0
    for t in range(T):
        for p in range(len(G)):
            f2 = np.zeros((1 + 2*N*T)*row_size*col_size)
            if p != L:
                for n in range(N):
                    f2[(1 + T*N)*row_size*col_size + (t*N*row_size*col_size) + (n*row_size*col_size) + G1[p]] = 1
            E[k, :] = f2
            k += 1
    
    # ==============================
    # Connectivity Constraint
    # ==============================
    # 2a Sink
    g = np.zeros((T, (1 + 2*N*T)*row_size*col_size))
    k = 0
    for t in range(T):
        g1 = np.zeros((1 + 2*N*T)*row_size*col_size)
        for n in range(N):
            for i in range(len(Irc_sink)):
                g1[row_size*col_size + (N*T*row_size*col_size) + (t*N*row_size*col_size) + (n*row_size*col_size) + Irc_sink[i]] = -1
        g[k, :] = g1
        k += 1
    
    # 2b Inter-UAV
    H = np.zeros((T*(N-1)*len(G), (1 + 2*N*T)*row_size*col_size))
    k = 0
    for t in range(T):
        for n in range(1, N):  # n=2:N in MATLAB becomes n=1:N-1 in Python
            for p in range(len(G)):
                h11 = np.zeros((1 + 2*N*T)*row_size*col_size)
                if p != L:
                    h11[row_size*col_size + T*N*row_size*col_size + (t*N*row_size*col_size) + (n*row_size*col_size) + G1[p]] = 1
                    for i in range(len(Irc[p])):
                        h11[row_size*col_size + (N*T*row_size*col_size) + (t*N*row_size*col_size) + ((n-1)*row_size*col_size) + Irc[p][i]] = -1
                H[k, :] = h11
                k += 1
    
    # ==============================
    # Mobility Constraint
    # ==============================
    I = np.zeros(((T-1)*N*len(G), (1 + 2*N*T)*row_size*col_size))
    k = 0
    for t in range(T-1):
        for n in range(N):
            for p in range(len(G)):
                h12 = np.zeros((1 + 2*N*T)*row_size*col_size)
                if p != L:
                    h12[row_size*col_size + (N*T*row_size*col_size) + ((t+1)*N*row_size*col_size) + (n*row_size*col_size) + G1[p]] = 1
                    for i in range(len(Irc[p])):
                        h12[row_size*col_size + (N*T*row_size*col_size) + (t*N*row_size*col_size) + (n*row_size*col_size) + Irc[p][i]] = -1
                I[k, :] = h12
                k += 1
    
    # ==============================
    # Cell coverage constraint variables
    # ==============================
    # 4(a)
    K = np.zeros((T*N*len(G), (1 + 2*N*T)*row_size*col_size))
    k = 0
    for t in range(T):
        for n in range(N):
            for p in range(len(G)):
                h1 = np.zeros((1 + 2*N*T)*row_size*col_size)
                if p != L:
                    h1[row_size*col_size + (t*N*row_size*col_size) + (n*row_size*col_size) + G1[p]] = 1
                    for i in range(len(Irs[p])):
                        h1[row_size*col_size + (N*T*row_size*col_size) + (t*N*row_size*col_size) + (n*row_size*col_size) + Irs[p][i]] = -1
                K[k, :] = h1
                k += 1
    
    # 4(b)
    J = np.zeros((N*T*len(G), (1 + 2*N*T)*row_size*col_size))
    k = 0
    for t in range(T):
        for n in range(N):
            for q in range(len(G)):
                h2 = np.zeros((1 + 2*N*T)*row_size*col_size)
                h2[G1[q]] = -1
                h2[G1[L]] = 0
                h2[row_size*col_size + (t*N*row_size*col_size) + (n*row_size*col_size) + G1[q]] = 1
                h2[row_size*col_size + (t*N*row_size*col_size) + (n*row_size*col_size) + G1[L]] = 0
                J[k, :] = h2
                k += 1
    
    # 4(c)
    Q = np.zeros((len(G), (1 + 2*N*T)*row_size*col_size))
    k = 0
    for q in range(len(G)):
        h3 = np.zeros((1 + 2*N*T)*row_size*col_size)
        h3[G1[q]] = 1
        h3[G1[L]] = 0
        for t in range(T):
            for n in range(N):
                h3[row_size*col_size + (t*N*row_size*col_size) + (n*row_size*col_size) + G1[q]] = -1
                h3[row_size*col_size + (t*N*row_size*col_size) + (n*row_size*col_size) + G1[L]] = 0
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
    lb = np.zeros((row_size*col_size) + 2*N*T*row_size*col_size)
    ub = np.ones((row_size*col_size) + 2*N*T*row_size*col_size)
    

    # -------- NO-FLY OBSTACLE BOUNDS --------
    # Helper indexers for the 3 blocks in your variable vector
    def idx_y(q):                      # Block A (selection), size row_size*col_size
        return q

    def idx_a(t, n, q):                # Block B (coverage assign), size N*T*row_size*col_size
        # offset_A = row_size*col_size
        return row_size*col_size + (t*N*row_size*col_size) + (n*row_size*col_size) + q

    def idx_x(t, n, q):                # Block C (positions), size N*T*row_size*col_size (binary)
        # offset_A = row_size*col_size; offset_B = N*T*row_size*col_size
        return row_size*col_size + (N*T*row_size*col_size) + (t*N*row_size*col_size) + (n*row_size*col_size) + q

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
    ctype = ['C'] * ((1 + N*T) * row_size * col_size) + ['B'] * (N*T * row_size * col_size)
    
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
    path_loc = c[c > (1 + T*N) * row_size * col_size]
    U = ss[path_loc]
    V = np.column_stack((path_loc, U))
    
    # Route tracing
    cov_path = np.zeros((N, row_size*col_size*T), dtype=int)
    for n in range(N):
        P1 = []
        for t in range(T):
            for i in range(len(path_loc)):
                Rmin = (1 + T*N)*row_size*col_size + t*N*row_size*col_size + n*row_size*col_size
                Rmax = (1 + T*N)*row_size*col_size + row_size*col_size + t*N*row_size*col_size + n*row_size*col_size
                if (Rmax <= (1 + T*N + T*N)*row_size*col_size and 
                    path_loc[i] > Rmin and path_loc[i] <= Rmax):
                    c1 = path_loc[i]
                    c2 = c1 - ((1 + T*N)*row_size*col_size + t*N*row_size*col_size + n*row_size*col_size)
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
                        if dist <= sensing_radius:
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
    plot_interactive_paths(G, uav_paths, uav_covered_nodes, sink, sensing_radius, comm_radius, row_size, col_size, O_lin)

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

    return 1

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Path Planning Args")
    
    parser.add_argument("--sensing-radius", type=int, help="sensing radius for every drone")
    parser.add_argument("--comm-radius", type=int, help="communication radius for every drone")
    parser.add_argument("--col-size", type=int, help="size of the col in grid area")
    parser.add_argument("--row-size", type=int, help="size of the row in grid")
    parser.add_argument("--N", type=int, help="number of drones")
    parser.add_argument("--T", type=int, help="Time limit for the optimisations")
    parser.add_argument("--row-sink", type=int, help="row location of sink")
    parser.add_argument("--col-sink", type=int, help="col location of sink")

    args = parser.parse_args()
    sensing_radius = args.sensing_radius
    comm_radius=args.comm_radius
    col_size = args.col_size
    row_size = args.row_size
    N = args.N
    T = args.T
    row_sink = args.row_sink
    col_sink = args.col_sink
    coords_obs = [
        (3, 3),
        (5, 4)
    ]
    result = coverage_optimize(
        sensing_radius=sensing_radius, 
        comm_radius=comm_radius, 
        col_size=col_size, 
        row_size=row_size, 
        N=N, 
        T=T, 
        row_sink=row_sink, 
        col_sink=col_sink,
        coords_obs=coords_obs
    )

    if result is not None:
        print("Optimization completed successfully!")
    else:
        print("Optimization failed!")    