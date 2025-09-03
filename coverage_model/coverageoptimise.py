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
from coverage_utils import *


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
        O_lin = np.array([ij_to_i((r-1, c-1), sz) for r, c in obs], dtype=int)
    else:
        O_lin = np.array([], dtype=int) 

    if obs.size > 0:
        obs_pairs = set(map(tuple, obs.astype(int)))
        G_index = np.array([tuple(comm_radius) not in obs_pairs for comm_radius in TG])
        G = TG[G_index]
        # Convert 1-based coordinates to 0-based linear indices
        G1 = np.array([ij_to_i((r-1, c-1), sz) for r, c in G])
        Cmax = row_size * col_size - len(obs) - 1
    else:
        G = TG
        # Convert 1-based coordinates to 0-based linear indices
        G1 = np.array([ij_to_i((r-1, c-1), sz) for r, c in TG])
        Cmax = row_size * col_size - 1
    
    print("Target set G1:", G1)
    
    # Communicable grid points sets
    Irc, Irc_sink = communicable_gpt(P_sink, G-1, sz, comm_radius, O_lin) 
    
    # Sensing grid points sets
    L, Irs, Irs_sink = sensing_gpt(P_sink, G-1, sz, sensing_radius, O_lin) 

    # Filtering out the obstacles from the neighbor sets
    if O_lin.size > 0:
        Irc = [[q for q in Irc[p] if q not in O_lin] for p in range(len(G))]
        Irs = [[q for q in Irs[p] if q not in O_lin] for p in range(len(G))]
        Irc_sink = [q for q in Irc_sink if q not in O_lin]

    # Objective function
    f = setup_obj(G1, P_sink, O_lin, N, T, row_size, col_size)
        
        
    
    # ==============================
    # Position Constraints
    # ==============================
    # 1(a) one-UAV-at-single-point
    F, E = poisition_constraints(N, T, G, G1, L, P_sink, row_size, col_size)
    
    # 2a Sink
    g = sink_connectivity_constraint(N, T, Irc_sink, row_size, col_size)
    
    # 2b Inter-UAV
    H = interUAV_connectivity_constraint(N, T, Irc, row_size, col_size, G, G1, L)
    
    # 3 Mobility
    I = mobility_constraint(N, T, Irc, row_size, col_size, G1, L)
    
    
    # Cell coverage constraint variables
    K, J, Q = cell_coverage_constraints(N, T, row_size, col_size, G1, L, Irs)
    
    
    # Optimization problem setup
    Z1 = -f  # Objective function
    
    # Inequality constraints
    Aineq, bineq = inequality_constraints(E, g, H, I, J, Q)
    
    # Equality constraints
    Aeq, beq = equality_constraints(F, K)
    
    # Bounds
    lb, ub = bounds(row_size, col_size, N, T)
    
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
    
    # Extract solution
    S = np.array(prob.solution.get_values())
    fval = prob.solution.get_objective_value()
    
    # Result calculations
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
    
    # Display results
    uav_paths, uav_covered_nodes, all_covered = display_results(
        Cov_Percent,
        Cov_time,
        fval,
        N,
        row_size,
        col_size,
        T,
        cov_path,
        path_loc,
        S,
        sz,
        G,
        sensing_radius,
        comm_radius,
        O_lin,
        sink
    )
    

    # Visualization
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