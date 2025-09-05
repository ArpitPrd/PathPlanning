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


def main(
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
    # 1(a) one-UAV-at-single-point, equality constraint
    F, E = poisition_constraints(N, T, G, G1, L, P_sink, row_size, col_size)
    
    # 2a Sink, inequality constraint
    g = sink_connectivity_constraint(N, T, Irc_sink, row_size, col_size)
    
    # 2b Inter-UAV, inequality constraint
    H = interUAV_connectivity_constraint(N, T, Irc, row_size, col_size, G, G1, L)
    
    # 3 Mobility, inequality constraint
    I = mobility_constraint(N, T, Irc, row_size, col_size, G1, L)
    
    
    # Cell coverage constraint variables, K equality constraint, J and Q inequality constraints
    K, J, Q = cell_coverage_constraints(N, T, row_size, col_size, G1, L, Irs)
    
    # Inequality constraints
    Aineq, bineq = combine_constraints((E, 1), (g, -1), (H, 0), (I, 0), (J, 0), (Q, 0))
    
    # Equality constraints
    Aeq, beq = combine_constraints((F, 1), (K, 0))
    
    # Optimization problem setup
    Z1 = -f  # Objective function
    
    # Bounds
    lb, ub = bounds(row_size, col_size, N, T)
    
    no_fly_obstacles(O_lin, row_size, col_size, N, T, lb, ub)

    # Variable types (continuous and binary)
    ctype = type_vars(row_size, col_size, N, T)
    
    # Solve the MILP using CPLEX
    try:
        start_time = time.time()
        Cov_Percent, Cov_time, fval, cov_path, path_loc, S, V = cplex_solver(Z1, lb, ub, Aineq, bineq, Aeq, beq, ctype, row_size, col_size, N, T, Cmax)
        end_time = time.time()
        Cov_time = end_time - start_time
        print(f"Coverage Percentage: {Cov_Percent:.2f}%")
        print(f"Computation Time: {Cov_time:.2f} seconds")
        print(f"Objective Function Value: {fval}")
    except CplexError as e:
        print(e)
        return None
    


    return {
        'S': S,
        'fval': fval,
        'Cov_time': Cov_time,
        'cov_path': cov_path,
        'Cov_Percent': Cov_Percent,
        'path_loc': path_loc,
        'V': V,
        'uav_paths': uav_paths,
        'uav_covered_nodes': uav_covered_nodes,
        'sz': sz,
        'G': G,
        'O_lin': O_lin,
        'sink': sink,
        'aux_tensor': V  # <--- NEW
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
    results = main(
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

        # Display results
    uav_paths, uav_covered_nodes, all_covered, battery_levels = display_results(
        results["Cov_Percent"],
        results["Solve_time"],
        results["Objective_value"],
        N,
        row_size,
        col_size,
        T,
        results["cov_path"],
        results["active_indices"],
        results["solution_vector"],
        results["sz"],
        results["G"],
        sensing_radius,
        comm_radius,
        results["O_lin"],
        results["sink"],
        results["aux_tensor"]   # <--- NEW
    )
    G = results["G"]
    O_lin = results["O_lin"]
    sink = results["sink"]
    # Visualization
    plot_interactive_paths(G, uav_paths, uav_covered_nodes, sink, sensing_radius, comm_radius, row_size, col_size, O_lin)
    if results is not None:
        print("Optimization completed successfully!")
    else:
        print("Optimization failed!")    