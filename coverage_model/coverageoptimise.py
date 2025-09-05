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
import json


def main(
        sensing_radius:int,
        comm_radius:int,
        col_size:int,
        row_size:int,
        N:int,
        T:int,
        row_sink:int,
        col_sink:int,
        coords_obs:list,
        b_turn:float = 1.0,
        b_mov:float = 2.0,
        b_steady:float = 0.1,
        b_full:float = 100.0,
        initial_battery=None,   # scalar or list of length N
        ebase:float = 0.0,
        solver_time_limit:int = 18000,
        solver_mipgap:float = 0.0
    ) -> dict:
    """
    Main coverage optimization function.
    """

    sz = (row_size, col_size)
    TG = make_grid(row_size, col_size)
    P_sink, sink = pos_sink(row_sink-1, col_sink-1, sz)

    # Obstacles
    obs = pos_obs(coords_obs, sz)
    if obs.size > 0:
        O_lin = np.array([ij_to_i((r-1, c-1), sz) for r, c in obs], dtype=int)
    else:
        O_lin = np.array([], dtype=int)

    if obs.size > 0:
        obs_pairs = set(map(tuple, obs.astype(int)))
        G_index = np.array([tuple(comm_radius) not in obs_pairs for comm_radius in TG])
        G = TG[G_index]
        G1 = np.array([ij_to_i((r-1, c-1), sz) for r, c in G])
        Cmax = row_size * col_size - len(obs) - 1
    else:
        G = TG
        G1 = np.array([ij_to_i((r-1, c-1), sz) for r, c in TG])
        Cmax = row_size * col_size - 1

    print("Target set G1:", G1)

    # Communication & sensing sets
    Irc, Irc_sink = communicable_gpt(P_sink, G-1, sz, comm_radius, O_lin)
    L, Irs, Irs_sink = sensing_gpt(P_sink, G-1, sz, sensing_radius, O_lin)

    # Remove obstacles from neighbor sets
    if O_lin.size > 0:
        Irc = [[q for q in Irc[p] if q not in O_lin] for p in range(len(G))]
        Irs = [[q for q in Irs[p] if q not in O_lin] for p in range(len(G))]
        Irc_sink = [q for q in Irc_sink if q not in O_lin]

    # Objective function (with battery params)
    f = setup_obj(G1, P_sink, O_lin, N, T, row_size, col_size,
                  b_turn, b_mov, b_steady, b_full)

    # ==============================
    # Constraints
    # ==============================
    F, E = poisition_constraints(N, T, G, G1, L, P_sink, row_size, col_size)
    g = sink_connectivity_constraint(N, T, Irc_sink, row_size, col_size)
    H = interUAV_connectivity_constraint(N, T, Irc, row_size, col_size, G, G1, L)
    I = mobility_constraint(N, T, Irc, row_size, col_size, G1, L)
    K, J, Q = cell_coverage_constraints(N, T, row_size, col_size, G1, L, Irs)

    # Inequality constraints
    Aineq, bineq = combine_constraints((E, 1), (g, -1), (H, 0), (I, 0), (J, 0), (Q, 0))

    # Equality constraints
    Aeq, beq = combine_constraints((F, 1), (K, 0))

    # Convert to float (important for CPLEX)
    Aineq = np.asarray(Aineq, dtype=float)
    bineq = np.asarray(bineq, dtype=float)
    Aeq = np.asarray(Aeq, dtype=float)
    beq = np.asarray(beq, dtype=float)

    # Objective vector
    Z1 = -f.astype(float)

    # Bounds
    lb, ub = bounds(row_size, col_size, N, T)
    lb = np.asarray(lb, dtype=float)
    ub = np.asarray(ub, dtype=float)

    no_fly_obstacles(O_lin, row_size, col_size, N, T, ub)

    # Variable types
    ctype = type_vars(row_size, col_size, N, T)

    # ==============================
    # Solve MILP
    # ==============================
    try:
        start_time = time.time()
        Cov_Percent, Cov_time, fval, cov_path, path_loc, S, V = cplex_solver(
            Z1, lb, ub,
            Aineq, bineq,
            Aeq, beq,
            ctype,
            row_size, col_size, N, T, Cmax,
            time_limit=solver_time_limit,
            mipgap=solver_mipgap
        )
        end_time = time.time()
        Cov_time = end_time - start_time

        print(f"Coverage Percentage: {Cov_Percent:.2f}%")
        print(f"Computation Time: {Cov_time:.2f} seconds")
        print(f"Objective Function Value: {fval}")

    except CplexError as e:
        print(f"CPLEX Error: {e}")
        return None

    # ==============================
    # Return results
    # ==============================
    return {
        'S': S,
        'fval': fval,
        'Cov_time': Cov_time,
        'cov_path': cov_path,
        'Cov_Percent': Cov_Percent,
        'path_loc': path_loc,
        'V': V,
        'sz': sz,
        'G': G,
        'O_lin': O_lin,
        'sink': sink,
        'aux_tensor': V  # TODO: replace with real aux tensor for battery vars
    }



if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, required=True, help="Path to JSON config file")
    args = parser.parse_args()
    json_path = args.config
    cfg = load_config(json_path)

    # Mandatory fields
    sensing_radius = cfg["sensing_radius"]
    comm_radius = cfg["comm_radius"]
    col_size = cfg["col_size"]
    row_size = cfg["row_size"]
    N = cfg["N"]
    T = cfg["T"]
    row_sink = cfg["row_sink"]
    col_sink = cfg["col_sink"]
    coords_obs = cfg.get("coords_obs", [])

    # Battery block (defaults if missing)
    battery = cfg.get("battery", {})
    b_turn = battery.get("b_turn", 1.0)
    b_mov = battery.get("b_mov", 2.0)
    b_steady = battery.get("b_steady", 0.1)
    b_full = battery.get("b_full", 100.0)
    initial_battery = battery.get("initial_battery", b_full)  # scalar or list
    ebase = battery.get("ebase", 0.0)

    # Solver options
    solver_cfg = cfg.get("solver", {})
    time_limit = solver_cfg.get("time_limit", 18000)
    mipgap = solver_cfg.get("mipgap", 0)

    # Call main
    results = main(
        sensing_radius=sensing_radius,
        comm_radius=comm_radius,
        col_size=col_size,
        row_size=row_size,
        N=N,
        T=T,
        row_sink=row_sink,
        col_sink=col_sink,
        coords_obs=coords_obs,
        b_turn=b_turn,
        b_mov=b_mov,
        b_steady=b_steady,
        b_full=b_full,
        initial_battery=initial_battery,
        ebase=ebase,
        solver_time_limit=time_limit,
        solver_mipgap=mipgap
    )

    if results is not None:
        # Display results
        uav_paths, uav_covered_nodes, all_covered, battery_levels = display_results(
            results["Cov_Percent"],
            results["Cov_time"],
            results["fval"],
            N,
            row_size,
            col_size,
            T,
            results["cov_path"],
            results["path_loc"],
            results["S"],
            results["sz"],
            results["G"],
            sensing_radius,
            comm_radius,
            results["O_lin"],
            results["sink"],
            results["aux_tensor"]   # placeholder for battery vars
        )

        # Visualization
        plot_interactive_paths(
            results["G"],
            uav_paths,
            uav_covered_nodes,
            results["sink"],
            sensing_radius,
            comm_radius,
            row_size,
            col_size,
            results["O_lin"],
            results["aux_tensor"]
        )

        print("Optimization completed successfully!")
    else:
        print("Optimization failed!")
