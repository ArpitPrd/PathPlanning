import numpy as np
import argparse
from coverage_utils import (
    load_config, VarHelper, setup_objective_and_sense, pos_sink, pos_obs, ij_to_i, i_to_ij,
    position_and_collision_constraints, connectivity_constraints, movement_and_mobility_constraints,
    cell_coverage_constraints, battery_constraints, model_specific_constraints, combine_constraints, cplex_solver
)
from Gpt_1 import communicable_gpt, sensing_gpt
from pathplotter_1 import plot_interactive_paths, animate_paths
from memory import get_total_size
import sys

import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib.widgets import Button
from matplotlib import animation
from feasability_checks import quick_feasibility_checks, inspect_Irs_Irc, check_variable_bounds_and_types, compute_max_coverage_relax, check_bigM_consistency


def process_results(solution, vh, Irs, sz):
    uav_paths = {n: [] for n in range(vh.N)}
    uav_covered_nodes = {n: [] for n in range(vh.N)}

    for n in range(vh.N):
        for t in range(vh.T):
            pos_found = False
            for i in range(vh.num_grid_cells):
                if solution[vh.z(t, n, i)] > 0.99:
                    uav_paths[n].append(i)
                    sensed_by_i = Irs[i] if i < len(Irs) else []
                    uav_covered_nodes[n].append(sensed_by_i + [i])
                    pos_found = True
                    break
            if not pos_found:
                uav_paths[n].append(None); uav_covered_nodes[n].append([])

    covered_cells_idx = {i for i in range(vh.num_grid_cells) if solution[vh.c_i(i)] > 0.99}
    return uav_paths, covered_cells_idx, uav_covered_nodes

def display_results(vh, uav_paths, covered_cells_idx, battery_levels):
    print("\n" + "="*50 + "\nOPTIMIZATION RESULTS\n" + "="*50)
    for n in range(vh.N):
        print(f"\n--- UAV {n+1} ---")
        path_coords = [i_to_ij(pos, (vh.rs, vh.cs)) if pos is not None else 'N/A' for pos in uav_paths[n]]
        print(f"  Path (row, col): {path_coords}")
        if battery_levels: # Only print if battery levels exist
            print(f"  Battery Levels: {[round(b, 2) for b in battery_levels[n]]}")

    coverage_percent = len(covered_cells_idx) / vh.num_grid_cells * 100
    print(f"\nTotal Cells Covered: {len(covered_cells_idx)} / {vh.num_grid_cells} ({coverage_percent:.2f}%)")


def main(cfg: dict):
    # ==============================
    # 1. SETUP GRID AND PARAMETERS
    # ==============================
    print("1. Setting up grid and parameters...")
    sensing_radius, comm_radius = cfg["sensing_radius"], cfg["comm_radius"]
    col_size, row_size = cfg["col_size"], cfg["row_size"]
    N, T = cfg["N"], cfg["T"]
    row_sink, col_sink = cfg["row_sink"], cfg["col_sink"]
    coords_obs = cfg.get("coords_obs", [])
    
    battery = cfg.get("battery", {}); b_mov, b_steady = battery.get("b_mov", 2.0), battery.get("b_steady", 0.1)
    b_full, e_base = battery.get("b_full", 100.0), battery.get("ebase", 10.0)
    initial_battery = battery.get("initial_battery", b_full)
    
    solver_cfg = cfg.get("solver", {}); time_limit, mipgap = solver_cfg.get("time_limit", 6000), solver_cfg.get("mipgap", 0.01)
    
    # --- Read Model and Constraint Toggles from Config ---
    model_cfg = cfg.get("model", {"name": "maximize_coverage"}) # Defaults to model 1
    constraints_cfg = cfg.get("constraints", {})
    print(f"Using optimization model: {model_cfg.get('name')}")
    print("Constraints config:", constraints_cfg)

    sz = (row_size, col_size); P_sink, _ = pos_sink(row_sink - 1, col_sink - 1, sz)
    obs = pos_obs(coords_obs); O_lin = {ij_to_i((r - 1, c - 1), sz) for r, c in obs} if obs.size > 0 else set()
    
    # ==============================
    # 2. INITIALIZE VARHELPER & OBJECTIVE
    # ==============================
    print("2. Initializing variable helper and objective function...")
    vh = VarHelper(N, T, row_size, col_size)
    objective, objective_sense = setup_objective_and_sense(vh, model_cfg, b_mov, b_steady)

    # ==============================
    # 3. COMPUTE NEIGHBORHOOD SETS
    # ==============================
    print("3. Computing neighborhood sets (Sensing/Communication)...")
    all_coords = np.array([i_to_ij(i, sz) for i in range(row_size * col_size)])
    Irc, Irc_sink = communicable_gpt(P_sink, all_coords, sz, comm_radius, list(O_lin))
    _, Irs, _ = sensing_gpt(P_sink, all_coords, sz, sensing_radius, list(O_lin))
    print("Memory usage after computing neighborhoods:")
    get_total_size(Irc, verbose=True)
    get_total_size(Irc_sink, verbose=True)
    get_total_size(Irs, verbose=True)
    # ==============================
    # 4. BUILD CONSTRAINT MATRICES
    # ==============================
    print("4. Building common constraint matrices with toggles...")
    C2, C3 = position_and_collision_constraints(vh, P_sink, constraints_cfg)
    C4, C5 = connectivity_constraints(vh, Irc, Irc_sink, P_sink, constraints_cfg)
    C6, C7a, C7b, C7c = movement_and_mobility_constraints(vh, Irc, P_sink, constraints_cfg)
    battery_blocks = battery_constraints(vh, b_mov, b_steady, b_full, P_sink, initial_battery, constraints_cfg)
    C13, C14, C15 = cell_coverage_constraints(vh, Irs, constraints_cfg)
    
    print("4a. Building model-specific constraints...")
    leq_model_block, geq_model_block = model_specific_constraints(vh, model_cfg, constraints_cfg, b_mov, b_steady)

    # ==============================
    # 5. DEFINE BOUNDS AND TYPES
    # ==============================
    print("5. Defining variable bounds and types...")
    lb = np.zeros(vh.total_vars); ub = np.ones(vh.total_vars); ctype = ['B'] * vh.total_vars

    any_battery_enabled = any(constraints_cfg.get(k, False) for k in [
        "eq8", "eq9", "eq10", "eq11", "eq12"
    ])

    if any_battery_enabled:
        print("Battery constraints are active. Setting battery variable types and bounds.")
        for t in range(T):
            for n in range(N):
                ctype[vh.b(t, n)] = 'C'
                lb[vh.b(t, n)] = 0.0
                ub[vh.b(t, n)] = b_full # Default max, can be overridden by eq11
        
        if constraints_cfg.get("eq11", False):
             for t in range(T):
                for n in range(N):
                    ub[vh.b(t, n)] = b_full
        
        if constraints_cfg.get("eq12", False):
            for t in range(T):
                for n in range(N):
                    lb[vh.b(t, n)] = e_base

    # Obstacle constraints
    for q_obs in O_lin:
        for t in range(T):
            for n in range(N): ub[vh.z(t, n, q_obs)] = 0

    # Coverage variables are continuous [0,1]
    any_coverage_enabled = any(constraints_cfg.get(k, False) for k in [
        "eq13", "eq14", "eq15"
    ])
    if any_coverage_enabled:
        for i in range(vh.num_grid_cells):
            ctype[vh.c_i(i)] = 'C'
            for t in range(T):
                for n in range(N): ctype[vh.c_in_k(t,n,i)] = 'C'

    print("\nMemory usage after building constraints:")
    get_total_size(C2, verbose=True)
    get_total_size(C3, verbose=True)
    get_total_size(C4, verbose=True)
    get_total_size(C5, verbose=True)
    get_total_size(C6, verbose=True)
    get_total_size(C7a, verbose=True)
    get_total_size(C7b, verbose=True)
    get_total_size(C7c, verbose=True)
    get_total_size(battery_blocks, verbose=True)
    get_total_size(C13, verbose=True)
    get_total_size(C14, verbose=True)
    get_total_size(C15, verbose=True)
    
    
    # ==============================
    # 🔍 5a. FEASIBILITY DIAGNOSTICS (insert here)
    # ==============================
    print("\nRunning feasibility diagnostics before combining constraints...\n")
    quick_feasibility_checks(vh, Irs, model_cfg)
    inspect_Irs_Irc(Irs, Irc)
    check_variable_bounds_and_types(vh, lb, ub, ctype)
    check_bigM_consistency(vh, b_mov, b_steady, b_full)
    
    # We'll temporarily compute LP-relaxation max coverage here:
    print("Computing LP relaxation (max coverage)...")
    # (You’ll define this function above main, same as I gave earlier)
    A_eq_dummy = np.empty((0, vh.total_vars))
    b_eq_dummy = np.array([])
    A_ineq_dummy = np.empty((0, vh.total_vars))
    b_ineq_dummy = np.array([])
    senses_dummy = ""
    max_cov, stat = compute_max_coverage_relax(
        vh, A_eq_dummy, b_eq_dummy, A_ineq_dummy, b_ineq_dummy, senses_dummy, lb, ub
    )
    print(f"LP-relax max coverage (without constraints): {max_cov}, status: {stat}\n")
    
    
    
    # ==============================
    # 6. COMBINE CONSTRAINTS
    # ==============================
    print("6. Combining all constraints...")
    eq_blocks = [
        (C2, np.ones(C2.shape[0])),
        (battery_blocks['initial'], battery_blocks['rhs_initial']),
        (C13, np.zeros(C13.shape[0]))
    ]
    leq_blocks = [
        (C3, np.ones(C3.shape[0])), (C5, np.zeros(C5.shape[0])),
        (C6, np.zeros(C6.shape[0])), (C7a, np.zeros(C7a.shape[0])),
        (C7b, np.zeros(C7b.shape[0])), (C7c, np.ones(C7c.shape[0])),
        (battery_blocks['charge_loc'], np.zeros(battery_blocks['charge_loc'].shape[0])),
        (battery_blocks['discharge_leq'], np.zeros(battery_blocks['discharge_leq'].shape[0])),
        (battery_blocks['charge_leq'], battery_blocks['rhs_charge_leq']),
        (C14, np.zeros(C14.shape[0])), (C15, np.zeros(C15.shape[0]))
    ]
    geq_blocks = [
        (C4, np.ones(C4.shape[0])),
        (battery_blocks['discharge_geq'], np.zeros(battery_blocks['discharge_geq'].shape[0])),
        (battery_blocks['charge_geq'], battery_blocks['rhs_charge_geq'])
    ]

    if leq_model_block: leq_blocks.append(leq_model_block)
    if geq_model_block: geq_blocks.append(geq_model_block)

    A_eq, b_eq, A_ineq, b_ineq, senses = combine_constraints(eq_blocks, leq_blocks, geq_blocks)
    print("\nMemory usage after combining constraints:")
    get_total_size(A_eq, verbose=True)
    get_total_size(b_eq, verbose=True)
    get_total_size(A_ineq, verbose=True)
    get_total_size(b_ineq, verbose=True)
    # ==============================
    # 7. SOLVE MILP
    # ==============================
    print("7. Handing off to CPLEX solver...")
    solution, fval, _, status = cplex_solver(
        "CoverageOptimization", objective, objective_sense, lb, ub, "".join(ctype),
        A_eq, b_eq, A_ineq, b_ineq, senses, time_limit, mipgap
    )

    # ==============================
    # 8. PROCESS AND DISPLAY RESULTS
    # ==============================
    print("\n--- Final Status ---")
    print(f"Solver Status: {status}")
    print(f"Objective Value: {fval if fval is not None else 'N/A'}")
    
    if solution is not None:
        uav_paths, covered_cells_idx, uav_covered_nodes = process_results(solution, vh, Irs, sz)
        
        battery_levels = {}
        if any_battery_enabled:
             battery_levels = {n: [solution[vh.b(t, n)] for t in range(vh.T)] for n in range(vh.N)}

        display_results(vh, uav_paths, covered_cells_idx, battery_levels)
        
        aux_tensor = None
        if battery_levels:
            aux_tensor = np.array([battery_levels[n] for n in range(vh.N)]).T.reshape(T,N,1)

        render_at_t, T = plot_interactive_paths(
            G=None, uav_paths=uav_paths, uav_covered_nodes=uav_covered_nodes,
            sink=P_sink, Rs=sensing_radius, Rc=comm_radius,
            Nx=col_size, Ny=row_size, O_lin=list(O_lin), aux_tensor=aux_tensor
        )
        # animate_paths(G=None, uav_paths=uav_paths, uav_covered_nodes=uav_covered_nodes, sink=P_sink, Rs=sensing_radius, Rc=comm_radius,
        #       Nx=col_size, Ny=row_size, O_lin=O_lin, aux_tensor=aux_tensor,
        #       filename="coverage.mp4", fps=1)

        # fig = plt.figure(figsize=(16, 8))
        # # (setup axes as before)

        # ani = animation.FuncAnimation(fig, lambda t: render_at_t(t), frames=T, interval=500)
        # ani.save("uav_simulation.mp4", writer="ffmpeg", fps=2)
    else:
        print("Optimization failed to find a feasible solution.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run Drone Coverage Optimization MILP.")
    parser.add_argument("--config", type=str, default="config.json", help="Path to JSON config file.")
    args = parser.parse_args()
    
    try:
        config = load_config(args.config)
        main(config)
        print("\nOptimization script finished.")
    except FileNotFoundError:
        print(f"Error: Config file not found at {args.config}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")