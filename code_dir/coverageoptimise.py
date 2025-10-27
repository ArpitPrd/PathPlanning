import numpy as np
import argparse
from coverage_utils import (
    load_config, VarHelper, setup_objective_and_sense, pos_sink, pos_obs, ij_to_i, i_to_ij,
    position_and_collision_constraints, connectivity_constraints, movement_and_mobility_constraints,
    cell_coverage_constraints, battery_constraints, model_specific_constraints, combine_constraints, cplex_solver
)
from Gpt import communicable_gpt, sensing_gpt
from pathplotter import plot_interactive_paths # Assuming this is in pathplotter.py
# from battery_model import compute_battery_for_paths # Assuming this is in battery_model.py

def process_results(solution, vh, Irs, sz):
    uav_paths = {n: [] for n in range(vh.N)}
    uav_covered_nodes = {n: [] for n in range(vh.N)} # This seems for plotting, keeping it

    for n in range(vh.N):
        for t in range(vh.T):
            pos_found = False
            for i in range(vh.num_grid_cells):
                if solution[vh.z(t, n, i)] > 0.99:
                    uav_paths[n].append(i_to_ij(i, sz)) # Store (r,c) coords
                    
                    # Store nodes sensed from this position
                    sensed_nodes_from_pos = []
                    for s_idx in Irs[i]:
                        sensed_nodes_from_pos.append(i_to_ij(s_idx, sz))
                    uav_covered_nodes[n].append(sensed_nodes_from_pos)
                    
                    pos_found = True
                    break
            if not pos_found:
                uav_paths[n].append(None); uav_covered_nodes[n].append([])

    covered_cells_idx = {i for i in range(vh.num_grid_cells) if solution[vh.c_i(i)] > 0.99}
    covered_cells_coords = {i_to_ij(i, sz) for i in covered_cells_idx}
    
    return uav_paths, covered_cells_coords, uav_covered_nodes

def display_results(vh, uav_paths, covered_cells_coords, battery_levels):
    print("\n" + "="*50 + "\nOPTIMIZATION RESULTS\n" + "="*50)
    for n in range(vh.N):
        print(f"\n--- UAV {n+1} ---")
        
        # --- CORRECTED LINE ---
        # Iterate over 'pos' first, then check if it's None before unpacking.
        path_coords = [f"({pos[0]+1},{pos[1]+1})" if pos is not None else 'N/A' for pos in uav_paths[n]]
        
        print(f"  Path (row, col): {path_coords}")
        
        if battery_levels and n in battery_levels: # Only print if battery levels exist for this UAV
            print(f"  Battery Levels: {[round(b, 2) for b in battery_levels[n]]}")

    # Note: num_grid_cells includes the sink. Paper excludes sink from coverage.
    # Adjusting for C_max = |G_s_bar|
    non_sink_cells = vh.num_grid_cells - 1 
    # Ensure denominator is not zero if grid is 1x1
    coverage_percent = len(covered_cells_coords) / max(1, non_sink_cells) * 100
    print(f"\nTotal Cells Covered (excl. sink): {len(covered_cells_coords)} / {non_sink_cells} ({coverage_percent:.2f}%)")
    # print(f"  Covered Coords: {sorted(list(covered_cells_coords))}")

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
    
    solver_cfg = cfg.get("solver", {}); time_limit, mipgap = solver_cfg.get("time_limit", 600), solver_cfg.get("mipgap", 0.01)
    
    # --- Read Model and Constraint Toggles from Config ---
    model_cfg = cfg.get("model", {"name": "maximize_coverage"})
    constraints_cfg = cfg.get("constraints", {})
    print(f"Using optimization model: {model_cfg.get('name')}")
    print(f"Constraints config: {constraints_cfg}")

    sz = (row_size, col_size); P_sink, sink_coord_rc0 = pos_sink(row_sink - 1, col_sink - 1, sz)
    obs = pos_obs(coords_obs); O_lin = {ij_to_i((r - 1, c - 1), sz) for r, c in obs} if obs.size > 0 else set()
    
    # ==============================
    # 2. INITIALIZE VARHELPER & OBJECTIVE
    # ==============================
    print("2. Initializing variable helper...")
    # *** CHANGED: Pass configs to VarHelper ***
    vh = VarHelper(N, T, row_size, col_size, constraints_cfg, model_cfg)
    
    print("2a. Setting up objective function...")
    objective, objective_sense = setup_objective_and_sense(vh, model_cfg, b_mov, b_steady)

    # ==============================
    # 3. COMPUTE NEIGHBORHOOD SETS
    # ==============================
    print("3. Computing neighborhood sets (Sensing/Communication)...")
    all_coords = np.array([i_to_ij(i, sz) for i in range(row_size * col_size)])
    Irc, Irc_sink = communicable_gpt(P_sink, all_coords, sz, comm_radius, list(O_lin))
    Irs = sensing_gpt(P_sink, all_coords, sz, sensing_radius, list(O_lin))

    # ==============================
    # 4. BUILD CONSTRAINT MATRICES
    # ==============================
    print("4. Building common constraint matrices with toggles...")
    C2, C3 = position_and_collision_constraints(vh, P_sink, constraints_cfg)
    C4, C5 = connectivity_constraints(vh, Irc, Irc_sink, P_sink, constraints_cfg)
    C6, C7a, C7b, C7c = movement_and_mobility_constraints(vh, Irc, constraints_cfg)
    battery_blocks = battery_constraints(vh, b_mov, b_steady, b_full, P_sink, initial_battery, constraints_cfg)
    
    # *** CHANGED: Unpack new Eq13 blocks ***
    C13_leq, C13_geq, C14, C15 = cell_coverage_constraints(vh, Irs, constraints_cfg)
    
    print("4a. Building model-specific constraints...")
    leq_model_block, geq_model_block = model_specific_constraints(vh, model_cfg, b_mov, b_steady)

    # ==============================
    # 5. DEFINE BOUNDS AND TYPES
    # ==============================
    print("5. Defining variable bounds and types...")
    lb = np.zeros(vh.total_vars); ub = np.ones(vh.total_vars); ctype = ['B'] * vh.total_vars

    # *** CHANGED: Check vh.has_b_vars ***
    if vh.has_b_vars:
        print("Battery variables are active. Setting battery variable types and bounds.")
        for t in range(T):
            for n in range(N):
                ctype[vh.b(t, n)] = 'C'
                lb[vh.b(t, n)] = 0.0
                ub[vh.b(t, n)] = b_full # Default max
        
        if constraints_cfg.get("eq11", False): # Max battery
             for t in range(T):
                for n in range(N):
                    ub[vh.b(t, n)] = b_full
        
        if constraints_cfg.get("eq12", False): # Min battery
            for t in range(T):
                for n in range(N):
                    lb[vh.b(t, n)] = e_base
    
    # *** CHANGED: Check vh.has_x_vars ***
    if not vh.has_x_vars:
        # If x-vars don't exist, ensure any binary vars at their (non-existent)
        # index are just standard [0,1] 'B'
        pass # ctype is already 'B' by default, lb=0, ub=1. No change needed.

    # Obstacle constraints (on z-vars)
    for q_obs in O_lin:
        for t in range(T):
            for n in range(N): ub[vh.z(t, n, q_obs)] = 0

    # Coverage variables are continuous [0,1] (always active)
    for i in range(vh.num_grid_cells):
        ctype[vh.c_i(i)] = 'C'
        for t in range(T):
            for n in range(N): ctype[vh.c_in_k(t,n,i)] = 'C'

    # ==============================
    # 6. COMBINE CONSTRAINTS
    # ==============================
    print("6. Combining all constraints...")
    
    # *** CHANGED: Removed C13 from eq_blocks ***
    eq_blocks = [
        (C2, np.ones(C2.shape[0])),
        (battery_blocks['initial'], battery_blocks['rhs_initial'])
    ]
    
    # *** CHANGED: Added C13_leq to leq_blocks ***
    leq_blocks = [
        (C3, np.ones(C3.shape[0])), (C5, np.zeros(C5.shape[0])),
        (C6, np.zeros(C6.shape[0])), (C7a, np.zeros(C7a.shape[0])),
        (C7b, np.zeros(C7b.shape[0])), (C7c, np.ones(C7c.shape[0])),
        (battery_blocks['charge_loc'], np.zeros(battery_blocks['charge_loc'].shape[0])),
        (battery_blocks['discharge_leq'], np.zeros(battery_blocks['discharge_leq'].shape[0])),
        (battery_blocks['charge_leq'], battery_blocks['rhs_charge_leq']),
        (C14, np.zeros(C14.shape[0])), (C15, np.zeros(C15.shape[0])),
        (C13_leq, np.zeros(C13_leq.shape[0])) # <-- ADDED
    ]
    
    # *** CHANGED: Added C13_geq to geq_blocks ***
    geq_blocks = [
        (C4, np.ones(C4.shape[0])),
        (battery_blocks['discharge_geq'], np.zeros(battery_blocks['discharge_geq'].shape[0])),
        (battery_blocks['charge_geq'], battery_blocks['rhs_charge_geq']),
        (C13_geq, np.zeros(C13_geq.shape[0])) # <-- ADDED
    ]

    # Add model-specific constraints
    if leq_model_block and leq_model_block[0] is not None: 
        leq_blocks.append((leq_model_block[0], leq_model_block[1]))
    if geq_model_block and geq_model_block[0] is not None: 
        geq_blocks.append((geq_model_block[0], geq_model_block[1]))

    A_eq, b_eq, A_ineq, b_ineq, senses = combine_constraints(eq_blocks, leq_blocks, geq_blocks)

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
        # Note: P_sink is 0-based linear index, need (r,c) 1-based for plotter
        sink_rc1 = (row_sink, col_sink) 
        
        uav_paths_rc0, covered_cells_coords, uav_covered_nodes_rc0 = process_results(solution, vh, Irs, sz)
        
        # Convert paths from 0-based (r,c) to 1-based (r,c) for plotter/display
        uav_paths_rc1 = {n: [(r+1, c+1) if (r,c) is not None else None for (r,c) in path] 
                         for n, path in uav_paths_rc0.items()}
        
        battery_levels = {}
        # *** CHANGED: Check vh.has_b_vars ***
        if vh.has_b_vars:
             battery_levels = {n: [solution[vh.b(t, n)] for t in range(T)] for n in range(vh.N)}

        display_results(vh, uav_paths_rc0, covered_cells_coords, battery_levels)
        
        # Post-process paths and battery for plotting
        # (This part depends on your external plotter and battery_model files)
        try:
            # Example: Re-compute battery trace using external model for plotting
            # battery_traces, position_traces = compute_battery_for_paths(
            #     uav_paths_rc1, sink_rc1, T
            # )
            
            plot_interactive_paths(
                G=None, 
                uav_paths=uav_paths_rc1, 
                uav_covered_nodes=uav_covered_nodes_rc0, # Pass 0-indexed coords
                sink=sink_rc1, 
                Rs=sensing_radius, Rc=comm_radius,
                Nx=row_size, Ny=col_size, O_lin=list(O_lin),
                battery_traces=None, # battery_traces, # Pass real traces if computed
                position_traces=None # position_traces
            )
        except Exception as plot_e:
            print(f"\nCould not generate plot: {plot_e}")
            print("Check 'pathplotter.py' and 'battery_model.py' imports.")

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
        import traceback
        traceback.print_exc()