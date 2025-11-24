import numpy as np
import argparse
from coverage_utils import (
    load_config, VarHelper, setup_objective_and_sense, pos_sink, pos_obs, ij_to_i, i_to_ij,
    position_and_collision_constraints, connectivity_constraints, movement_and_mobility_constraints,
    cell_coverage_constraints, battery_constraints, model_specific_constraints, combine_constraints, cplex_solver
)
from Gpt import communicable_gpt, sensing_gpt, movement_gpt
from pathplotter import plot_interactive_paths  # <-- IMPORT ADDED

def process_results(solution, vh, Irs, sz, P_sink):
    """
    Processes the CPLEX solution vector into paths and coverage lists.
    - uav_paths_lin: Dict of paths as lists of LINEAR indices (for plotter)
    - uav_paths_rc0: Dict of paths as lists of (r,c) tuples (for display)
    - covered_cells_coords: Set of (r,c) tuples of globally covered cells
    - uav_covered_nodes_lin: Dict of lists of lists of LINEAR indices (for plotter)
    """
    uav_paths_lin = {n: [] for n in range(vh.N)}
    uav_paths_rc0 = {n: [] for n in range(vh.N)}
    uav_covered_nodes_lin = {n: [] for n in range(vh.N)}

    for n in range(vh.N):
        for t in range(vh.T):
            pos_found = False
            for i in range(vh.num_grid_cells):
                if solution[vh.z(t, n, i)] > 0.99:
                    # Store linear index for plotter
                    uav_paths_lin[n].append(i)
                    # Store (r,c) tuple for text display
                    uav_paths_rc0[n].append(i_to_ij(i, sz))
                    
                    # Store sensed nodes as linear indices (for plotter)
                    sensed_by_i_lin = []
                    for j in range(len(Irs)):
                        if i in Irs[j]:
                            sensed_by_i_lin.append(j)
                    # sensed_by_i_lin = Irs[i] if i < len(Irs) else []
                    uav_covered_nodes_lin[n].append(sensed_by_i_lin)
                    
                    pos_found = True
                    break
            if not pos_found:
                uav_paths_lin[n].append(None)
                uav_paths_rc0[n].append(None)
                uav_covered_nodes_lin[n].append([])

    covered_cells_idx = {i for i in range(vh.num_grid_cells) if solution[vh.c_i(i)] > 0.99 and i != P_sink}
    covered_cells_coords = {i_to_ij(i, sz) for i in covered_cells_idx}
    return uav_paths_lin, uav_paths_rc0, covered_cells_coords, uav_covered_nodes_lin

def display_results(vh, uav_paths_rc0, covered_cells_coords, battery_levels):
    """
    Prints the results to the console.
    Receives uav_paths_rc0, which is already a list of (r,c) tuples or None.
    """
    print("\n" + "="*50 + "\nOPTIMIZATION RESULTS\n" + "="*50)
    for n in range(vh.N):
        print(f"\n--- UAV {n+1} ---")
        
        # --- CORRECTED LOGIC ---
        # uav_paths_rc0 is now a list of (r,c) tuples or None
        path_coords = [f"({pos[0]+1},{pos[1]+1})" if pos is not None else 'N/A' for pos in uav_paths_rc0[n]]
        
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
    movement_type = cfg.get("movement_type", "euclidean")
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
    vh = VarHelper(N, T, row_size, col_size, constraints_cfg, model_cfg)
    
    print("2a. Setting up objective function...")
    objective, objective_sense = setup_objective_and_sense(vh, model_cfg, b_mov, b_steady, P_sink)

    # ==============================
    # 3. COMPUTE NEIGHBORHOOD SETS
    # ==============================
    print("3. Computing neighborhood sets (Sensing/Communication)...")
    all_coords = np.array([i_to_ij(i, sz) for i in range(row_size * col_size)])
    Irc, Irc_sink = communicable_gpt(P_sink, all_coords, sz, comm_radius, list(O_lin))
    _,Irs_neighbors,_ = sensing_gpt(P_sink, all_coords, sz, sensing_radius, list(O_lin))
    Irc[P_sink] = sorted(list(set(Irc[P_sink]) | {P_sink}))
    Irc_sink = Irc[P_sink]
    
    if movement_type.lower() == "manhattan":
        Imv, Imv_sink = movement_gpt(P_sink, all_coords, sz, comm_radius, list(O_lin))
        print("Using Manhattan distance for movement (BFS-based)")
    else:  # euclidean
        Imv, Imv_sink = Irc, Irc_sink
        print("Using Euclidean distance for movement (same as communication range)")
    # for i in range(len(Imv)):
    #     if P_sink in Imv[i]:
    #         Imv[i] = [cell for cell in Imv[i] if cell != P_sink]
    # Imv[P_sink] = []  
    
    # ← END OF ADDED BLOCK
    Irs = []
    for i in range(vh.num_grid_cells):
        # Add the cell itself (linear index i) to its list of neighbors
        if i == P_sink:
            Irs.append(sorted(list(set(Irs_neighbors[i]))))  # Sink does not include itself
        else:
            Irs.append(sorted(list(set(Irs_neighbors[i]) | {i})))

    # ==============================
    # 4. BUILD CONSTRAINT MATRICES
    # ==============================
    print("4. Building common constraint matrices with toggles...")
    C2, C3 = position_and_collision_constraints(vh, P_sink, constraints_cfg)
    C4, C5 = connectivity_constraints(vh, Irc, Irc_sink, P_sink, constraints_cfg)
    C6, C7a, C7b, C7c = movement_and_mobility_constraints(vh, Imv, constraints_cfg)
    battery_blocks = battery_constraints(vh, b_mov, b_steady, b_full, P_sink, initial_battery, constraints_cfg)
    
    C13_leq, C13_geq, C14, C15 = cell_coverage_constraints(vh, Irs, constraints_cfg, P_sink)
    
    print("4a. Building model-specific constraints...")
    leq_model_block, geq_model_block = model_specific_constraints(vh, model_cfg, b_mov, b_steady)

    # ==============================
    # 5. DEFINE BOUNDS AND TYPES
    # ==============================
    print("5. Defining variable bounds and types...")
    lb = np.zeros(vh.total_vars); ub = np.ones(vh.total_vars); ctype = ['B'] * vh.total_vars

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
    
    eq_blocks = [
        (C2, np.ones(C2.shape[0])),
        (battery_blocks['initial'], battery_blocks['rhs_initial'])
    ]
    
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
        # --- UPDATED DATA PROCESSING ---
        uav_paths_lin, uav_paths_rc0, covered_cells_coords, uav_covered_nodes_lin = process_results(solution, vh, Irs, sz, P_sink)
        
        battery_levels = {}
        aux_tensor = None
        if vh.has_b_vars:
             battery_levels = {n: [solution[vh.b(t, n)] for t in range(T)] for n in range(vh.N)}
             # Reshape for plotter
             aux_tensor = np.array([battery_levels[n] for n in range(vh.N)]).T.reshape(T,N,1)

        # Display text results using (r,c) paths
        display_results(vh, uav_paths_rc0, covered_cells_coords, battery_levels)
        
        # Plot results using linear-index paths
        try:
            plot_interactive_paths(
                G=None, 
                uav_paths=uav_paths_lin,            # <-- Pass linear index paths
                uav_covered_nodes=uav_covered_nodes_lin, # <-- Pass linear index coverage
                sink=P_sink,                        # <-- Pass linear index sink
                Rs=sensing_radius, Rc=comm_radius,
                Nx=col_size,                        # <-- Pass col_size as Nx
                Ny=row_size,                        # <-- Pass row_size as Ny
                O_lin=list(O_lin),
                aux_tensor=aux_tensor               # <-- Pass battery tensor
            )
            print("\nPlot saved to plot.png and displayed.")
        except Exception as plot_e:
            print(f"\nCould not generate plot: {plot_e}")
            print("Check 'pathplotter.py' for errors.")

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