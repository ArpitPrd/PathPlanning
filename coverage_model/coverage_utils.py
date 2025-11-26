import numpy as np
import cplex
from cplex.exceptions import CplexError
import time
import json

# ==============================================================================
# CONFIG AND GRID HELPERS (No changes)
# ==============================================================================
def load_config(json_path: str):
    with open(json_path, 'r') as f:
        return json.load(f)

def i_to_ij(index, sz):
    return np.unravel_index(index, sz)

def ij_to_i(cod, sz):
    return np.ravel_multi_index(cod, sz)

def pos_sink(row_sink, col_sink, sz):
    p_sink = ij_to_i((row_sink, col_sink), sz)
    sink_coord = np.array([row_sink, col_sink])
    return p_sink, sink_coord

def pos_obs(coords_obs):
    if not coords_obs: return np.array([])
    return np.array(coords_obs)

# ==============================================================================
# DYNAMIC VARIABLE INDEXING HELPER CLASS (Rewritten)
# ==============================================================================
class VarHelper:
    """
    Manages indices of decision variables *dynamically* based on config.
    Only creates variables that are actually needed by the enabled constraints.
    """
    def __init__(self, N, T, row_size, col_size, constraints_cfg, model_cfg):
        self.N, self.T = N, T
        self.rs, self.cs = row_size, col_size
        self.num_grid_cells = row_size * col_size
        
        # --- Decide which optional variables are needed ---
        model_name = model_cfg.get("name")
        
        # m-vars (movement) are needed for:
        # 1. Eq 7 (Movement definition)
        # 2. Eq 9 (Battery discharge)
        # 3. Model "minimize_energy" (Objective)
        # 4. Model "maximize_coverage" with a B_max budget (Eq 16)
        self.has_m_vars = constraints_cfg.get("eq7", False) or \
                          constraints_cfg.get("eq9", False) or \
                          model_name == "minimize_energy" or \
                          (model_name == "maximize_coverage" and "B_max" in model_cfg)

        # x-vars (charging) are needed for:
        # 1. Eq 8 (Charging location)
        # 2. Eq 9 (Battery discharge)
        # 3. Eq 10 (Battery charge)
        self.has_x_vars = constraints_cfg.get("eq8", False) or \
                          constraints_cfg.get("eq9", False) or \
                          constraints_cfg.get("eq10", False)
                          
        # b-vars (battery level) are needed if any battery logic is on
        # (for initial state, min/max, or charging/discharging)
        self.has_b_vars = self.has_x_vars or \
                          constraints_cfg.get("eq11", False) or \
                          constraints_cfg.get("eq12", False)

        # --- Sequentially build variable offsets ---
        offset = 0
        
        # z (position) - always on
        self.num_z = self.num_grid_cells * N * T
        self.start_z = offset
        offset += self.num_z
        
        # c_i (global coverage) - always on
        self.num_c_i = self.num_grid_cells
        self.start_c_i = offset
        offset += self.num_c_i
        
        # c_in_k (local coverage) - always on
        self.num_c_in_k = self.num_grid_cells * N * T
        self.start_c_in_k = offset
        offset += self.num_c_in_k
        
        # m (movement) - optional
        self.start_m = offset
        if self.has_m_vars:
            self.num_m = self.num_grid_cells**2 * N * (T - 1)
            offset += self.num_m
            
        # x_charge (charging) - optional
        self.start_x_charge = offset
        if self.has_x_vars:
            self.num_x = N * T
            offset += self.num_x

        # b (battery level) - optional
        self.start_b = offset
        if self.has_b_vars:
            self.num_b = N * T
            offset += self.num_b
            
        self.total_vars = offset
        
        print(f"VarHelper initialized: Total variables = {self.total_vars}")
        print(f"  - has_m_vars: {self.has_m_vars}")
        print(f"  - has_x_vars: {self.has_x_vars}")
        print(f"  - has_b_vars: {self.has_b_vars}")


    def z(self, t, n, i): return self.start_z + t*self.N*self.num_grid_cells + n*self.num_grid_cells + i
    def c_i(self, i): return self.start_c_i + i
    def c_in_k(self, t, n, i): return self.start_c_in_k + t*self.N*self.num_grid_cells + n*self.num_grid_cells + i
    
    def m(self, t, n, i, j): 
        if not self.has_m_vars: raise AttributeError("m-variables are not active in this model.")
        return self.start_m + t*self.N*self.num_grid_cells**2 + n*self.num_grid_cells**2 + i*self.num_grid_cells + j
    
    def x_charge(self, t, n): 
        if not self.has_x_vars: raise AttributeError("x_charge-variables are not active in this model.")
        return self.start_x_charge + t * self.N + n
        
    def b(self, t, n): 
        if not self.has_b_vars: raise AttributeError("b-variables are not active in this model.")
        return self.start_b + t * self.N + n

# ==============================================================================
# OBJECTIVE AND CONSTRAINTS (Updated for Toggling and Correctness)
# ==============================================================================
def setup_objective_and_sense(vh, model_cfg, b_mov, b_steady, P_sink):
    """Sets up the objective vector and optimization sense based on the model config."""
    model_name = model_cfg.get("name", "maximize_coverage")
    f = np.zeros(vh.total_vars)
    
    if model_name == "maximize_coverage":
        # Model 1: Maximize sum(c_i)
        for i in range(vh.num_grid_cells):
            if i == P_sink: continue
            f[vh.c_i(i)] = 1.0
        f[P_sink] = 0.0  # Ensure sink coverage is not counted
        return f, cplex.Cplex().objective.sense.maximize
    
    elif model_name == "minimize_energy":
        # Model 2: Minimize total energy consumption
        if not vh.has_m_vars:
            raise ValueError("Cannot 'minimize_energy': m-variables are not active. Enable 'eq7' or 'eq9' in config.")
        
        for t in range(vh.T - 1):
            for n in range(vh.N):
                for i in range(vh.num_grid_cells):
                    for j in range(vh.num_grid_cells):
                        cost = b_steady if i == j else b_mov
                        f[vh.m(t, n, i, j)] = cost
        return f, cplex.Cplex().objective.sense.minimize
        
    else:
        raise ValueError(f"Unknown model name in config: {model_name}")

def position_and_collision_constraints(vh, P_sink, cfg):
    # Eq 2: Unique position
    eq2_rows = []
    if cfg.get("eq2", True):
        for t in range(vh.T):
            for n in range(vh.N):
                row = np.zeros(vh.total_vars)
                for i in range(vh.num_grid_cells):
                    row[vh.z(t, n, i)] = 1
                eq2_rows.append(row)
    
    # Eq 3: Collision avoidance
    eq3_rows = []
    if cfg.get("eq3", True):
        for t in range(vh.T):
            for i in range(vh.num_grid_cells):
                if i == P_sink: continue
                row = np.zeros(vh.total_vars)
                for n in range(vh.N):
                    row[vh.z(t, n, i)] = 1
                eq3_rows.append(row)
            
    return np.array(eq2_rows), np.array(eq3_rows)

def connectivity_constraints(vh, Irc, Irc_sink, P_sink, cfg):
    # Eq 4: Sink connectivity
    eq4_rows = []
    if cfg.get("eq4", True):
        for t in range(vh.T):
            row = np.zeros(vh.total_vars)
            for n in range(vh.N):
                for p in Irc_sink:
                    row[vh.z(t, n, p)] = 1
            eq4_rows.append(row)

    # Eq 5: Inter-UAV link
    eq5_rows = []
    if cfg.get("eq5", True):
        for t in range(vh.T):
            for n in range(1, vh.N):
                for i in range(vh.num_grid_cells):
                    row = np.zeros(vh.total_vars)
                    row[vh.z(t, n, i)] = 1
                    for p in Irc[i]:
                        row[vh.z(t, n - 1, p)] = -1
                    eq5_rows.append(row)

    return np.array(eq4_rows), np.array(eq5_rows)

def movement_and_mobility_constraints(vh, Irc, cfg):
    # Eq 6: Mobility rule
    eq6_rows = []
    if cfg.get("eq6", True):
        # NEW TOGGLE: 'eq6_allow_stay'
        # True = (eq.pdf logic): z(t+1) <= z(t) + sum(z_neighbors)
        # False = (ccpp.pdf logic): z(t+1) <= sum(z_neighbors)
        allow_stay = cfg.get("eq6_allow_stay", True)
        
        for t in range(vh.T - 1):
            for n in range(vh.N):
                for i in range(vh.num_grid_cells):
                    row = np.zeros(vh.total_vars)
                    row[vh.z(t + 1, n, i)] = 1
                    
                    # Determine neighbor set based on toggle
                    neighbors = set(Irc[i] + [i]) if allow_stay else set(Irc[i])
                    
                    for p in neighbors:
                        row[vh.z(t, n, p)] = -1
                    eq6_rows.append(row)

    # Eq 7a, 7b, 7c: Movement definition
    eq7a, eq7b, eq7c = [], [], []
    if cfg.get("eq7", False):
        if not vh.has_m_vars:
            print("Warning: eq7 is True in config, but m-vars were not created. Skipping eq7.")
        else:
            for t in range(vh.T - 1):
                for n in range(vh.N):
                    for i in range(vh.num_grid_cells):
                        for j in range(vh.num_grid_cells):
                            # 7a: m <= z(t)
                            r7a = np.zeros(vh.total_vars); r7a[vh.m(t,n,i,j)]=1; r7a[vh.z(t,n,i)]=-1; eq7a.append(r7a)
                            # 7b: m <= z(t+1)
                            r7b = np.zeros(vh.total_vars); r7b[vh.m(t,n,i,j)]=1; r7b[vh.z(t+1,n,j)]=-1; eq7b.append(r7b)
                            # 7c: m >= z(t) + z(t+1) - 1
                            r7c = np.zeros(vh.total_vars); r7c[vh.m(t,n,i,j)]=-1; r7c[vh.z(t,n,i)]=1; r7c[vh.z(t+1,n,j)]=1; eq7c.append(r7c)
    
    return np.array(eq6_rows), np.array(eq7a), np.array(eq7b), np.array(eq7c)

def battery_constraints(vh, b_mov, b_steady, b_full, P_sink, initial_battery, cfg):
    # Check if *any* battery logic is enabled
    any_battery_logic = vh.has_b_vars or vh.has_x_vars
    
    empty = np.empty((0, vh.total_vars))
    results = {
        'charge_loc': empty, 'discharge_leq': empty, 'discharge_geq': empty,
        'charge_leq': empty, 'rhs_charge_leq': np.array([]),
        'charge_geq': empty, 'rhs_charge_geq': np.array([]),
        'initial': empty, 'rhs_initial': np.array([])
    }
    
    if not any_battery_logic:
        return results
    
    if not vh.has_b_vars:
        print("Warning: Battery constraints (eq8-10) enabled but no b-vars (eq11/12) are on. Skipping.")
        return results

    M = 25
    eq8, eq9a, eq9b, eq10a, eq10b = [], [], [], [], []

    for t in range(vh.T - 1):
        for n in range(vh.N):
            # Eq 8: Charging location: x <= z_sink
            if cfg.get("eq8", False):
                if not vh.has_x_vars: continue
                row8 = np.zeros(vh.total_vars); row8[vh.x_charge(t,n)]=1; row8[vh.z(t,n,P_sink)]=-1; eq8.append(row8)
            
            # Eq 9: Battery discharge
            if cfg.get("eq9", False):
                if not vh.has_m_vars or not vh.has_x_vars:
                    print("Warning: Skipping eq9. It requires 'eq7' (for m-vars) and 'eq8' or 'eq10' (for x-vars) to be enabled.")
                    continue
                
                energy_consumed = np.zeros(vh.total_vars)
                for i in range(vh.num_grid_cells):
                    for j in range(vh.num_grid_cells):
                        cost = b_steady if i == j else b_mov
                        energy_consumed[vh.m(t, n, i, j)] = cost
                # Eq 9a: b(t+1) <= b(t) - E_consumed + M*x  -->  b(t+1) - b(t) + E_consumed - M*x <= 0
                r9a = np.copy(energy_consumed); r9a[vh.b(t+1,n)]=1; r9a[vh.b(t,n)]=-1; r9a[vh.x_charge(t,n)]=-M; eq9a.append(r9a)
                # Eq 9b: b(t+1) >= b(t) - E_consumed - M*x  -->  b(t+1) - b(t) + E_consumed + M*x >= 0
                r9b = np.copy(energy_consumed); r9b[vh.b(t+1,n)]=1; r9b[vh.b(t,n)]=-1; r9b[vh.x_charge(t,n)]=M; eq9b.append(r9b)

            # Eq 10: Battery charge
            if cfg.get("eq10", False):
                if not vh.has_x_vars: continue
                # Eq 10a: b(t+1) <= b_full + M(1-x) --> b(t+1) + M*x <= b_full + M
                r10a = np.zeros(vh.total_vars); r10a[vh.b(t+1,n)]=1; r10a[vh.x_charge(t,n)]=M; eq10a.append(r10a)
                # Eq 10b: b(t+1) >= b_full - M(1-x) --> b(t+1) - M*x >= b_full - M
                r10b = np.zeros(vh.total_vars); r10b[vh.b(t+1,n)]=1; r10b[vh.x_charge(t,n)]=-M; eq10b.append(r10b)

    results['charge_loc'] = np.array(eq8)
    results['discharge_leq'] = np.array(eq9a)
    results['discharge_geq'] = np.array(eq9b)
    results['charge_leq'] = np.array(eq10a)
    results['rhs_charge_leq'] = np.full(len(eq10a), b_full + M)
    results['charge_geq'] = np.array(eq10b)
    results['rhs_charge_geq'] = np.full(len(eq10b), b_full - M)

    # Initial battery state is mandatory if b-vars exist
    B_initial = np.zeros((vh.N, vh.total_vars))
    rhs_initial = np.full(vh.N, initial_battery)
    for n in range(vh.N): B_initial[n, vh.b(0, n)] = 1
    results['initial'] = B_initial
    results['rhs_initial'] = rhs_initial
            
    return results

def cell_coverage_constraints(vh, Irs, cfg, P_sink):
    # Eq 13: Local coverage definition (CORRECTED Big-M Version)
    eq13_leq = []  # c_in_k <= sum(z)
    eq13_geq = []  # M * c_in_k >= sum(z)
    
    if cfg.get("eq13", True):
        for t in range(vh.T):
            for n in range(vh.N):
                for i in range(vh.num_grid_cells):
                    if i == P_sink: continue
                    # M can be the max number of UAVs (N) or |S_i|
                    M = vh.N 
                    
                    # 1. c_in_k <= sum(z_p)  -->  c_in_k - sum(z_p) <= 0
                    row_leq = np.zeros(vh.total_vars)
                    row_leq[vh.c_in_k(t, n, i)] = 1
                    for p in Irs[i]:
                        row_leq[vh.z(t, n, p)] = -1
                    eq13_leq.append(row_leq)

                    # 2. M * c_in_k >= sum(z_p)  -->  M*c_in_k - sum(z_p) >= 0
                    row_geq = np.zeros(vh.total_vars)
                    row_geq[vh.c_in_k(t, n, i)] = M
                    for p in Irs[i]:
                        row_geq[vh.z(t, n, p)] = -1
                    eq13_geq.append(row_geq)

    # Eq 14 & 15: Global coverage mapping
    eq14, eq15 = [], []
    for i in range(vh.num_grid_cells):
        if i == P_sink: continue
        r15 = np.zeros(vh.total_vars)
        if cfg.get("eq15", True):
            r15[vh.c_i(i)] = 1
            
        for t in range(vh.T):
            for n in range(vh.N):
                if cfg.get("eq14", True):
                    # c_in_k - c_i <= 0
                    r14 = np.zeros(vh.total_vars); r14[vh.c_in_k(t,n,i)]=1; r14[vh.c_i(i)]=-1; eq14.append(r14)
                if cfg.get("eq15", True):
                    # Used to build: c_i - sum(c_in_k) <= 0
                    r15[vh.c_in_k(t, n, i)] = -1
        
        if cfg.get("eq15", True):
            eq15.append(r15)
            
    # Return new LEQ and GEQ blocks for Eq 13
    return np.array(eq13_leq), np.array(eq13_geq), np.array(eq14), np.array(eq15)

def model_specific_constraints(vh, model_cfg, b_mov, b_steady):
    """Builds the single constraint unique to each optimization model."""
    model_name = model_cfg.get("name")

    if model_name == "maximize_coverage":
        B_max = model_cfg.get("B_max")
        if B_max is None:
            return None, None # No constraint if B_max is not specified
        
        if not vh.has_m_vars:
            raise ValueError("Cannot set 'B_max': m-variables are not active. Enable 'eq7' or 'eq9' in config.")

        # Eq 16: Total energy budget: sum(energy) <= B_max
        row = np.zeros(vh.total_vars)
        for t in range(vh.T - 1):
            for n in range(vh.N):
                for i in range(vh.num_grid_cells):
                    for j in range(vh.num_grid_cells):
                        cost = b_steady if i == j else b_mov
                        row[vh.m(t, n, i, j)] = cost
        
        return (np.array([row]), np.array([B_max])), None

    elif model_name == "minimize_energy":
        C_min = model_cfg.get("C_min")
        if C_min is None: raise ValueError("C_min must be set for 'minimize_energy' model.")
            
        # Eq 17: Minimum coverage: sum(c_i) >= C_min
        row = np.zeros(vh.total_vars)
        for i in range(vh.num_grid_cells):
            row[vh.c_i(i)] = 1
            
        return None, (np.array([row]), np.array([C_min]))
    
    return None, None

# ==============================================================================
# CPLEX SOLVER and COMBINER (No changes needed)
# ==============================================================================
def combine_constraints(eq_blocks, leq_blocks, geq_blocks):
    """Combines multiple constraint blocks into final matrices."""
    def vstack_if_any(blocks):
        blocks = [b for b in blocks if b.shape[0] > 0]
        return np.vstack(blocks) if blocks else np.empty((0, blocks[0].shape[1] if blocks else 0))

    A_eq = vstack_if_any([b for b, _ in eq_blocks])
    b_eq = np.hstack([r for b, r in eq_blocks if b.shape[0] > 0])
    
    A_leq = vstack_if_any([b for b, _ in leq_blocks])
    b_leq = np.hstack([r for b, r in leq_blocks if b.shape[0] > 0])
    
    A_geq = vstack_if_any([b for b, _ in geq_blocks])
    b_geq = np.hstack([r for b, r in geq_blocks if b.shape[0] > 0])
    
    A_ineq_list = [A_leq, A_geq]
    b_ineq_list = [b_leq, b_geq]
    senses = 'L' * A_leq.shape[0] + 'G' * A_geq.shape[0]
    
    A_ineq = vstack_if_any(A_ineq_list)
    b_ineq = np.hstack(b_ineq_list) if any(b.size > 0 for b in b_ineq_list) else np.array([])
    
    return A_eq, b_eq, A_ineq, b_ineq, senses

def cplex_solver(prob_name, objective, objective_sense, lb, ub, ctype, A_eq, b_eq, A_ineq, b_ineq, senses, time_limit, mipgap):
    prob = cplex.Cplex()
    prob.set_problem_name(prob_name)
    prob.objective.set_sense(objective_sense)
    prob.variables.add(obj=objective, lb=lb, ub=ub, types=ctype, names=[f"var_{i}" for i in range(len(objective))])

    if A_eq.shape[0] > 0:
        prob.linear_constraints.add(
            lin_expr=[cplex.SparsePair(ind=r.nonzero()[0].tolist(), val=r[r.nonzero()].tolist()) for r in A_eq],
            senses=['E'] * A_eq.shape[0], rhs=b_eq.tolist(),
            names=[f"eq_{i}" for i in range(A_eq.shape[0])]
        )
    if A_ineq.shape[0] > 0:
         prob.linear_constraints.add(
            lin_expr=[cplex.SparsePair(ind=r.nonzero()[0].tolist(), val=r[r.nonzero()].tolist()) for r in A_ineq],
            senses=senses, rhs=b_ineq.tolist(),
            names=[f"ineq_{i}" for i in range(A_ineq.shape[0])]
        )

    prob.parameters.mip.tolerances.mipgap.set(mipgap)
    prob.parameters.timelimit.set(time_limit)
    prob.parameters.threads.set(4)
    # prob.set_log_stream(None); prob.set_error_stream(None); prob.set_warning_stream(None); prob.set_results_stream(None)

    print(f"Starting CPLEX solver (timelimit: {time_limit}s, mipgap: {mipgap})...")
    start_time = time.time()
    try: prob.solve()
    except CplexError as e: return None, None, None, f"CPLEX Error: {e}"
    
    solve_time = time.time() - start_time
    print(f"Solver finished in {solve_time:.2f} seconds.")
    
    status_string = prob.solution.get_status_string()
    if prob.solution.get_status() in [101, 102]: # MIP Optimal or Feasible
        return np.array(prob.solution.get_values()), prob.solution.get_objective_value(), solve_time, status_string
    else:
        if 'infeasible' in status_string.lower():
            print("Solver status is infeasible. Attempting to find conflicting constraints...")
            try:
                # Use default list of constraints/bounds to check
                prob.conflict.refine(prob.conflict.all_constraints())
                print("\n--- CONFLICT REPORT ---")
                
                conflict_groups = prob.conflict.get_groups()
                
                if not conflict_groups:
                    print("No conflicts found by the refiner (or refiner failed).")
                
                for i, group in enumerate(conflict_groups):
                    print(f"Conflict Group {i+1} (Status: {prob.conflict.get_group_status(i)[1]}):")
                    group_members = group[1]

                    for conflict_member in group_members:
                        constraint_type = conflict_member[0]
                        constraint_index = conflict_member[1]
                        
                        if constraint_type == prob.conflict.constraint_type.linear:
                            name = prob.linear_constraints.get_names(constraint_index)
                            print(f"  - Conflicting Linear Constraint: {name}")
                        elif constraint_type == prob.conflict.constraint_type.lower_bound:
                            name = prob.variables.get_names(constraint_index)
                            val = prob.variables.get_lower_bounds(constraint_index)
                            print(f"  - Conflicting Lower Bound: {name} >= {val}")
                        elif constraint_type == prob.conflict.constraint_type.upper_bound:
                            name = prob.variables.get_names(constraint_index)
                            val = prob.variables.get_upper_bounds(constraint_index)
                            print(f"  - Conflicting Upper Bound: {name} <= {val}")
                
                print("-----------------------\n")
            except Exception as conf_e:
                print(f"Could not run conflict refiner: {conf_e}")
        return None, None, solve_time, status_string