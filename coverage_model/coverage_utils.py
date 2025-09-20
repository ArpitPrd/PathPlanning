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
# VARIABLE INDEXING HELPER CLASS (No changes)
# ==============================================================================
class VarHelper:
    """Manages indices of decision variables based on eq.pdf formulation."""
    def __init__(self, N, T, row_size, col_size):
        self.N, self.T = N, T
        self.rs, self.cs = row_size, col_size
        self.num_grid_cells = row_size * col_size
        self.start_z = 0
        self.start_c_i = self.start_z + self.num_grid_cells * N * T
        self.start_c_in_k = self.start_c_i + self.num_grid_cells
        self.start_m = self.start_c_in_k + self.num_grid_cells * N * T
        self.start_x_charge = self.start_m + self.num_grid_cells**2 * N * (T - 1)
        self.start_b = self.start_x_charge + N * T
        self.total_vars = self.start_b + N * T

    def z(self, t, n, i): return self.start_z + t*self.N*self.num_grid_cells + n*self.num_grid_cells + i
    def c_i(self, i): return self.start_c_i + i
    def c_in_k(self, t, n, i): return self.start_c_in_k + t*self.N*self.num_grid_cells + n*self.num_grid_cells + i
    def m(self, t, n, i, j): return self.start_m + t*self.N*self.num_grid_cells**2 + n*self.num_grid_cells**2 + i*self.num_grid_cells + j
    def x_charge(self, t, n): return self.start_x_charge + t * self.N + n
    def b(self, t, n): return self.start_b + t * self.N + n

# ==============================================================================
# OBJECTIVE AND CONSTRAINTS (Updated for Toggling)
# ==============================================================================
def setup_obj(vh):
    f = np.zeros(vh.total_vars)
    for i in range(vh.num_grid_cells):
        f[vh.c_i(i)] = 1.0
    return f

def position_and_collision_constraints(vh, P_sink, cfg):
    # Eq 2: Unique position
    eq2_rows = []
    if cfg.get("eq2_unique_position", True):
        for t in range(vh.T):
            for n in range(vh.N):
                row = np.zeros(vh.total_vars)
                for i in range(vh.num_grid_cells):
                    row[vh.z(t, n, i)] = 1
                eq2_rows.append(row)
    
    # Eq 3: Collision avoidance
    eq3_rows = []
    if cfg.get("eq3_collision_avoidance", True):
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
    if cfg.get("eq4_sink_connectivity", True):
        for t in range(vh.T):
            row = np.zeros(vh.total_vars)
            for n in range(vh.N):
                for p in Irc_sink:
                    row[vh.z(t, n, p)] = 1
            eq4_rows.append(row)

    # Eq 5: Inter-UAV link
    eq5_rows = []
    if cfg.get("eq5_inter_uav_link", True):
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
    if cfg.get("eq6_mobility_rule", True):
        for t in range(vh.T - 1):
            for n in range(vh.N):
                for i in range(vh.num_grid_cells):
                    row = np.zeros(vh.total_vars)
                    row[vh.z(t + 1, n, i)] = 1
                    for p in set(Irc[i] + [i]):
                        row[vh.z(t, n, p)] = -1
                    eq6_rows.append(row)

    # Eqs 7a, 7b, 7c: Movement definition (required for battery)
    eq7a, eq7b, eq7c = [], [], []
    if cfg.get("eq7_movement_definition", False):
        for t in range(vh.T - 1):
            for n in range(vh.N):
                for i in range(vh.num_grid_cells):
                    for j in range(vh.num_grid_cells):
                        r7a = np.zeros(vh.total_vars); r7a[vh.m(t,n,i,j)]=1; r7a[vh.z(t,n,i)]=-1; eq7a.append(r7a)
                        r7b = np.zeros(vh.total_vars); r7b[vh.m(t,n,i,j)]=1; r7b[vh.z(t+1,n,j)]=-1; eq7b.append(r7b)
                        r7c = np.zeros(vh.total_vars); r7c[vh.m(t,n,i,j)]=-1; r7c[vh.z(t,n,i)]=1; r7c[vh.z(t+1,n,j)]=1; eq7c.append(r7c)
    
    return np.array(eq6_rows), np.array(eq7a), np.array(eq7b), np.array(eq7c)

def battery_constraints(vh, b_mov, b_steady, b_full, P_sink, initial_battery, cfg):
    any_battery_logic = (
        cfg.get("eq8_charging_location", False) or
        cfg.get("eq9_battery_discharge", False) or
        cfg.get("eq10_battery_charge", False)
    )
    
    empty = np.empty((0, vh.total_vars))
    results = {
        'charge_loc': empty, 'discharge_leq': empty, 'discharge_geq': empty,
        'charge_leq': empty, 'rhs_charge_leq': np.array([]),
        'charge_geq': empty, 'rhs_charge_geq': np.array([]),
        'initial': empty, 'rhs_initial': np.array([])
    }
    
    if not any_battery_logic:
        return results
    
    M = b_full
    eq8, eq9a, eq9b, eq10a, eq10b = [], [], [], [], []

    for t in range(vh.T - 1):
        for n in range(vh.N):
            # Eq 8: Charging location
            if cfg.get("eq8_charging_location", False):
                row8 = np.zeros(vh.total_vars); row8[vh.x_charge(t,n)]=1; row8[vh.z(t,n,P_sink)]=-1; eq8.append(row8)
            
            # Eq 9: Battery discharge
            if cfg.get("eq9_battery_discharge", False):
                energy_consumed = np.zeros(vh.total_vars)
                for i in range(vh.num_grid_cells):
                    for j in range(vh.num_grid_cells):
                        cost = b_steady if i == j else b_mov
                        energy_consumed[vh.m(t, n, i, j)] = cost
                r9a = -np.copy(energy_consumed); r9a[vh.b(t+1,n)]=1; r9a[vh.b(t,n)]=-1; r9a[vh.x_charge(t,n)]=-M; eq9a.append(r9a)
                r9b = np.copy(energy_consumed); r9b[vh.b(t+1,n)]=-1; r9b[vh.b(t,n)]=1; r9b[vh.x_charge(t,n)]=-M; eq9b.append(r9b)
            
            # Eq 10: Battery charge
            if cfg.get("eq10_battery_charge", False):
                r10a = np.zeros(vh.total_vars); r10a[vh.b(t+1,n)]=1; r10a[vh.x_charge(t,n)]=M; eq10a.append(r10a)
                r10b = np.zeros(vh.total_vars); r10b[vh.b(t+1,n)]=-1; r10b[vh.x_charge(t,n)]=M; eq10b.append(r10b)

    results['charge_loc'] = np.array(eq8)
    results['discharge_leq'] = np.array(eq9a)
    results['discharge_geq'] = np.array(eq9b)
    results['charge_leq'] = np.array(eq10a)
    results['rhs_charge_leq'] = np.full(len(eq10a), b_full + M)
    results['charge_geq'] = np.array(eq10b)
    results['rhs_charge_geq'] = np.full(len(eq10b), -b_full + M)

    # Initial battery state is mandatory if any other battery constraint is active
    B_initial = np.zeros((vh.N, vh.total_vars))
    rhs_initial = np.full(vh.N, initial_battery)
    for n in range(vh.N): B_initial[n, vh.b(0, n)] = 1
    results['initial'] = B_initial
    results['rhs_initial'] = rhs_initial
            
    return results

def cell_coverage_constraints(vh, Irs, cfg):
    # Eq 13: Local coverage definition
    eq13 = []
    if cfg.get("eq13_local_coverage", True):
        for t in range(vh.T):
            for n in range(vh.N):
                for i in range(vh.num_grid_cells):
                    row = np.zeros(vh.total_vars)
                    row[vh.c_in_k(t, n, i)] = 1
                    for p in Irs[i]:
                        row[vh.z(t, n, p)] = -1
                    eq13.append(row)

    # Eq 14 & 15: Global coverage mapping
    eq14, eq15 = [], []
    for i in range(vh.num_grid_cells):
        r15 = np.zeros(vh.total_vars)
        if cfg.get("eq15_global_coverage", True):
            r15[vh.c_i(i)] = 1
            
        for t in range(vh.T):
            for n in range(vh.N):
                if cfg.get("eq14_global_mapping", True):
                    r14 = np.zeros(vh.total_vars); r14[vh.c_in_k(t,n,i)]=1; r14[vh.c_i(i)]=-1; eq14.append(r14)
                if cfg.get("eq15_global_coverage", True):
                    r15[vh.c_in_k(t, n, i)] = -1
        
        if cfg.get("eq15_global_coverage", True):
            eq15.append(r15)
        
    return np.array(eq13), np.array(eq14), np.array(eq15)

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

def cplex_solver(prob_name, objective, lb, ub, ctype, A_eq, b_eq, A_ineq, b_ineq, senses, time_limit, mipgap):
    prob = cplex.Cplex()
    prob.set_problem_name(prob_name)
    prob.objective.set_sense(prob.objective.sense.maximize)
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
    prob.set_log_stream(None); prob.set_error_stream(None); prob.set_warning_stream(None); prob.set_results_stream(None)

    print(f"Starting CPLEX solver (timelimit: {time_limit}s, mipgap: {mipgap})...")
    start_time = time.time()
    try: prob.solve()
    except CplexError as e: return None, None, None, f"CPLEX Error: {e}"
    
    solve_time = time.time() - start_time
    print(f"Solver finished in {solve_time:.2f} seconds.")
    
    status_string = prob.solution.get_status_string()
    if prob.solution.get_status() in [101, 102]:
        return np.array(prob.solution.get_values()), prob.solution.get_objective_value(), solve_time, status_string
    else:
        if 'infeasible' in status_string:
            print("Solver status is infeasible. Attempting to find conflicting constraints...")
            try:
                all_constraints_to_check = []
                for i in range(prob.linear_constraints.get_num()):
                    all_constraints_to_check.append((prob.conflict.constraint_type.linear, i))
                for i in range(prob.variables.get_num()):
                    all_constraints_to_check.append((prob.conflict.constraint_type.upper_bound, i))
                    all_constraints_to_check.append((prob.conflict.constraint_type.lower_bound, i))

                prob.conflict.refine((1.0, all_constraints_to_check))
                
                print("\n--- CONFLICT REPORT ---")
                
                conflict_groups = prob.conflict.get_groups()
                
                if not conflict_groups:
                    print("No conflicts found by the refiner.")
                
                for i, group in enumerate(conflict_groups):
                    print(f"Conflict Group {i+1}:")
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