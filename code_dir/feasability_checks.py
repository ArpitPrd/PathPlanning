import numpy as np
import cplex
from cplex.exceptions import CplexError
import time
import json
import sys


def quick_feasibility_checks(vh, Irs, model_cfg):
    G = vh.num_grid_cells
    N_T = vh.N * vh.T
    coverable = {i for i in range(G) if len(Irs[i]) > 0}
    num_coverable = len(coverable)
    C_min = model_cfg.get('C_min', None)*(G/100) if model_cfg.get('C_min', None) is not None else None

    print("=== QUICK CHECKS ===")
    print("Grid cells (G):", G)
    print("UAV-time slots (N*T):", N_T)
    print("Cells with non-empty Irs (coverable):", num_coverable)
    print("Coverable cell indices (first 20):", sorted(list(coverable))[:20])
    if C_min is not None:
        print("Requested C_min:", C_min)
        print("Trivial upper bound on coverage:", min(G, N_T, num_coverable))
        if C_min > min(G, N_T, num_coverable):
            print(">>> TRIVIAL INFEASIBLE: C_min exceeds obvious upper bound.")
    print("====================\n")


def inspect_Irs_Irc(Irs, Irc):
    print("=== Irs / Irc INSPECTION ===")
    empty_Irs = [i for i in range(len(Irs)) if len(Irs[i]) == 0]
    print("Cells with empty Irs:", len(empty_Irs), empty_Irs[:20])
    # Check Irc symmetry or invalid indices
    bad_ircs = []
    for i, neigh in enumerate(Irc):
        for p in neigh:
            if p < 0 or p >= len(Irc):
                bad_ircs.append((i,p))
    print("Invalid indices in Irc (if any):", bad_ircs[:20])
    print("===========================\n")


def check_variable_bounds_and_types(vh, lb, ub, ctype):
    # Inspect c_i indices
    c_indices = [vh.c_i(i) for i in range(vh.num_grid_cells)]
    lb_c = [lb[idx] for idx in c_indices]
    ub_c = [ub[idx] for idx in c_indices]
    types_c = [ctype[idx] for idx in c_indices]
    print("=== c_i variable bounds / types ===")
    print("Unique lower bounds for c_i:", sorted(set(lb_c)))
    print("Unique upper bounds for c_i:", sorted(set(ub_c)))
    print("Example c_i lb/ub/type for first 20:")
    for i, idx in enumerate(c_indices[:20]):
        print(f"  c_{i}: idx={idx}, lb={lb[idx]}, ub={ub[idx]}, type={ctype[idx]}")
    print("===================================\n")


def compute_max_coverage_relax(vh, A_eq, b_eq, A_ineq, b_ineq, senses, lb, ub, time_limit=30):
    obj = np.zeros(vh.total_vars)
    for i in range(vh.num_grid_cells):
        obj[vh.c_i(i)] = 1.0
    ctype = 'C' * vh.total_vars
    prob = cplex.Cplex()
    prob.set_problem_name("max_coverage_relax")
    prob.objective.set_sense(prob.objective.sense.maximize)
    prob.variables.add(obj=obj.tolist(), lb=lb, ub=ub, types=ctype,
                       names=[f"var_{i}" for i in range(vh.total_vars)])
    if A_eq.shape[0] > 0:
        prob.linear_constraints.add(
            lin_expr=[cplex.SparsePair(ind=r.nonzero()[0].tolist(), val=r[r.nonzero()].tolist()) for r in A_eq],
            senses=['E'] * A_eq.shape[0], rhs=b_eq.tolist()
        )
    if A_ineq.shape[0] > 0:
        prob.linear_constraints.add(
            lin_expr=[cplex.SparsePair(ind=r.nonzero()[0].tolist(), val=r[r.nonzero()].tolist()) for r in A_ineq],
            senses=senses, rhs=b_ineq.tolist()
        )
    prob.parameters.timelimit.set(time_limit)
    try:
        prob.solve()
    except CplexError as e:
        return None, f"CPLEX Error: {e}"
    status = prob.solution.get_status_string()
    if prob.solution.get_status() in [101, 102, 1, 2]:
        return prob.solution.get_objective_value(), status
    else:
        return None, status

def run_conflict_refiner_and_print(prob):
    # prob is a cplex.Cplex() already with variables and constraints added
    try:
        # Build list: all linear constraints and variable bounds
        all_cons = []
        for i in range(prob.linear_constraints.get_num()):
            all_cons.append((prob.conflict.constraint_type.linear, i))
        for i in range(prob.variables.get_num()):
            all_cons.append((prob.conflict.constraint_type.upper_bound, i))
            all_cons.append((prob.conflict.constraint_type.lower_bound, i))

        prob.conflict.refine((1.0, all_cons))
        groups = prob.conflict.get_groups()
        print("\n--- CONFLICT REPORT ---")
        if not groups:
            print("No conflicts found by refiner.")
        for gi, group in enumerate(groups):
            print(f"Conflict Group {gi+1}:")
            for member in group[1]:
                ctype, idx = member
                if ctype == prob.conflict.constraint_type.linear:
                    name = prob.linear_constraints.get_names(idx)
                    print(f"  Linear constraint: index={idx}, name={name}")
                elif ctype == prob.conflict.constraint_type.lower_bound:
                    name = prob.variables.get_names(idx)
                    lb = prob.variables.get_lower_bounds(idx)
                    print(f"  Variable lower bound: var_index={idx}, name={name}, lb={lb}")
                elif ctype == prob.conflict.constraint_type.upper_bound:
                    name = prob.variables.get_names(idx)
                    ub = prob.variables.get_upper_bounds(idx)
                    print(f"  Variable upper bound: var_index={idx}, name={name}, ub={ub}")
        print("-----------------------\n")
    except Exception as e:
        print("Conflict refiner failed:", e)


def check_bigM_consistency(vh, b_mov, b_steady, b_full):
    # compute maximum possible energy in one time step for a UAV (worst-case)
    max_energy_per_step = 0
    for i in range(vh.num_grid_cells):
        for j in range(vh.num_grid_cells):
            cost = b_steady if i == j else b_mov
            max_energy_per_step = max(max_energy_per_step, cost)
    # but if energy is sum across all i->j possible in formulation, check sum
    total_possible_energy_vars = 0
    for i in range(vh.num_grid_cells):
        for j in range(vh.num_grid_cells):
            cost = b_steady if i==j else b_mov
            total_possible_energy_vars += cost
    print("b_full:", b_full)
    print("max single m(i,j) cost:", max_energy_per_step)
    print("sum of all m(i,j) costs (per UAV-step if your formulation sums them):", total_possible_energy_vars)
    print("If M < any of these relevant energy sums, big-M modeling may be invalid.\n")



