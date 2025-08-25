import numpy as np

def solve_with_cplex(model):
    try:
        import cplex
    except Exception as e:
        return None, str(e)

    Z1 = model['Z1']; A = model['A']; b = model['b']; senses = model['senses']
    lb = model['lb']; ub = model['ub']; vtypes = model['vtypes']
    total = model['total']

    prob = cplex.Cplex()
    prob.objective.set_sense(prob.objective.sense.minimize)
    prob.variables.add(obj=Z1.tolist(), lb=lb.tolist(), ub=ub.tolist(),
                       types="".join(vtypes.tolist()),
                       names=[f"x_{i}" for i in range(total)])

    cplex_senses = "".join(['L' if s=='L' else ('E' if s=='E' else 'G') for s in senses.tolist()])
    rows = []
    for i in range(A.shape[0]):
        idx = A[i,:].nonzero()[0]
        val = A[i, idx].tolist()
        rows.append(cplex.SparsePair(ind=[f"x_{k}" for k in idx], val=val))
    prob.linear_constraints.add(lin_expr=rows, senses=cplex_senses, rhs=b.tolist())
    prob.parameters.mip.tolerances.mipgap.set(0)
    prob.solve()

    status = prob.solution.get_status_string()
    obj = prob.solution.get_objective_value() if prob.solution.is_primal_feasible() else None
    x = np.array(prob.solution.get_values()) if obj is not None else None
    return dict(status=status, obj=obj, x=x), None


def solve_with_pulp(model):
    from pulp import LpProblem, LpVariable, LpMinimize, LpBinary, LpContinuous, lpSum, PULP_CBC_CMD, LpStatus
    Z1 = model['Z1']; A = model['A']; b = model['b']; senses = model['senses']
    lb = model['lb']; ub = model['ub']; vtypes = model['vtypes']
    total = model['total']

    m = LpProblem("milp", LpMinimize)
    vars_ = []
    for i in range(total):
        low, up = float(lb[i]), float(ub[i])
        if vtypes[i] == 'B':
            v = LpVariable(f"x_{i}", lowBound=0, upBound=1, cat=LpBinary)
        else:
            v = LpVariable(f"x_{i}", lowBound=low, upBound=up, cat=LpContinuous)
        vars_.append(v)
    m += lpSum([Z1[i]*vars_[i] for i in range(total)])

    for i in range(A.shape[0]):
        expr = lpSum([A[i,j]*vars_[j] for j in range(total) if A[i,j] != 0.0])
        if senses[i] == 'L':
            m += (expr <= b[i])
        elif senses[i] == 'E':
            m += (expr == b[i])
        else:
            m += (expr >= b[i])

    solver = PULP_CBC_CMD(msg=1)
    m.solve(solver)
    status = LpStatus[m.status]
    x = np.array([v.value() for v in vars_], dtype=float)
    obj = float(m.objective.value())
    return dict(status=status, obj=obj, x=x), None


def solve_with_ortools(model):
    from ortools.linear_solver import pywraplp
    Z1 = model['Z1']; A = model['A']; b = model['b']; senses = model['senses']
    lb = model['lb']; ub = model['ub']; vtypes = model['vtypes']
    total = model['total']

    solver = pywraplp.Solver.CreateSolver("CBC")
    if solver is None:
        return None, "OR-Tools CBC solver not available."

    vars_ = []
    for i in range(total):
        if vtypes[i] == 'B':
            v = solver.BoolVar(f"x_{i}")
        else:
            v = solver.NumVar(lb[i], ub[i], f"x_{i}")
        vars_.append(v)
    objective = solver.Objective()
    for i in range(total):
        objective.SetCoefficient(vars_[i], float(Z1[i]))
    objective.SetMinimization()

    for i in range(A.shape[0]):
        row = A[i,:]
        idx = row.nonzero()[0]
        expr = solver.Sum([float(row[j]) * vars_[j] for j in idx])
        if senses[i] == 'L':
            solver.Add(expr <= float(b[i]))
        elif senses[i] == 'E':
            solver.Add(expr == float(b[i]))
        else:
            solver.Add(expr >= float(b[i]))

    status = solver.Solve()
    if status not in (pywraplp.Solver.OPTIMAL, pywraplp.Solver.FEASIBLE):
        return dict(status="INFEASIBLE_OR_ERROR", obj=None, x=None), None
    x = np.array([vars_[i].solution_value() for i in range(total)], dtype=float)
    obj = objective.Value()
    return dict(status="OPTIMAL_OR_FEASIBLE", obj=obj, x=x), None


def solve_with_pyomo(model, solver_name="highs"):
    import pyomo.environ as pyo
    Z1 = model['Z1']; A = model['A']; b = model['b']; senses = model['senses']
    lb = model['lb']; ub = model['ub']; vtypes = model['vtypes']
    total = model['total']

    m = pyo.ConcreteModel()
    m.I = pyo.RangeSet(0, total-1)

    def var_domain(m, i):
        return pyo.Binary if vtypes[i] == 'B' else pyo.Reals
    m.x = pyo.Var(m.I, domain=var_domain)

    # bounds
    for i in range(total):
        m.x[i].setlb(float(lb[i]))
        m.x[i].setub(float(ub[i]))

    m.obj = pyo.Objective(expr=sum(float(Z1[i])*m.x[i] for i in m.I), sense=pyo.minimize)
    m.J = pyo.RangeSet(0, A.shape[0]-1)

    def con_rule(m, j):
        expr = sum(float(A[j,i])*m.x[i] for i in m.I)
        if senses[j] == 'L':
            return expr <= float(b[j])
        elif senses[j] == 'E':
            return expr == float(b[j])
        else:
            return expr >= float(b[j])
    m.con = pyo.Constraint(m.J, rule=con_rule)

    solver = pyo.SolverFactory(solver_name)
    res = solver.solve(m, tee=False)
    if (res.solver.status != pyo.SolverStatus.ok) or (res.solver.termination_condition not in [pyo.TerminationCondition.optimal, pyo.TerminationCondition.feasible]):
        return dict(status=str(res.solver), obj=None, x=None), None
    x = np.array([pyo.value(m.x[i]) for i in m.I], dtype=float)
    obj = pyo.value(m.obj)
    return dict(status=str(res.solver.termination_condition), obj=obj, x=x), None


def pick_and_solve(model, preferred="cplex"):
    order = [preferred, "ortools", "pulp", "pyomo"]
    for name in order:
        try:
            # if name == "cplex":
            #     res, err = solve_with_cplex(model)
            # elif name == "pulp":
            #     res, err = solve_with_pulp(model)
            # elif name == "ortools":
            #     res, err = solve_with_ortools(model)
            # elif name == "pyomo":
            #     res, err = solve_with_pyomo(model, solver_name="highs")
            # else:
            #     continue
            # if res is not None and res.get("x") is not None:
            #     return res, name
            res, err = solve_with_cplex(model)
        except Exception:
            continue
    return dict(status="No solver succeeded", obj=None, x=None), None
