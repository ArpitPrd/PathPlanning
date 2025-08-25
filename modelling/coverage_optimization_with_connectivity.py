import numpy as np
from model.problem_builder import build_coverage_problem
from model.solvers import pick_and_solve
from model.postprocess import extract_paths, coverage_percent

def main():
    # User-defined input (matches your MATLAB script)
    Rs = 1.0
    Rc = 2*Rs
    x = list(range(1, 6+1))
    y = list(range(1, 6+1))
    P_sink = 1
    Obs = []  # e.g., [[3,3],[3,4],[4,3],[4,4]]
    N = 3
    T = 4

    model = build_coverage_problem(Rs, Rc, x, y, P_sink, N, T, Obs=np.array(Obs, dtype=int) if len(Obs)>0 else None)
    res, used = pick_and_solve(model, preferred="cplex")
    print(f"Solver used: {used}, status: {res['status']}")
    if res['x'] is None:
        print("No solution found.")
        return

    xsol = res['x']
    cov_path = extract_paths(model, xsol)
    Cov_Percent = coverage_percent(res['obj'], model['Cmax'])
    print("Coverage percent:", Cov_Percent)
    print("cov_path (indices):")
    print(cov_path)

if __name__ == "__main__":
    main()
