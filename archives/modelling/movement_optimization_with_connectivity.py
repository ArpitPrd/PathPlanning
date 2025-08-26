import numpy as np
from model.problem_builder import build_movement_problem
from model.solvers import pick_and_solve
from model.postprocess import extract_paths, path_total_distance

def main():
    # User-defined input (matches your MATLAB script)
    Rs = 2**0.5
    Rc = 2*Rs
    cr = 1.0
    x = list(range(1, 12+1))
    y = list(range(1, 12+1))
    P_sink = 79
    Obs = []  # e.g., [[2,2],[2,3],[3,2],[3,3]]
    N = 3
    T = 10

    model = build_movement_problem(Rs, Rc, x, y, P_sink, N, T, cr=cr, Obs=np.array(Obs, dtype=int) if len(Obs)>0 else None)
    res, used = pick_and_solve(model, preferred="cplex")
    print(f"Solver used: {used}, status: {res['status']}")
    if res['x'] is None:
        print("No solution found.")
        return

    xsol = res['x']
    cov_path = extract_paths(model, xsol)
    path_cost = path_total_distance(cov_path, model['sz'])
    print("path_cost:", path_cost)
    print("cov_path (indices):")
    print(cov_path)

if __name__ == "__main__":
    main()
