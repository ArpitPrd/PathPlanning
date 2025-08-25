#!/usr/bin/env python3
"""
uav_milp_matched.py
-------------------
One-for-one transliteration of your MATLAB MILP:

Variables (vector S length = (1 + 2*N*T)*M, where M = Nx*Ny):
  - Block 1: q[p]                    indices: 0 .. M-1                     (continuous)
  - Block 2: X1[t,n,p] (sensing)     indices: M .. M + (N*T*M) - 1         (binary)
  - Block 3: X2[t,n,p] (position)    indices: M + (N*T*M) .. end           (binary)

Objective:
  Maximize sum of X2[t,n,p] over all t,n,p; zero contribution for sink cell.

Constraints:
  Aineq = [E; F; g; H; I; J; Q; W’], bineq as in MATLAB
  Aeq   = [K], beq = 0

Live CPLEX logging enabled (dynamic search, deterministic parallel).
"""

import sys
import time
import math
from typing import List, Tuple, Optional

import numpy as np

import cplex
from cplex.exceptions import CplexError


# ----------------------------
# Helpers
# ----------------------------

def lin_to_rc(idx: int, Ny: int) -> Tuple[int, int]:
    return idx // Ny, idx % Ny

def rc_to_lin(r: int, c: int, Ny: int) -> int:
    return r * Ny + c

def euclid2(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    dr = a[0] - b[0]
    dc = a[1] - b[1]
    return dr*dr + dc*dc

def grid_coords(Nx: int, Ny: int) -> np.ndarray:
    rr, cc = np.meshgrid(np.arange(Nx), np.arange(Ny), indexing="ij")
    return np.column_stack([rr.ravel(), cc.ravel()])  # shape (M,2)

def build_neighbors(Nx: int, Ny: int, R: float) -> List[List[int]]:
    """All cells within Euclidean radius R (including the center)."""
    G = grid_coords(Nx, Ny)
    M = G.shape[0]
    R2 = R*R
    neigh = [[] for _ in range(M)]
    for p in range(M):
        rp, cp = int(G[p,0]), int(G[p,1])
        rmin = max(0, int(math.floor(rp - R) - 1))
        rmax = min(Nx-1, int(math.ceil(rp + R) + 1))
        cmin = max(0, int(math.floor(cp - R) - 1))
        cmax = min(Ny-1, int(math.ceil(cp + R) + 1))
        for r in range(rmin, rmax+1):
            for c in range(cmin, cmax+1):
                if euclid2((rp,cp),(r,c)) <= R2 + 1e-9:
                    neigh[p].append(rc_to_lin(r,c,Ny))
    return neigh

def build_sink_neighbors(L: int, Nx: int, Ny: int, R: float) -> List[int]:
    """Neighbors of sink within R, excluding sink itself."""
    rp, cp = lin_to_rc(L, Ny)
    R2 = R*R
    out = []
    rmin = max(0, int(math.floor(rp - R) - 1))
    rmax = min(Nx-1, int(math.ceil(rp + R) + 1))
    cmin = max(0, int(math.floor(cp - R) - 1))
    cmax = min(Ny-1, int(math.ceil(cp + R) + 1))
    for r in range(rmin, rmax+1):
        for c in range(cmin, cmax+1):
            idx = rc_to_lin(r,c,Ny)
            if idx == L:
                continue
            if euclid2((rp,cp),(r,c)) <= R2 + 1e-9:
                out.append(idx)
    return out

# ----------------------------
# Indices for blocks
# ----------------------------

def idx_q(p: int) -> int:
    return p

def idx_X1(t: int, n: int, p: int, Nx: int, Ny: int, N: int, T: int) -> int:
    M = Nx*Ny
    return M + (t*N + n)*M + p  # middle block

def idx_X2(t: int, n: int, p: int, Nx: int, Ny: int, N: int, T: int) -> int:
    M = Nx*Ny
    return M + (N*T)*M + (t*N + n)*M + p  # last block

# ----------------------------
# Model assembly (MATLAB match)
# ----------------------------

def build_and_solve(Nx: int = 12, Ny: int = 12, N: int = 3, T: int = 10,
                    Rs: float = math.sqrt(2.0), Rc: float = 2.0*math.sqrt(2.0),
                    P_sink: int = 79, cr: float = 1.0, timelimit: float = 1800.0):
    """
    Returns (sol_vector, objective_value, prob).
    sol_vector is the full S of length (1+2*N*T)*Nx*Ny.
    """

    start_time = time.time()
    M = Nx*Ny
    G = grid_coords(Nx, Ny)
    L = P_sink                       # sink index in 0..M-1
    Cmax = M - 1                     # non-sink cell count

    # Neighbor sets
    Irc  = build_neighbors(Nx, Ny, Rc)    # communication/mobility neighbors (include self)
    Irs  = build_neighbors(Nx, Ny, Rs)    # sensing neighbors (include self)
    Irc_sink = build_sink_neighbors(L, Nx, Ny, Rc)

    # ---------------- Objective ff ----------------
    num_vars = (1 + 2*N*T)*M
    ff = np.zeros(num_vars, dtype=float)
    for t in range(T):
        for n in range(N):
            # set 1's on X2[t,n,:]
            base = idx_X2(t, n, 0, Nx, Ny, N, T)
            ff[base: base + M] = 1.0
            # zero sink contribution
            ff[idx_X2(t, n, L, Nx, Ny, N, T)] = 0.0

    # ---------------- Bounds & types ----------------
    lb = np.zeros(num_vars, dtype=float)
    ub = np.ones(num_vars, dtype=float)

    # EXACTLY like your reference: first M continuous, rest binary
    types = (['C'] * M) + (['B'] * (2*N*T*M))

    # ---------------- Build constraints ----------------
    A_rows: List[List[float]] = []
    senses: List[str] = []
    rhs: List[float] = []
    names: List[str] = []

    def add_row(name: str, row: np.ndarray, sense: str, rhs_val: float):
        # skip all-zero rows to avoid "0 = const" infeasibilities
        if np.any(row != 0.0):
            A_rows.append(row.tolist())
            senses.append(sense)
            rhs.append(float(rhs_val))
            names.append(name)

    # ---- E: single-UAV-at-one-pt (sum_n X2[t,*,p] <= 1), skip sink p=L
    for t in range(T):
        for p in range(M):
            if p == L:
                continue
            row = np.zeros(num_vars, dtype=float)
            for n in range(N):
                row[idx_X2(t, n, p, Nx, Ny, N, T)] = 1.0
            add_row(f"E_t{t}_p{p}", row, 'L', 1.0)

    # ---- F: one-UAV-at-single-pt (sum_p X2[t,n,p] = 1), with sink entry forced 0
    for t in range(T):
        for n in range(N):
            row = np.zeros(num_vars, dtype=float)
            # sum over all p
            base = idx_X2(t, n, 0, Nx, Ny, N, T)
            row[base: base+M] = 1.0
            # zero at sink (same as MATLAB setting that coefficient to 0)
            row[idx_X2(t, n, L, Nx, Ny, N, T)] = 0.0
            add_row(f"F_t{t}_n{n}", row, 'E', 1.0)

    # ---- g: sink connectivity (sum_{n,i∈Irc_sink} -X2[t,n,i] <= -1)
    for t in range(T):
        if len(Irc_sink) == 0:
            continue
        row = np.zeros(num_vars, dtype=float)
        for n in range(N):
            for i in Irc_sink:
                row[idx_X2(t, n, i, Nx, Ny, N, T)] = -1.0
        add_row(f"g_sink_t{t}", row, 'L', -1.0)

    # ---- H: inter-UAV (for n>=1): X2[t,n,p] - sum_{i∈Irc[p]} X2[t,n-1,i] <= 0, skip sink
    for t in range(T):
        for n in range(1, N):
            for p in range(M):
                if p == L:
                    continue
                row = np.zeros(num_vars, dtype=float)
                row[idx_X2(t, n, p, Nx, Ny, N, T)] = 1.0
                for i in Irc[p]:
                    row[idx_X2(t, n-1, i, Nx, Ny, N, T)] -= 1.0
                add_row(f"H_t{t}_n{n}_p{p}", row, 'L', 0.0)

    # ---- I: mobility: X2[t+1,n,p] - sum_{i∈Irc[p]} X2[t,n,i] <= 0, skip sink
    for t in range(T-1):
        for n in range(N):
            for p in range(M):
                if p == L:
                    continue
                row = np.zeros(num_vars, dtype=float)
                row[idx_X2(t+1, n, p, Nx, Ny, N, T)] = 1.0
                for i in Irc[p]:
                    row[idx_X2(t, n, i, Nx, Ny, N, T)] -= 1.0
                add_row(f"I_t{t}_n{n}_p{p}", row, 'L', 0.0)

    # ---- K: sensing equalities: X1[t,n,p] - sum_{i∈Irs[p]} X2[t,n,i] = 0, skip sink
    for t in range(T):
        for n in range(N):
            for p in range(M):
                if p == L:
                    continue
                row = np.zeros(num_vars, dtype=float)
                row[idx_X1(t, n, p, Nx, Ny, N, T)] = 1.0
                for i in Irs[p]:
                    row[idx_X2(t, n, i, Nx, Ny, N, T)] -= 1.0
                add_row(f"K_t{t}_n{n}_p{p}", row, 'E', 0.0)

    # ---- J: -q[p] + X1[t,n,p] <= 0  (sink terms forced to 0)
    for t in range(T):
        for n in range(N):
            for p in range(M):
                if p == L:
                    continue
                row = np.zeros(num_vars, dtype=float)
                row[idx_q(p)] = -1.0
                row[idx_X1(t, n, p, Nx, Ny, N, T)] = 1.0
                add_row(f"J_t{t}_n{n}_p{p}", row, 'L', 0.0)

    # ---- Q: q[p] - sum_{t,n} X1[t,n,p] <= 0 (skip sink)
    for p in range(M):
        if p == L:
            continue
        row = np.zeros(num_vars, dtype=float)
        row[idx_q(p)] = 1.0
        for t in range(T):
            for n in range(N):
                row[idx_X1(t, n, p, Nx, Ny, N, T)] -= 1.0
        add_row(f"Q_p{p}", row, 'L', 0.0)

    # ---- W': sum_p (-q[p]) <= -cr * Cmax (sink has 0 coeff)
    row = np.zeros(num_vars, dtype=float)
    for p in range(M):
        if p == L:
            continue
        row[idx_q(p)] = -1.0
    add_row("W_total", row, 'L', -cr * Cmax)

    # --- Split Aeq/Beq vs Aineq/Bineq exactly like MATLAB ---
    # K is equality; others are inequality
    Aeq_rows, Aeq_names = [], []
    beq = []
    Aineq_rows, Aineq_names = [], []
    bineq = []

    # Partition rows by their sense
    for row, s, r, nm in zip(A_rows, senses, rhs, names):
        if s == 'E':
            Aeq_rows.append(row); beq.append(r); Aeq_names.append(nm)
        else:
            Aineq_rows.append(row); bineq.append(r); Aineq_names.append(nm)

    # --------------- Build CPLEX ---------------
    prob = cplex.Cplex()

    # logs → terminal, dynamic search + deterministic parallel
    prob.set_log_stream(sys.stdout)
    prob.set_results_stream(sys.stdout)
    prob.set_warning_stream(sys.stderr)
    prob.set_error_stream(sys.stderr)

    prob.parameters.read.datacheck.set(1)
    prob.parameters.mip.display.set(2)            # node table
    prob.parameters.mip.strategy.search.set(0)    # dynamic search (matches your sample log)
    prob.parameters.parallel.set(1)               # deterministic parallel
    # prob.parameters.threads.set(8)              # optional: pin thread count

    prob.parameters.mip.tolerances.mipgap.set(0.0)
    prob.parameters.timelimit.set(float(timelimit))
    prob.objective.set_sense(prob.objective.sense.maximize)

    # Variables
    prob.variables.add(
        obj=ff.tolist(),
        lb=lb.tolist(),
        ub=ub.tolist(),
        types="".join(types),
        names=[f"S_{i}" for i in range(num_vars)]
    )

    # Inequalities
    lin_expr_ineq = []
    for row in Aineq_rows:
        ind = [i for i, v in enumerate(row) if v != 0.0]
        val = [row[i] for i in ind]
        lin_expr_ineq.append(cplex.SparsePair(ind=ind, val=val))
    if lin_expr_ineq:
        prob.linear_constraints.add(
            lin_expr=lin_expr_ineq,
            senses="".join(['L'] * len(lin_expr_ineq)),   # all are ≤ in our packing
            rhs=bineq,
            names=Aineq_names
        )

    # Equalities
    lin_expr_eq = []
    for row in Aeq_rows:
        ind = [i for i, v in enumerate(row) if v != 0.0]
        val = [row[i] for i in ind]
        lin_expr_eq.append(cplex.SparsePair(ind=ind, val=val))
    if lin_expr_eq:
        prob.linear_constraints.add(
            lin_expr=lin_expr_eq,
            senses="".join(['E'] * len(lin_expr_eq)),
            rhs=beq,
            names=Aeq_names
        )

    # quick model stats
    nz_ineq = sum(len(sp.ind) for sp in lin_expr_ineq) if lin_expr_ineq else 0
    nz_eq   = sum(len(sp.ind) for sp in lin_expr_eq) if lin_expr_eq else 0
    print(f"Model built. rows={len(Aineq_rows)+len(Aeq_rows)}, cols={num_vars}, nz={nz_ineq+nz_eq}")

    try:
        prob.write("uav_model_matched.lp")
        print("Wrote model to uav_model_matched.lp")
    except Exception as e:
        print("LP write failed:", e)

    # --------------- Solve ---------------
    print("Starting CPLEX solve...")
    try:
        prob.solve()
    except CplexError as exc:
        print("CPLEX solve error:", exc)
        return None, None, prob

    solve_time = time.time() - start_time
    print(f"Solve time: {solve_time:.2f} seconds")
    status = prob.solution.get_status()
    print(f"CPLEX status: {status} ({prob.solution.get_status_string()})")

    if not prob.solution.is_primal_feasible():
        print("No primal-feasible solution.")
        return None, None, prob

    S = np.array(prob.solution.get_values(), dtype=float)
    obj = prob.solution.get_objective_value()
    try:
        gap = prob.solution.MIP.get_mip_relative_gap()
        print(f"MIP gap: {gap:.6f}")
    except Exception:
        pass

    print(f"Objective value: {obj:.6f}")
    return S, obj, prob

# ----------------------------
# Utilities: Extract paths from X2
# ----------------------------
def extract_paths_from_X2(S: np.ndarray, Nx: int, Ny: int, N: int, T: int) -> List[List[int]]:
    """Read paths from the X2 block (last N*T*M slice)."""
    M = Nx*Ny
    offset = M + (N*T)*M
    X2 = S[offset: offset + (N*T*M)].reshape(T, N, M)
    paths = []
    for n in range(N):
        seq = [int(np.argmax(X2[t, n, :])) for t in range(T)]
        paths.append(seq)
    return paths

def plot_paths(paths: List[List[int]], Nx: int, Ny: int, sink_index: int):
    try:
        import matplotlib.pyplot as plt
    except Exception:
        print("[plot] matplotlib not available; skipping plot.")
        return
    plt.figure(figsize=(7,7))
    for r in range(Nx+1):
        plt.plot([-0.5, Ny-0.5],[r-0.5, r-0.5], linewidth=0.5)
    for c in range(Ny+1):
        plt.plot([c-0.5, c-0.5],[-0.5, Nx-0.5], linewidth=0.5)
    for n, seq in enumerate(paths):
        xs, ys = [], []
        for p in seq:
            rr, cc = lin_to_rc(p, Ny)
            xs.append(cc); ys.append(rr)
        plt.plot(xs, ys, marker='o', label=f"UAV {n}")
    sr, sc = lin_to_rc(sink_index, Ny)
    plt.scatter([sc],[sr], marker='*', s=200, zorder=5, label='Sink')
    plt.xlim([-0.5, Ny-0.5]); plt.ylim([-0.5, Nx-0.5])
    plt.gca().invert_yaxis(); plt.gca().set_aspect('equal', adjustable='box')
    plt.legend(); plt.tight_layout(); plt.title("UAV Paths (X2)"); plt.show()

# ----------------------------
# Main
# ----------------------------

if __name__ == "__main__":
    Nx, Ny, N, T = 12, 12, 3, 10
    Rs = math.sqrt(2.0)
    Rc = 2.0 * math.sqrt(2.0)
    P_sink = 79
    S, obj, prob = build_and_solve(Nx=Nx, Ny=Ny, N=N, T=T, Rs=Rs, Rc=Rc, P_sink=P_sink, cr=1.0, timelimit=1800.0)
    if S is not None:
        paths = extract_paths_from_X2(S, Nx, Ny, N, T)
        print("\nRecovered UAV paths (from X2):")
        for i, seq in enumerate(paths):
            print(f"UAV {i}: {seq}")
        # Optional plot:
        # plot_paths(paths, Nx, Ny, P_sink)
