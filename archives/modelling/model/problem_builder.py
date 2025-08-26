import numpy as np
from .grid_utils import build_grid, sub2ind, sink_pos_from_index, compute_sets

def build_common_data(x, y, P_sink, Obs=None):
    """
    Builds TG (all grid points as [row, col]),
    G (filtered by obstacles or identical to TG),
    G1 (MATLAB-style linear indices of points in G),
    sz (Nx, Ny),
    Cmax (# of coverable cells excluding sink and obstacles).
    P_sink must be a MATLAB-style linear index (1-based).
    """
    TG, sz = build_grid(x, y)
    Nx, Ny = sz

    if Obs is not None and len(Obs) > 0:
        # Filter TG by removing obstacles
        Obs = np.array(Obs, dtype=int)
        # keep points t in TG where t NOT equal to any obstacle row
        mask = ~np.array([any((t == o).all() for o in Obs) for t in TG])
        G = TG[mask]
        G1 = np.array([sub2ind(sz, int(g[0]), int(g[1])) for g in G], dtype=int)
        Cmax = Nx*Ny - len(Obs) - 1  # exclude sink later
    else:
        G = TG
        G1 = np.array([sub2ind(sz, int(g[0]), int(g[1])) for g in TG], dtype=int)
        Cmax = Nx*Ny - 1  # exclude sink later

    return TG, G, G1, sz, Cmax


def index_blocks(Nx, Ny, N, T):
    """
    Layout matches MATLAB variable stacking:
    - First block: ci (Nx*Ny)
    - Second block: ct (N*T*Nx*Ny)
    - Third block: z  (N*T*Nx*Ny)
    Return block sizes and total variable count.
    """
    nvars_ci = Nx * Ny
    nvars_ct = N * T * Nx * Ny
    nvars_zn = N * T * Nx * Ny
    total = nvars_ci + nvars_ct + nvars_zn
    return nvars_ci, nvars_ct, nvars_zn, total


def _zero_out_disallowed_ubs(ub, Nx, Ny, N, T, G1, P_sink, nvars_ci, nvars_ct, nvars_zn):
    """
    Force variables corresponding to disallowed cells (obstacles + out of G)
    and the sink to be 0 by setting their upper bounds to 0. This prevents
    any accidental usage outside allowed set in ct/z and prevents ci at sink.
    """
    all_lin = np.arange(1, Nx * Ny + 1, dtype=int)
    allowed = set(int(i) for i in G1)
    disallowed = [i for i in all_lin if i not in allowed]

    # (1) ci block
    # - disallowed cells
    for idx in disallowed:
        ub[idx - 1] = 0.0
    # - sink cell (do not cover sink)
    ub[P_sink - 1] = 0.0

    # (2) ct block
    # For every (t,n), zero disallowed and sink entries
    for t in range(T):
        for n in range(N):
            base_ct = nvars_ci + t * N * Nx * Ny + n * Nx * Ny
            for idx in disallowed:
                ub[base_ct + (idx - 1)] = 0.0
            ub[base_ct + (P_sink - 1)] = 0.0

    # (3) z block
    # For every (t,n), zero disallowed and keep sink usable for connectivity (OPTION!)
    # Usually, we also zero the sink for z, since we don't want to place agents at sink as a state.
    # If you need z to reference the sink for connectivity edges, COMMENT OUT the sink line below.
    for t in range(T):
        for n in range(N):
            base_zn = nvars_ci + nvars_ct + t * N * Nx * Ny + n * Nx * Ny
            for idx in disallowed:
                ub[base_zn + (idx - 1)] = 0.0
            ub[base_zn + (P_sink - 1)] = 0.0


def build_coverage_problem(Rs, Rc, x, y, P_sink, N, T, Obs=None):
    """
    Python equivalent of coverageOptimizationWithConnectivity.m
    Objective: maximize coverage subject to connectivity and mobility constraints.
    Returns a dict with:
      Z1, A, b, senses, lb, ub, vtypes, and useful metadata (G, G1, sz, L, Cmax, N, T, P_sink)
    """
    TG, G, G1, sz, Cmax = build_common_data(x, y, P_sink, Obs)
    L, Irs, Irs_sink, Irc, Irc_sink = compute_sets(G, P_sink, sz, Rs, Rc)
    Nx, Ny = sz

    nvars_ci, nvars_ct, nvars_zn, total = index_blocks(Nx, Ny, N, T)

    # Objective f: f(G1)=1, f(P_sink)=0, maximize f' * ci -> set Z1 = -f to minimize
    f = np.zeros(total)
    # ci occupies positions [0 : Nx*Ny)
    f[(G1 - 1)] = 1.0  # Only allowed cells contribute
    f[P_sink - 1] = 0.0
    Z1 = -f  # minimize -f equals maximize f

    rows = []
    rhs = []
    senses = []  # 'E' for =, 'L' for <=, 'G' for >=

    # 1(a) one-UAV-at-single-point: for each (t, n): sum_p ct(t,n,p) = 1 (exclude sink & obstacles)
    for t in range(1, T + 1):
        for n in range(1, N + 1):
            row = np.zeros(total)
            base_ct = nvars_ci + (t - 1) * N * Nx * Ny + (n - 1) * Nx * Ny
            row[base_ct + (G1 - 1)] = 1.0
            # explicitly exclude sink
            row[base_ct + (P_sink - 1)] = 0.0
            rows.append(row); rhs.append(1.0); senses.append('E')

    # 1(b) single-UAV-at-one-pt: for each (t,p), sum_n ct(t,n,p) <= 1, skip sink point index L
    for t in range(1, T + 1):
        for p in range(len(G)):
            row = np.zeros(total)
            if p != L:
                for n in range(1, N + 1):
                    base_ct = nvars_ci + (t - 1) * N * Nx * Ny + (n - 1) * Nx * Ny
                    row[base_ct + (G1[p] - 1)] += 1.0
            rows.append(row); rhs.append(1.0); senses.append('L')

    # 2(a) connectivity to sink per t: sum_{n,i in Irc_sink} z(t,n,i) >= 1 -> -sum z <= -1
    for t in range(1, T + 1):
        row = np.zeros(total)
        for n in range(1, N + 1):
            base_zn = nvars_ci + nvars_ct + (t - 1) * N * Nx * Ny + (n - 1) * Nx * Ny
            for i in Irc_sink:
                row[base_zn + (i - 1)] += -1.0
        rows.append(row); rhs.append(-1.0); senses.append('L')

    # 2(b) Inter-UAV connectivity: for n=2..N, for each p:
    # ct(t,n,p) - sum z(t, n-1, i in Irc[p]) <= 0
    for t in range(1, T + 1):
        for n in range(2, N + 1):
            for p in range(len(G)):
                row = np.zeros(total)
                if p != L:
                    base_ct = nvars_ci + (t - 1) * N * Nx * Ny + (n - 1) * Nx * Ny
                    row[base_ct + (G1[p] - 1)] += 1.0
                    base_zn_prev = nvars_ci + nvars_ct + (t - 1) * N * Nx * Ny + (n - 2) * Nx * Ny
                    for i in Irc[p]:
                        row[base_zn_prev + (i - 1)] += -1.0
                rows.append(row); rhs.append(0.0); senses.append('L')

    # Mobility: z(t+1,n,p) - sum z(t,n,i in Irc[p]) <= 0
    for t in range(1, T):
        for n in range(1, N + 1):
            for p in range(len(G)):
                row = np.zeros(total)
                if p != L:
                    base_zn_next = nvars_ci + nvars_ct + t * N * Nx * Ny + (n - 1) * Nx * Ny
                    base_zn_cur  = nvars_ci + nvars_ct + (t - 1) * N * Nx * Ny + (n - 1) * Nx * Ny
                    row[base_zn_next + (G1[p] - 1)] += 1.0
                    for i in Irc[p]:
                        row[base_zn_cur + (i - 1)] += -1.0
                rows.append(row); rhs.append(0.0); senses.append('L')

    # 4(a) Coverage linking: ct(t,n,p) - sum z(t,n, i in Irs[p]) = 0
    for t in range(1, T + 1):
        for n in range(1, N + 1):
            for p in range(len(G)):
                row = np.zeros(total)
                if p != L:
                    base_ct = nvars_ci + (t - 1) * N * Nx * Ny + (n - 1) * Nx * Ny
                    base_zn = nvars_ci + nvars_ct + (t - 1) * N * Nx * Ny + (n - 1) * Nx * Ny
                    row[base_ct + (G1[p] - 1)] += 1.0
                    for i in Irs[p]:
                        row[base_zn + (i - 1)] += -1.0
                rows.append(row); rhs.append(0.0); senses.append('E')

    # 4(b) ci(q) - ct(t,n,q) >= 0  -> -ci(q) + ct(t,n,q) <= 0
    for t in range(1, T + 1):
        for n in range(1, N + 1):
            for q in range(len(G)):
                row = np.zeros(total)
                row[(G1[q] - 1)] += -1.0
                base_ct = nvars_ci + (t - 1) * N * Nx * Ny + (n - 1) * Nx * Ny
                row[base_ct + (G1[q] - 1)] += 1.0
                rows.append(row); rhs.append(0.0); senses.append('L')

    # 4(c) sum_{t,n} ct(t,n,q) - ci(q) <= 0  (as in MATLAB)
    for q in range(len(G)):
        row = np.zeros(total)
        row[(G1[q] - 1)] += 1.0
        for t in range(1, T + 1):
            for n in range(1, N + 1):
                base_ct = nvars_ci + (t - 1) * N * Nx * Ny + (n - 1) * Nx * Ny
                row[base_ct + (G1[q] - 1)] += -1.0
        rows.append(row); rhs.append(0.0); senses.append('L')

    # Bounds and variable types
    lb = np.zeros(total)
    ub = np.ones(total)
    vtypes = np.array(['C'] * total, dtype='<U1')
    # If you want ct to be binary as well, uncomment the next block:
    # for t in range(T):
    #     for n in range(N):
    #         base_ct = nvars_ci + t*N*Nx*Ny + n*Nx*Ny
    #         vtypes[base_ct : base_ct + Nx*Ny] = 'B'
    # z variables are binary
    vtypes[nvars_ci + nvars_ct : nvars_ci + nvars_ct + nvars_zn] = 'B'

    # IMPORTANT: zero-out UBs for disallowed cells & sink
    _zero_out_disallowed_ubs(ub, Nx, Ny, N, T, G1, P_sink, nvars_ci, nvars_ct, nvars_zn)

    A = np.vstack(rows)
    b = np.array(rhs, dtype=float)
    senses = np.array(senses)

    return dict(
        Z1=Z1, A=A, b=b, senses=senses,
        lb=lb, ub=ub, vtypes=vtypes,
        nvars_ci=nvars_ci, nvars_ct=nvars_ct, nvars_zn=nvars_zn,
        total=total, G=G, G1=G1, sz=sz, L=L, Cmax=Cmax,
        N=N, T=T, P_sink=P_sink
    )


def build_movement_problem(Rs, Rc, x, y, P_sink, N, T, cr=1.0, Obs=None):
    """
    Python equivalent of movementOptimizationWithConnectivity.m
    Objective: minimize movements (through z variables) while meeting desired coverage (W' S <= -cr*Cmax).
    Returns a dict model like build_coverage_problem.
    """
    TG, G, G1, sz, Cmax = build_common_data(x, y, P_sink, Obs)
    L, Irs, Irs_sink, Irc, Irc_sink = compute_sets(G, P_sink, sz, Rs, Rc)
    Nx, Ny = sz

    nvars_ci, nvars_ct, nvars_zn, total = index_blocks(Nx, Ny, N, T)

    # Objective ff: sum over z(t,n, allowed positions), set sink to 0
    ff = np.zeros(total)
    for t in range(1, T + 1):
        for n in range(1, N + 1):
            base_zn = nvars_ci + nvars_ct + (t - 1) * N * Nx * Ny + (n - 1) * Nx * Ny
            ff[base_zn + (G1 - 1)] = 1.0
            ff[base_zn + (P_sink - 1)] = 0.0
    Z1 = ff

    rows = []
    rhs = []
    senses = []

    # 1(a) one-UAV-at-single-point: sum_p ct(t,n,p) = 1 (exclude sink & obstacles)
    for t in range(1, T + 1):
        for n in range(1, N + 1):
            row = np.zeros(total)
            base_ct = nvars_ci + (t - 1) * N * Nx * Ny + (n - 1) * Nx * Ny
            row[base_ct + (G1 - 1)] = 1.0
            row[base_ct + (P_sink - 1)] = 0.0  # exclude sink explicitly
            rows.append(row); rhs.append(1.0); senses.append('E')

    # 1(b) single-UAV-at-one-pt: for each (t,p), sum_n ct(t,n,p) <= 1
    for t in range(1, T + 1):
        for p in range(len(G)):
            row = np.zeros(total)
            if p != L:
                for n in range(1, N + 1):
                    base_ct = nvars_ci + (t - 1) * N * Nx * Ny + (n - 1) * Nx * Ny
                    row[base_ct + (G1[p] - 1)] += 1.0
            rows.append(row); rhs.append(1.0); senses.append('L')

    # 2(a) sink connectivity on z: -sum_{n,i in Irc_sink} z(t,n,i) <= -1
    for t in range(1, T + 1):
        row = np.zeros(total)
        for n in range(1, N + 1):
            base_zn = nvars_ci + nvars_ct + (t - 1) * N * Nx * Ny + (n - 1) * Nx * Ny
            for i in Irc_sink:
                row[base_zn + (i - 1)] += -1.0
        rows.append(row); rhs.append(-1.0); senses.append('L')

    # 2(b) inter-UAV: ct(t,n,p) - sum z(t, n-1, Irc[p]) <= 0 for n>=2
    for t in range(1, T + 1):
        for n in range(2, N + 1):
            for p in range(len(G)):
                row = np.zeros(total)
                if p != L:
                    base_ct = nvars_ci + (t - 1) * N * Nx * Ny + (n - 1) * Nx * Ny
                    row[base_ct + (G1[p] - 1)] += 1.0
                    base_zn_prev = nvars_ci + nvars_ct + (t - 1) * N * Nx * Ny + (n - 2) * Nx * Ny
                    for i in Irc[p]:
                        row[base_zn_prev + (i - 1)] += -1.0
                rows.append(row); rhs.append(0.0); senses.append('L')

    # Mobility: z(t+1,n,p) - sum z(t,n, Irc[p]) <= 0
    for t in range(1, T):
        for n in range(1, N + 1):
            for p in range(len(G)):
                row = np.zeros(total)
                if p != L:
                    base_zn_next = nvars_ci + nvars_ct + t * N * Nx * Ny + (n - 1) * Nx * Ny
                    base_zn_cur  = nvars_ci + nvars_ct + (t - 1) * N * Nx * Ny + (n - 1) * Nx * Ny
                    row[base_zn_next + (G1[p] - 1)] += 1.0
                    for i in Irc[p]:
                        row[base_zn_cur + (i - 1)] += -1.0
                rows.append(row); rhs.append(0.0); senses.append('L')

    # 4(a) ct linking to sensing z: ct(t,n,p) - sum z(t,n, Irs[p]) = 0
    for t in range(1, T + 1):
        for n in range(1, N + 1):
            for p in range(len(G)):
                row = np.zeros(total)
                if p != L:
                    base_ct = nvars_ci + (t - 1) * N * Nx * Ny + (n - 1) * Nx * Ny
                    base_zn = nvars_ci + nvars_ct + (t - 1) * N * Nx * Ny + (n - 1) * Nx * Ny
                    row[base_ct + (G1[p] - 1)] += 1.0
                    for i in Irs[p]:
                        row[base_zn + (i - 1)] += -1.0
                rows.append(row); rhs.append(0.0); senses.append('E')

    # 4(b) ci(q) - ct(t,n,q) >= 0  -> -ci(q) + ct(t,n,q) <= 0
    for t in range(1, T + 1):
        for n in range(1, N + 1):
            for q in range(len(G)):
                row = np.zeros(total)
                row[(G1[q] - 1)] += -1.0
                base_ct = nvars_ci + (t - 1) * N * Nx * Ny + (n - 1) * Nx * Ny
                row[base_ct + (G1[q] - 1)] += 1.0
                rows.append(row); rhs.append(0.0); senses.append('L')

    # 4(c) sum_{t,n} ct(t,n,q) - ci(q) <= 0
    for q in range(len(G)):
        row = np.zeros(total)
        row[(G1[q] - 1)] += 1.0
        for t in range(1, T + 1):
            for n in range(1, N + 1):
                base_ct = nvars_ci + (t - 1) * N * Nx * Ny + (n - 1) * Nx * Ny
                row[base_ct + (G1[q] - 1)] += -1.0
        rows.append(row); rhs.append(0.0); senses.append('L')

    # Desired coverage: W'.S <= -cr*Cmax, with W(G1) = -1 applied to ci -> -sum(ci) <= -cr*Cmax
    row = np.zeros(total)
    row[0 : (Nx * Ny)] = -1.0
    # ci at disallowed and sink already have ub=0, so they don't affect feasibility
    rows.append(row); rhs.append(-cr * Cmax); senses.append('L')

    # Bounds and variable types
    lb = np.zeros(total)
    ub = np.ones(total)
    vtypes = np.array(['C'] * total, dtype='<U1')
    # If you want ct to be binary as well, uncomment:
    # for t in range(T):
    #     for n in range(N):
    #         base_ct = nvars_ci + t*N*Nx*Ny + n*Nx*Ny
    #         vtypes[base_ct : base_ct + Nx*Ny] = 'B'
    vtypes[nvars_ci + nvars_ct : nvars_ci + nvars_ct + nvars_zn] = 'B'  # z are binary

    # IMPORTANT: zero-out UBs for disallowed cells & sink
    _zero_out_disallowed_ubs(ub, Nx, Ny, N, T, G1, P_sink, nvars_ci, nvars_ct, nvars_zn)

    A = np.vstack(rows)
    b = np.array(rhs, dtype=float)
    senses = np.array(senses)

    return dict(
        Z1=Z1, A=A, b=b, senses=senses,
        lb=lb, ub=ub, vtypes=vtypes,
        nvars_ci=nvars_ci, nvars_ct=nvars_ct, nvars_zn=nvars_zn,
        total=total, G=G, G1=G1, sz=sz, L=L, Cmax=Cmax,
        N=N, T=T, P_sink=P_sink
    )
