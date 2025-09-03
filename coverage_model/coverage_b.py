def coverage_optimize():
    """
    Main coverage optimization function with battery & action decisions.
    Implements:
      - Max/Min battery limits
      - Turn vs Move exclusivity
      - Exchange vs No-Exchange exclusivity
      - Exchange requires presence at a base station
      - Battery update rule with recharge or drain (linearized)
      - Multi-objective: maximize coverage, minimize energy use
    References: your battery constraints sheet.  # (see chat for citation)
    """
    import numpy as np
    import cplex
    from cplex.exceptions import CplexError
    import time
    from Gpt import communicable_gpt, sensing_gpt
    from pathplotter import plot_interactive_paths

    # ==============================
    # User defined input
    # ==============================
    Rs = 1   # Sensing Radius
    Rc = 2 * Rs  # Communication Radius

    # ---------- Battery / energy params ----------
    b_full  = 10.0   # full battery units (set to your real number)
    e_base  =  2.0   # minimum safe battery level
    b_turn  =  0.5   # cost if turning (no translation)
    b_move  =  1.0   # cost if moving to a neighbor cell
    BIG_M   = b_full # big-M for linearization; safe choice here

    # Multi-objective weights (weighted sum)
    w_cov    = 1.0   # coverage reward weight
    w_energy = 0.05  # energy penalty weight (tune as you like)

    # ==============================
    # Network Grid
    # ==============================
    x = np.arange(1, 7)  # 1:6
    y = np.arange(1, 7)  # 1:6
    b = x
    a = y

    A, B = np.meshgrid(a, b)
    Nx = len(x)
    Ny = len(y)
    TG = np.column_stack((A.flatten(), B.flatten()))
    sz = (Nx, Ny)

    P_sink = 0
    row, col = np.unravel_index(P_sink, sz)
    sink = np.array([row + 1, col + 1])

    # Obstacles (example)
    Obs = np.array([(3,3), (5,3)]).reshape(-1, 2)

    if Obs.size > 0:
        O_lin = np.array([np.ravel_multi_index((r-1, c-1), sz) for r, c in Obs], dtype=int)
    else:
        O_lin = np.array([], dtype=int)

    if Obs.size > 0:
        obs_pairs = set(map(tuple, Obs.astype(int)))
        G_index = np.array([tuple(rc) not in obs_pairs for rc in TG])
        G = TG[G_index]
        G1 = np.array([np.ravel_multi_index((r-1, c-1), sz) for r, c in G])
        Cmax = Nx * Ny - len(Obs) - 1
    else:
        G = TG
        G1 = np.array([np.ravel_multi_index((r-1, c-1), sz) for r, c in TG])
        Cmax = Nx * Ny - 1

    # ---------- Base stations ----------
    # By default: only the sink cell is a base. Add more indices if needed.
    BASE_CELLS = {np.ravel_multi_index((sink[0]-1, sink[1]-1), sz)}

    # ==============================
    # Communicable & sensing sets
    # ==============================
    P_sink_0based = np.ravel_multi_index((sink[0]-1, sink[1]-1), sz)
    Irc, Irc_sink = communicable_gpt(P_sink_0based, G-1, sz, Rc, O_lin)
    L, Irs, Irs_sink = sensing_gpt(P_sink_0based, G-1, sz, Rs, O_lin)

    if O_lin.size > 0:
        Irc = [[q for q in Irc[p] if q not in O_lin] for p in range(len(G))]
        Irs = [[q for q in Irs[p] if q not in O_lin] for p in range(len(G))]
        Irc_sink = [q for q in Irc_sink if q not in O_lin]

    N = 4  # UAVs
    T = 6  # time steps

    # ==============================
    # Objective base (coverage)
    # ==============================
    f = np.zeros((1 + 2*N*T) * Nx * Ny)
    f[G1] = 1
    f[P_sink] = 0
    if O_lin.size > 0:
        f[O_lin] = 0

    # Position constraints (same as your code) ...
    # --- 1(a) one-UAV-at-single-point
    F = np.zeros((N*T, (1 + 2*N*T)*Nx*Ny))
    k = 0
    for t in range(T):
        for n in range(N):
            f1 = np.zeros((1 + 2*N*T)*Nx*Ny)
            f1[G1 + (1 + T*N)*Nx*Ny + (t*N*Nx*Ny) + (n*Nx*Ny)] = 1
            F[k, :] = f1
            k += 1

    # --- 1(b) single-UAV-at-one-pt
    E = np.zeros((T*len(G), (1 + 2*N*T)*Nx*Ny))
    k = 0
    for t in range(T):
        for p in range(len(G)):
            f2 = np.zeros((1 + 2*N*T)*Nx*Ny)
            if p != L:
                for n in range(N):
                    f2[(1 + T*N)*Nx*Ny + (t*N*Nx*Ny) + (n*Nx*Ny) + G1[p]] = 1
            E[k, :] = f2
            k += 1

    # Connectivity (same) ...
    g = np.zeros((T, (1 + 2*N*T)*Nx*Ny))
    k = 0
    for t in range(T):
        g1 = np.zeros((1 + 2*N*T)*Nx*Ny)
        for n in range(N):
            for i in range(len(Irc_sink)):
                g1[Nx*Ny + (N*T*Nx*Ny) + (t*N*Nx*Ny) + (n*Nx*Ny) + Irc_sink[i]] = -1
        g[k, :] = g1
        k += 1

    H = np.zeros((T*(N-1)*len(G), (1 + 2*N*T)*Nx*Ny))
    k = 0
    for t in range(T):
        for n in range(1, N):
            for p in range(len(G)):
                h11 = np.zeros((1 + 2*N*T)*Nx*Ny)
                if p != L:
                    h11[Nx*Ny + T*N*Nx*Ny + (t*N*Nx*Ny) + (n*Nx*Ny) + G1[p]] = 1
                    for i in range(len(Irc[p])):
                        h11[Nx*Ny + (N*T*Nx*Ny) + (t*N*Nx*Ny) + ((n-1)*Nx*Ny) + Irc[p][i]] = -1
                H[k, :] = h11
                k += 1

    # Mobility (same) ...
    I = np.zeros(((T-1)*N*len(G), (1 + 2*N*T)*Nx*Ny))
    k = 0
    for t in range(T-1):
        for n in range(N):
            for p in range(len(G)):
                h12 = np.zeros((1 + 2*N*T)*Nx*Ny)
                if p != L:
                    h12[Nx*Ny + (N*T*Nx*Ny) + ((t+1)*N*Nx*Ny) + (n*Nx*Ny) + G1[p]] = 1
                    for i in range(len(Irc[p])):
                        h12[Nx*Ny + (N*T*Nx*Ny) + (t*N*Nx*Ny) + (n*Nx*Ny) + Irc[p][i]] = -1
                I[k, :] = h12
                k += 1

    # Coverage constraints (same) ...
    K = np.zeros((T*N*len(G), (1 + 2*N*T)*Nx*Ny))
    k = 0
    for t in range(T):
        for n in range(N):
            for p in range(len(G)):
                h1 = np.zeros((1 + 2*N*T)*Nx*Ny)
                if p != L:
                    h1[Nx*Ny + (t*N*Nx*Ny) + (n*Nx*Ny) + G1[p]] = 1
                    for i in range(len(Irs[p])):
                        h1[Nx*Ny + (N*T*Nx*Ny) + (t*N*Nx*Ny) + (n*Nx*Ny) + Irs[p][i]] = -1
                K[k, :] = h1
                k += 1

    J = np.zeros((N*T*len(G), (1 + 2*N*T)*Nx*Ny))
    k = 0
    for t in range(T):
        for n in range(N):
            for q in range(len(G)):
                h2 = np.zeros((1 + 2*N*T)*Nx*Ny)
                h2[G1[q]] = -1
                h2[G1[L]] = 0
                h2[Nx*Ny + (t*N*Nx*Ny) + (n*Nx*Ny) + G1[q]] = 1
                h2[Nx*Ny + (t*N*Nx*Ny) + (n*Nx*Ny) + G1[L]] = 0
                J[k, :] = h2
                k += 1

    Q = np.zeros((len(G), (1 + 2*N*T)*Nx*Ny))
    k = 0
    for q in range(len(G)):
        h3 = np.zeros((1 + 2*N*T)*Nx*Ny)
        h3[G1[q]] = 1
        h3[G1[L]] = 0
        for t in range(T):
            for n in range(N):
                h3[Nx*Ny + (t*N*Nx*Ny) + (n*Nx*Ny) + G1[q]] = -1
                h3[Nx*Ny + (t*N*Nx*Ny) + (n*Nx*Ny) + G1[L]] = 0
        Q[k, :] = h3
        k += 1

    # ==============================
    # Optimization problem setup
    # ==============================
    Z1 = -f  # coverage maximize -> minimize negative

    # Inequalities / equalities (base)
    Aineq = np.vstack([E, g, H, I, J, Q])
    bineq = np.hstack([
        np.ones(E.shape[0]),
        -np.ones(g.shape[0]),
        np.zeros(H.shape[0]),
        np.zeros(I.shape[0]),
        np.zeros(J.shape[0]),
        np.zeros(Q.shape[0])
    ])
    Aeq = np.vstack([F, K])
    beq = np.hstack([np.ones(F.shape[0]), np.zeros(K.shape[0])])

    # Bounds for existing vars
    lb = np.zeros((Nx*Ny) + 2*N*T*Nx*Ny)
    ub = np.ones((Nx*Ny) + 2*N*T*Nx*Ny)

    # -------- NO-FLY OBSTACLE BOUNDS --------
    def idx_y(q):                      # selection
        return q
    def idx_a(t, n, q):                # coverage assign
        return Nx*Ny + (t*N*Nx*Ny) + (n*Nx*Ny) + q
    def idx_x(t, n, q):                # positions (binary)
        return Nx*Ny + (N*T*Nx*Ny) + (t*N*Nx*Ny) + (n*Nx*Ny) + q

    if O_lin.size > 0:
        for q in O_lin:
            ub[idx_y(q)] = 0.0
            for t in range(T):
                for n in range(N):
                    ub[idx_a(t, n, q)] = 0.0
                    ub[idx_x(t, n, q)] = 0.0
    # -------- END NO-FLY --------

    # ==============================
    # ### NEW (BATTERY) — add variables
    # ==============================
    # We add, for each (k,n):
    #   b[k,n]            : battery level (continuous)
    #   y_turn[k,n]       : binary
    #   y_move[k,n]       : binary
    #   Y_ex[k,n]         : binary (exchange)
    #   Y_noex[k,n]       : binary (no-exchange)
    #   z_base[k,n]       : in [0,1], equals sum of x over base cells at (k,n)

    # We'll append these to the model after your original vector.
    base_var_count = (Nx*Ny) + 2*N*T*Nx*Ny  # your original variables
    def id_b(k, n):
        return base_var_count + (k*N + n)
    def id_yturn(k, n):
        return base_var_count + (T*N) + (k*N + n)
    def id_ymove(k, n):
        return base_var_count + (T*N) + (T*N) + (k*N + n)
    def id_yex(k, n):
        return base_var_count + (3*T*N) + (k*N + n)
    def id_ynoex(k, n):
        return base_var_count + (4*T*N) + (k*N + n)
    def id_zbase(k, n):
        return base_var_count + (5*T*N) + (k*N + n)

    extra_vars = 6*T*N
    # Extend bounds/types/objective
    lb_ext = []
    ub_ext = []
    ctype_ext = []
    obj_ext = []

    # b[k,n]
    for k in range(T):
        for n in range(N):
            lb_ext.append(0.0)
            ub_ext.append(b_full)
            ctype_ext.append('C')
            obj_ext.append(0.0)

    # y_turn[k,n]
    for k in range(T):
        for n in range(N):
            lb_ext.append(0.0); ub_ext.append(1.0); ctype_ext.append('B'); obj_ext.append(w_energy * b_turn)

    # y_move[k,n]
    for k in range(T):
        for n in range(N):
            lb_ext.append(0.0); ub_ext.append(1.0); ctype_ext.append('B'); obj_ext.append(w_energy * b_move)

    # Y_exchange[k,n]
    for k in range(T):
        for n in range(N):
            lb_ext.append(0.0); ub_ext.append(1.0); ctype_ext.append('B'); obj_ext.append(w_energy * (b_full))  # upper-bound cost weight

    # Y_noexchange[k,n]
    for k in range(T):
        for n in range(N):
            lb_ext.append(0.0); ub_ext.append(1.0); ctype_ext.append('B'); obj_ext.append(0.0)

    # z_base[k,n] (continuous in [0,1])
    for k in range(T):
        for n in range(N):
            lb_ext.append(0.0); ub_ext.append(1.0); ctype_ext.append('C'); obj_ext.append(0.0)

    # Merge with existing vectors
    lb = np.concatenate([lb, np.array(lb_ext)])
    ub = np.concatenate([ub, np.array(ub_ext)])
    ctype = ['C'] * ((1 + N*T) * Nx * Ny) + ['B'] * (N*T * Nx * Ny) + ctype_ext
    Z1 = np.concatenate([Z1, np.array(obj_ext)])

    # ==============================
    # ### NEW (BATTERY) — constraints
    # ==============================
    Ain_list = []
    bin_list = []
    sense_list = []

    # Max / Min battery (1)(2)
    for k in range(T):
        for n in range(N):
            # b_kn ≤ b_full  ->  b_kn - b_full ≤ 0
            row = cplex.SparsePair(ind=[id_b(k,n)], val=[1.0])
            Ain_list.append(row); bin_list.append(b_full); sense_list.append("L")
            # b_kn ≥ e_base  ->  -b_kn ≤ -e_base
            row = cplex.SparsePair(ind=[id_b(k,n)], val=[-1.0])
            Ain_list.append(row); bin_list.append(-e_base); sense_list.append("L")

    # Turn vs Move (3): y_turn + y_move ≤ 1
    for k in range(T):
        for n in range(N):
            row = cplex.SparsePair(ind=[id_yturn(k,n), id_ymove(k,n)], val=[1.0, 1.0])
            Ain_list.append(row); bin_list.append(1.0); sense_list.append("L")

    # Exchange vs No-Exchange (4): Y_ex + Y_noex ≤ 1
    for k in range(T):
        for n in range(N):
            row = cplex.SparsePair(ind=[id_yex(k,n), id_ynoex(k,n)], val=[1.0, 1.0])
            Ain_list.append(row); bin_list.append(1.0); sense_list.append("L")

    # Exchange ⇒ at base (9): Y_ex ≤ z_base
    for k in range(T):
        for n in range(N):
            row = cplex.SparsePair(ind=[id_yex(k,n), id_zbase(k,n)], val=[1.0, -1.0])
            Ain_list.append(row); bin_list.append(0.0); sense_list.append("L")

    # z_base(k,n) = sum_{q in BASE} x_{k,n,q}
    Aeq_list = []
    beq_list = []
    for k in range(T):
        for n in range(N):
            inds = [id_zbase(k,n)]
            vals = [1.0]
            for q in BASE_CELLS:
                inds.append(idx_x(k, n, q))
                vals.append(-1.0)
            Aeq_list.append(cplex.SparsePair(ind=inds, val=vals))
            beq_list.append(0.0)

    # Battery update (10) linearized:
    # b_{k+1,n} = Y_noex*(b_{k,n} - y_turn*b_turn - y_move*b_move) + Y_ex*b_full
    # -> four inequalities (upper/lower) with big-M for each branch.
    for k in range(T-1):
        for n in range(N):
            #  b_{k+1} ≤ b_full + M*(1 - Y_ex)
            inds = [id_b(k+1,n), id_yex(k,n)]
            vals = [ 1.0,           BIG_M]
            Ain_list.append(cplex.SparsePair(ind=inds, val=vals))
            bin_list.append(b_full + BIG_M)  # move RHS to include "+ M"
            sense_list.append("L")
            #  b_{k+1} ≥ b_full - M*(1 - Y_ex)  ->  -b_{k+1} ≤ -b_full + M*(1 - Y_ex)
            inds = [id_b(k+1,n), id_yex(k,n)]
            vals = [-1.0,         BIG_M]
            Ain_list.append(cplex.SparsePair(ind=inds, val=vals))
            bin_list.append(-b_full + BIG_M)
            sense_list.append("L")

            #  b_{k+1} ≤ b_k - y_turn*b_turn - y_move*b_move + M*(1 - Y_noex)
            inds = [id_b(k+1,n), id_b(k,n), id_yturn(k,n), id_ymove(k,n), id_ynoex(k,n)]
            vals = [ 1.0,        -1.0,      b_turn,         b_move,        BIG_M]
            Ain_list.append(cplex.SparsePair(ind=inds, val=vals))
            bin_list.append(BIG_M)
            sense_list.append("L")

            #  b_{k+1} ≥ b_k - y_turn*b_turn - y_move*b_move - M*(1 - Y_noex)
            # ->  -b_{k+1} ≤ -b_k + y_turn*b_turn + y_move*b_move + M*(1 - Y_noex)
            inds = [id_b(k+1,n), id_b(k,n), id_yturn(k,n), id_ymove(k,n), id_ynoex(k,n)]
            vals = [-1.0,         1.0,      b_turn,         b_move,        BIG_M]
            Ain_list.append(cplex.SparsePair(ind=inds, val=vals))
            bin_list.append(BIG_M)
            sense_list.append("L")

    # Link y_move to position change between k and k+1:
    # For all q:  x_{k+1,n,q} - x_{k,n,q} ≤ y_move  and  x_{k,n,q} - x_{k+1,n,q} ≤ y_move
    for k in range(T-1):
        for n in range(N):
            for q in range(Nx*Ny):
                # skip impossible obstacle cells (bounds already 0) — constraints harmless either way
                inds = [idx_x(k+1,n,q), idx_x(k,n,q), id_ymove(k,n)]
                vals = [1.0,            -1.0,        -1.0]
                Ain_list.append(cplex.SparsePair(ind=inds, val=vals))
                bin_list.append(0.0); sense_list.append("L")

                inds = [idx_x(k,n,q), idx_x(k+1,n,q), id_ymove(k,n)]
                vals = [1.0,           -1.0,          -1.0]
                Ain_list.append(cplex.SparsePair(ind=inds, val=vals))
                bin_list.append(0.0); sense_list.append("L")

    # (Optional) force exactly-one of exchange/no-exchange each step:
    # Comment out if you truly want "≤ 1". Equality usually stabilizes the update.
    for k in range(T):
        for n in range(N):
            Aeq_list.append(cplex.SparsePair(ind=[id_yex(k,n), id_ynoex(k,n)], val=[1.0, 1.0]))
            beq_list.append(1.0)

    # ==============================
    # Build and solve
    # ==============================
    prob = cplex.Cplex()
    prob.objective.set_sense(prob.objective.sense.minimize)

    # Variables (original + battery/action block)
    prob.variables.add(obj=Z1.tolist(), lb=lb.tolist(), ub=ub.tolist(), types=ctype)

    # Original inequality constraints
    for i in range(Aineq.shape[0]):
        prob.linear_constraints.add(
            lin_expr=[cplex.SparsePair(ind=list(range(len(Aineq[i]))), val=Aineq[i].tolist())],
            senses=["L"],
            rhs=[bineq[i]]
        )

    # Original equality constraints
    for i in range(Aeq.shape[0]):
        prob.linear_constraints.add(
            lin_expr=[cplex.SparsePair(ind=list(range(len(Aeq[i]))), val=Aeq[i].tolist())],
            senses=["E"],
            rhs=[beq[i]]
        )

    # NEW: battery/action inequalities
    if Ain_list:
        prob.linear_constraints.add(lin_expr=Ain_list, senses=sense_list, rhs=bin_list)

    # NEW: battery/action equalities
    if Aeq_list:
        prob.linear_constraints.add(lin_expr=Aeq_list, senses=["E"]*len(Aeq_list), rhs=beq_list)

    # Solver options
    prob.parameters.mip.tolerances.mipgap.set(0)
    prob.parameters.timelimit.set(18000)
    prob.parameters.mip.strategy.search.set(1)

    # Solve
    t0 = time.time()
    try:
        prob.solve()
    except CplexError as e:
        print(f"CPLEX Error: {e}")
        return None
    Cov_time = time.time() - t0

    # Extract & report (same as yours) ...
    S = np.array(prob.solution.get_values())
    fval = prob.solution.get_objective_value()

    ss = np.round(S, 1)
    c = np.where(ss > 0)[0]
    path_loc = c[c > (1 + T*N) * Nx * Ny]
    U = ss[path_loc]
    V = np.column_stack((path_loc, U))

    cov_path = np.zeros((N, Nx*Ny*T), dtype=int)
    for n in range(N):
        P1 = []
        for t in range(T):
            for i in range(len(path_loc)):
                Rmin = (1 + T*N)*Nx*Ny + t*N*Nx*Ny + n*Nx*Ny
                Rmax = Rmin + Nx*Ny
                if (Rmax <= (1 + T*N + T*N)*Nx*Ny and Rmin < path_loc[i] <= Rmax):
                    c1 = path_loc[i]
                    c2 = c1 - ((1 + T*N)*Nx*Ny + t*N*Nx*Ny + n*Nx*Ny)
                    P1.append(c2)
        if len(P1) > 0:
            cov_path[n, :len(P1)] = P1

    Cov_Percent = (-1) * (np.dot(-f, S[:len(f)]) / Cmax) * 100.0

    print(f"Coverage Percentage (reward part): {Cov_Percent:.2f}%")
    print(f"Optimization Time: {Cov_time:.2f} seconds")
    print(f"Objective Value (weighted): {fval:.4f}")
    print("="*50)

    # Reconstruct readable paths & coverage (same as yours) ...
    uav_paths = {}
    uav_covered_nodes = {}
    for n in range(N):
        non_zero = cov_path[n, cov_path[n] != 0]
        if len(non_zero) > 0:
            path_coords = []
            for idx in non_zero:
                row, col = np.unravel_index(idx, sz)
                path_coords.append((row+1, col+1))
            uav_paths[n] = path_coords

            covered_nodes = set()
            for t in range(T):
                if t < len(path_coords):
                    current_pos = path_coords[t]
                    for node in G:
                        dist = np.sqrt((current_pos[0] - node[0])**2 + (current_pos[1] - node[1])**2)
                        if dist <= Rs:
                            covered_nodes.add(tuple(node))
            uav_covered_nodes[n] = list(covered_nodes)

            print(f"\nUAV {n+1}:")
            print(f"  Path: {path_coords}")
            print(f"  Total nodes covered: {len(uav_covered_nodes[n])}")
        else:
            print(f"\nUAV {n+1}: No path assigned")
            uav_paths[n] = []
            uav_covered_nodes[n] = []

    all_covered = set()
    for nodes in uav_covered_nodes.values():
        all_covered.update(nodes)
    print(f"\nTotal unique nodes covered: {len(all_covered)}")

    # Visual
    plot_interactive_paths(G, uav_paths, uav_covered_nodes, sink, Rs, Rc, Nx, Ny, O_lin)

    return {
        'S': S, 'fval': fval, 'Cov_time': Cov_time, 'cov_path': cov_path,
        'Cov_Percent': Cov_Percent, 'path_loc': path_loc, 'V': V,
        'uav_paths': uav_paths, 'uav_covered_nodes': uav_covered_nodes
    }

if __name__ == "__main__":

    coverage_optimize()