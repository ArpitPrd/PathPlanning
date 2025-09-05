import numpy as np
import cplex
from cplex.exceptions import CplexError
import time
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.colors import ListedColormap
from matplotlib.widgets import Button
from Gpt import communicable_gpt, sensing_gpt
from pathplotter import plot_interactive_paths
import argparse
import json


def load_config(json_path: str):
    """Load optimization parameters from a JSON config file."""
    with open(json_path, 'r') as f:
        cfg = json.load(f)

    return cfg



def sz_to_XY(row_size, col_size):
    """
    general purpose meshgrid maker

    need a, b to be arrays
    """
    x = np.arange(1, row_size+1)  # 1:6
    y = np.arange(1, col_size+1)  # 1:6
    
    return np.meshgrid(y, x)

def i_to_ij(index, sz):
    """
    convert lin length to row, col
    """
    return np.unravel_index(index, sz)

def ij_to_i(cod, sz):
    """
    convert coordinates to lin length wrt sz
    """
    # TODO
    return np.ravel_multi_index(cod, sz)

def XY_to_xy(A:np.array, B:np.array) -> np.array:
    """
    A, B must be obtained in the format from np.meshgrid to obtain all the coordis
    """
    return np.column_stack((A.flatten(), B.flatten()))

def make_grid(row_size, col_size):
    """
    prepares complete meshgrid and returns the gird points

    different from mesh grid because it returns XY
    """
    
    A, B = sz_to_XY(row_size, col_size)
    return XY_to_xy(A, B)

def pos_sink(row_sink, col_sink, sz):
    """
    prepares the sink wrt to the grid
    """
    P_sink = ij_to_i((row_sink, col_sink), sz)
    sink = np.array([row_sink + 1, col_sink + 1])
    return P_sink, sink

def pos_obs(coords_obs, sz=None):
    """
    positions the obstructions in the grid wrt to the size
    """
    return np.array(coords_obs).reshape(-1, len(coords_obs[0]))

# def setup_obj(G, P_sink, O_lin, N, T, row_size, col_size):
#     """
#     general purpose objective function definition
#     """
#     f = np.zeros((1 + 2*N*T) * row_size * col_size)
#     f[G] = 1
#     f[P_sink] = 0
    
#     #set the reward function for the obstacle to zero
#     if O_lin.size > 0:
#         f[O_lin] = 0
    
#     return f

def setup_obj(G, P_sink, O_lin, N, T, row_size, col_size,
              b_turn, b_mov, b_steady, b_full):
    """
    Objective function with battery-aware terms.
    """
    num_vars = (1 + 2*N*T) * row_size * col_size
    f = np.zeros(num_vars)

    # --- (1) Coverage reward ---
    f[G] = 1
    f[P_sink] = 0
    if O_lin.size > 0:
        f[O_lin] = 0

    # --- (2) Energy penalties & recharge ---
    for t in range(T):
        for n in range(N):
            base_idx = row_size*col_size + (N*T*row_size*col_size) + (t*N*row_size*col_size) + (n*row_size*col_size)

            # indices for decision variables at this (t, n)
            y_turn_idx     = base_idx      # example: first slot
            y_move_idx     = base_idx + 1  # example: second slot
            Y_noex_idx     = base_idx + 2
            Y_ex_idx       = base_idx + 3
            bkn_idx        = base_idx + 4

            # penalties
            f[y_turn_idx] -= b_turn
            f[y_move_idx] -= b_mov
            f[Y_noex_idx] -= b_steady

            # recharge reward
            f[Y_ex_idx]  += b_full
            f[bkn_idx]   -= 1   # part of (b_full - b_kn)

    return f

def battery_constraint(N, T, row_size, col_size,
                       b_turn, b_mov, b_steady, b_full):
    """
    Construct linearized battery constraints for all UAVs across time.
    """
    num_vars = (1 + 2*N*T) * row_size * col_size
    B = np.zeros(((T-1)*N, num_vars))
    rhs = np.zeros((T-1)*N)

    k = 0
    for t in range(T-1):
        for n in range(N):
            h = np.zeros(num_vars)

            # indices
            base_idx_t   = row_size*col_size + (N*T*row_size*col_size) + (t*N*row_size*col_size) + (n*row_size*col_size)
            base_idx_t1  = row_size*col_size + (N*T*row_size*col_size) + ((t+1)*N*row_size*col_size) + (n*row_size*col_size)

            b_kn_idx   = base_idx_t + 4
            b_k1n_idx  = base_idx_t1 + 4
            y_turn_idx = base_idx_t
            y_move_idx = base_idx_t + 1
            Y_noex_idx = base_idx_t + 2
            Y_ex_idx   = base_idx_t + 3

            # Row encodes:
            # b_{t+1,n} - b_{t,n} + b_turn*y_turn + b_mov*y_move
            # + b_steady*(1 - y_turn - y_move)*Y_noex - b_full*Y_ex = 0
            h[b_k1n_idx] =  1
            h[b_kn_idx]  = -1
            h[y_turn_idx] = b_turn
            h[y_move_idx] = b_mov
            h[Y_noex_idx] = b_steady
            h[Y_ex_idx]   = -b_full

            B[k, :] = h
            rhs[k] = 0
            k += 1

    return B, rhs

def filter_obs(O_lin, Irc, Irs, Irc_sink, G):
    if O_lin.size > 0:
        Irc = [[q for q in Irc[p] if q not in O_lin] for p in range(len(G))]
        Irs = [[q for q in Irs[p] if q not in O_lin] for p in range(len(G))]
        Irc_sink = [q for q in Irc_sink if q not in O_lin]

    return Irc, Irs, Irc_sink

def poisition_constraints(N, T, G, G1, L, P_sink, row_size, col_size):
    # ==============================
    # Position Constraints
    # ==============================
    # 1(a) one-UAV-at-single-point
    F = np.zeros((N*T, (1 + 2*N*T)*row_size*col_size))
    k = 0
    for t in range(T):
        for n in range(N):
            f1 = np.zeros((1 + 2*N*T)*row_size*col_size)
            f1[G1 + (1 + T*N)*row_size*col_size + (t*N*row_size*col_size) + (n*row_size*col_size)] = 1
            f1[(1 + T*N)*row_size*col_size + (t*N*row_size*col_size) + (n*row_size*col_size) + P_sink] = 0
            F[k, :] = f1
            k += 1
    
    # 1(b) single-UAV-at-one-pt
    E = np.zeros((T*len(G), (1 + 2*N*T)*row_size*col_size))
    k = 0
    for t in range(T):
        for p in range(len(G)):
            f2 = np.zeros((1 + 2*N*T)*row_size*col_size)
            if p != L:
                for n in range(N):
                    f2[(1 + T*N)*row_size*col_size + (t*N*row_size*col_size) + (n*row_size*col_size) + G1[p]] = 1
            E[k, :] = f2
            k += 1
    return F, E

def sink_connectivity_constraint(N, T, Irc_sink, row_size, col_size):
    g = np.zeros((T, (1 + 2*N*T)*row_size*col_size))
    k = 0
    for t in range(T):
        g1 = np.zeros((1 + 2*N*T)*row_size*col_size)
        for n in range(N):
            for i in range(len(Irc_sink)):
                g1[row_size*col_size + (N*T*row_size*col_size) + (t*N*row_size*col_size) + (n*row_size*col_size) + Irc_sink[i]] = -1
        g[k, :] = g1
        k += 1
    return g

def interUAV_connectivity_constraint(N, T, Irc, row_size, col_size, G, G1, L):
    H = np.zeros((T*(N-1)*len(G), (1 + 2*N*T)*row_size*col_size))
    k = 0
    for t in range(T):
        for n in range(1, N):  # n=2:N in MATLAB becomes n=1:N-1 in Python
            for p in range(len(G)):
                h11 = np.zeros((1 + 2*N*T)*row_size*col_size)
                if p != L:
                    h11[row_size*col_size + T*N*row_size*col_size + (t*N*row_size*col_size) + (n*row_size*col_size) + G1[p]] = 1
                    for i in range(len(Irc[p])):
                        h11[row_size*col_size + (N*T*row_size*col_size) + (t*N*row_size*col_size) + ((n-1)*row_size*col_size) + Irc[p][i]] = -1
                H[k, :] = h11
                k += 1
    return H

def mobility_constraint(N, T, Irc, row_size, col_size, G1, L):
    I = np.zeros(((T-1)*N*len(G1), (1 + 2*N*T)*row_size*col_size))
    k = 0
    for t in range(T-1):
        for n in range(N):
            for p in range(len(G1)):
                h12 = np.zeros((1 + 2*N*T)*row_size*col_size)
                if p != L:
                    h12[row_size*col_size + (N*T*row_size*col_size) + ((t+1)*N*row_size*col_size) + (n*row_size*col_size) + G1[p]] = 1
                    for i in range(len(Irc[p])):
                        h12[row_size*col_size + (N*T*row_size*col_size) + (t*N*row_size*col_size) + (n*row_size*col_size) + Irc[p][i]] = -1
                I[k, :] = h12
                k += 1
    return I

def battery_constraint(N, T, row_size, col_size, b_turn, b_mov, b_steady, b_full, G1, L):
    """
    Construct battery update constraints (equation 11 in PDF).

    Parameters
    ----------
    N : int
        Number of agents
    T : int
        Number of time steps
    row_size, col_size : int
        Grid dimensions
    b_turn, b_mov, b_steady, b_full : float
        Battery parameters
    G1 : list
        Node indices
    L : int
        Special index to exclude (like in mobility_constraint)

    Returns
    -------
    B : np.ndarray
        Battery constraint matrix
    rhs : np.ndarray
        RHS vector
    """
    num_vars = (1 + 2*N*T) * row_size * col_size
    B = np.zeros(((T-1)*N, num_vars))  # one constraint per agent per step
    rhs = np.zeros((T-1)*N)

    k = 0
    for t in range(T-1):
        for n in range(N):
            h = np.zeros(num_vars)

            # Indexing battery variable b_{k,n} and b_{k+1,n}
            b_kn_idx  = (t*N + n) * row_size * col_size
            b_k1n_idx = ((t+1)*N + n) * row_size * col_size

            # Add coefficients for b_{k+1,n} - b_{k,n}
            h[b_k1n_idx] = 1
            h[b_kn_idx]  = -1

            # Subtract turn, move, idle costs (only when no-exchange = 1)
            # These will link with y_turn, y_move, Y_noexchange, Y_exchange variables
            # Placeholders for where you'd index those variables:
            #   h[y_turn_idx]       = -b_turn
            #   h[y_move_idx]       = -b_mov
            #   h[idle_expr_idx]    = -b_steady
            #   h[Y_exchange_idx]   = b_full

            B[k, :] = h
            rhs[k] = 0
            k += 1

    return B, rhs


def cell_coverage_constraints(N, T, row_size, col_size, G1, L, Irs):
    # 4(a)
    K = np.zeros((T*N*len(G1), (1 + 2*N*T)*row_size*col_size))
    k = 0
    for t in range(T):
        for n in range(N):
            for p in range(len(G1)):
                h1 = np.zeros((1 + 2*N*T)*row_size*col_size)
                if p != L:
                    h1[row_size*col_size + (t*N*row_size*col_size) + (n*row_size*col_size) + G1[p]] = 1
                    for i in range(len(Irs[p])):
                        h1[row_size*col_size + (N*T*row_size*col_size) + (t*N*row_size*col_size) + (n*row_size*col_size) + Irs[p][i]] = -1
                K[k, :] = h1
                k += 1
    
    # 4(b)
    J = np.zeros((N*T*len(G1), (1 + 2*N*T)*row_size*col_size))
    k = 0
    for t in range(T):
        for n in range(N):
            for q in range(len(G1)):
                h2 = np.zeros((1 + 2*N*T)*row_size*col_size)
                h2[G1[q]] = -1
                h2[G1[L]] = 0
                h2[row_size*col_size + (t*N*row_size*col_size) + (n*row_size*col_size) + G1[q]] = 1
                h2[row_size*col_size + (t*N*row_size*col_size) + (n*row_size*col_size) + G1[L]] = 0
                J[k, :] = h2
                k += 1
    
    # 4(c)
    Q = np.zeros((len(G1), (1 + 2*N*T)*row_size*col_size))
    k = 0
    for q in range(len(G1)):
        h3 = np.zeros((1 + 2*N*T)*row_size*col_size)
        h3[G1[q]] = 1
        h3[G1[L]] = 0
        for t in range(T):
            for n in range(N):
                h3[row_size*col_size + (t*N*row_size*col_size) + (n*row_size*col_size) + G1[q]] = -1
                h3[row_size*col_size + (t*N*row_size*col_size) + (n*row_size*col_size) + G1[L]] = 0
        Q[k, :] = h3
        k += 1
    return K, J, Q

def inequality_constraints(E, g, H, I, J, Q):
    """
    combines all the inequality constraints into one matrix
    """
    return np.vstack([E, g, H, I, J, Q]), np.hstack([
        np.ones(E.shape[0]),
        -np.ones(g.shape[0]),
        np.zeros(H.shape[0]),
        np.zeros(I.shape[0]),
        np.zeros(J.shape[0]),
        np.zeros(Q.shape[0])
    ])

import numpy as np

def combine_constraints(*blocks):
    """
    Combine multiple constraint blocks (equality or inequality) into one big matrix.

    Parameters
    ----------
    *blocks : list of (np.ndarray, rhs)
        Each block is a tuple:
          - constraint matrix (np.ndarray)
          - rhs values:
              - scalar (applied to all rows of the block)
              - array matching row count
              - None → defaults to zeros

    Returns
    -------
    A : np.ndarray
        Combined constraint matrix (stacked vertically).
    b : np.ndarray
        Combined RHS vector (stacked horizontally).
    """
    matrices = []
    rhs_list = []

    for matrix, rhs in blocks:
        matrices.append(matrix)

        if rhs is None:
            rhs_vals = np.zeros(matrix.shape[0])
        elif np.isscalar(rhs):
            rhs_vals = np.full(matrix.shape[0], rhs)
        else:
            rhs = np.asarray(rhs)
            assert rhs.shape[0] == matrix.shape[0], \
                f"RHS length {rhs.shape[0]} does not match rows {matrix.shape[0]}"
            rhs_vals = rhs

        rhs_list.append(rhs_vals)

    A = np.vstack(matrices)
    b = np.hstack(rhs_list)
    return A, b




def bounds(row_size, col_size, N, T):
    """
    sets up the bounds for the optimization variables
    """
    lb = np.zeros((row_size*col_size) + 2*N*T*row_size*col_size)
    ub = np.ones((row_size*col_size) + 2*N*T*row_size*col_size)
    return lb, ub

def type_vars(row_size, col_size, N, T):
    """
    sets up the variable types for the optimization problem
    """
    vartype = ['C'] * (row_size*col_size) + ['B'] * (2*N*T*row_size*col_size)
    return vartype

def no_fly_obstacles(O_lin, row_size, col_size, N, T, ub):
    # -------- NO-FLY OBSTACLE BOUNDS --------
    # Helper indexers for the 3 blocks in your variable vector
    def idx_y(q):                      # Block A (selection), size row_size*col_size
        return q

    def idx_a(t, n, q):                # Block B (coverage assign), size N*T*row_size*col_size
        # offset_A = row_size*col_size
        return row_size*col_size + (t*N*row_size*col_size) + (n*row_size*col_size) + q

    def idx_x(t, n, q):                # Block C (positions), size N*T*row_size*col_size (binary)
        # offset_A = row_size*col_size; offset_B = N*T*row_size*col_size
        return row_size*col_size + (N*T*row_size*col_size) + (t*N*row_size*col_size) + (n*row_size*col_size) + q

    if O_lin.size > 0:
        for q in O_lin:
            # Never select obstacle cells
            ub[idx_y(q)] = 0.0
            # Never assign coverage to obstacle cells
            for t in range(T):
                for n in range(N):
                    ub[idx_a(t, n, q)] = 0.0
                    # Never occupy obstacle cells (NO-FLY)
                    ub[idx_x(t, n, q)] = 0.0

    # -------- END NO-FLY OBSTACLE BOUNDS --------

def display_results(
        Cov_Percent,
        Cov_time,
        fval,
        N,
        row_size,
        col_size,
        T,
        cov_path,
        active_indices,
        S,
        sz,
        G,
        sensing_radius,
        comm_radius,
        O_lin,
        sink,
        aux_tensor=None  # new argument
    ):
    print(f"Coverage Percentage: {Cov_Percent:.2f}%")
    print(f"Optimization Time: {Cov_time:.2f} seconds")
    print(f"Objective Value: {fval:.4f}")
    print("\n" + "="*50)
    print("UAV PATHS, COVERAGE, AND BATTERY ANALYSIS")
    print("="*50)

    # Extract and display paths for each UAV
    uav_paths = {}
    uav_covered_nodes = {}
    battery_levels = {}

    for n in range(N):
        non_zero = cov_path[n, cov_path[n] != 0]
        if len(non_zero) > 0:
            # Convert linear indices to 2D coordinates
            path_coords = []
            for idx in non_zero:
                row, col = np.unravel_index(idx, sz)
                path_coords.append((row+1, col+1))  # 1-based for display
            uav_paths[n] = path_coords

            # Coverage analysis
            covered_nodes = set()
            for t in range(T):
                if t < len(path_coords):
                    current_pos = path_coords[t]
                    for node in G:
                        dist = np.sqrt((current_pos[0] - node[0])**2 +
                                       (current_pos[1] - node[1])**2)
                        if dist <= sensing_radius:
                            covered_nodes.add(tuple(node))
            uav_covered_nodes[n] = list(covered_nodes)

            print(f"\nUAV {n+1}:")
            print(f"  Path: {path_coords}")
            print(f"  Nodes covered: {sorted(uav_covered_nodes[n])}")
            print(f"  Total nodes covered: {len(uav_covered_nodes[n])}")

            # Battery analysis
            if aux_tensor is not None:
                if aux_tensor.ndim == 3:
                    # expected format (T, N, vars_per_uav)
                    battery_levels[n] = [aux_tensor[t, n, 0] for t in range(T)]
                else:
                    print("  [!] aux_tensor is not 3D, skipping battery analysis")
                    battery_levels[n] = []
            else:
                battery_levels[n] = []


        else:
            print(f"\nUAV {n+1}: No path assigned")
            uav_paths[n] = []
            uav_covered_nodes[n] = []
            battery_levels[n] = []

    # Total coverage
    all_covered = set()
    for nodes in uav_covered_nodes.values():
        all_covered.update(nodes)
    print(f"\nTotal unique nodes covered: {len(all_covered)}")
    print(f"All covered nodes: {sorted(list(all_covered))}")

    return uav_paths, uav_covered_nodes, all_covered, battery_levels

def cplex_solver(Z1, lb, ub, Aineq, bineq, Aeq, beq,
                 ctype, row_size, col_size, N, T, Cmax,
                 time_limit=18000, mipgap=0.0):
    """
    Solve MILP using CPLEX with safe type casting for all inputs.
    """

    # ==============================
    # Setup CPLEX problem
    # ==============================
    prob = cplex.Cplex()
    prob.objective.set_sense(prob.objective.sense.minimize)

    # Convert vectors to Python floats
    obj = [float(x) for x in Z1]
    lb = [float(x) for x in lb]
    ub = [float(x) for x in ub]

    prob.variables.add(obj=obj, lb=lb, ub=ub, types=ctype)

    # ==============================
    # Add inequality constraints
    # ==============================
    for i in range(Aineq.shape[0]):
        row = [float(x) for x in Aineq[i]]
        rhs_val = float(bineq[i])
        prob.linear_constraints.add(
            lin_expr=[cplex.SparsePair(
                ind=list(range(len(row))),
                val=row
            )],
            senses=["L"],
            rhs=[rhs_val]
        )

    # ==============================
    # Add equality constraints
    # ==============================
    for i in range(Aeq.shape[0]):
        row = [float(x) for x in Aeq[i]]
        rhs_val = float(beq[i])
        prob.linear_constraints.add(
            lin_expr=[cplex.SparsePair(
                ind=list(range(len(row))),
                val=row
            )],
            senses=["E"],
            rhs=[rhs_val]
        )

    # ==============================
    # Solver options
    # ==============================
    prob.parameters.mip.tolerances.mipgap.set(float(mipgap))
    prob.parameters.timelimit.set(float(time_limit))  # seconds
    prob.parameters.mip.strategy.search.set(1)

    # ==============================
    # Solve
    # ==============================
    start_time = time.time()
    try:
        prob.solve()
    except CplexError as e:
        print(f"CPLEX Error: {e}")
        return None

    Cov_time = time.time() - start_time

    # ==============================
    # Extract solution
    # ==============================
    S = np.array(prob.solution.get_values(), dtype=float)
    fval = float(prob.solution.get_objective_value())

    # Round solution
    ss = np.round(S, 1)
    c = np.where(ss > 0)[0]
    path_loc = c[c > (1 + T*N) * row_size * col_size]
    U = ss[path_loc]
    V = np.column_stack((path_loc, U))

    # Route tracing
    cov_path = np.zeros((N, row_size*col_size*T), dtype=int)
    for n in range(N):
        P1 = []
        for t in range(T):
            for i in range(len(path_loc)):
                Rmin = (1 + T*N)*row_size*col_size + t*N*row_size*col_size + n*row_size*col_size
                Rmax = (1 + T*N)*row_size*col_size + row_size*col_size + t*N*row_size*col_size + n*row_size*col_size
                if (Rmax <= (1 + T*N + T*N)*row_size*col_size and
                        path_loc[i] > Rmin and path_loc[i] <= Rmax):
                    c1 = path_loc[i]
                    c2 = c1 - ((1 + T*N)*row_size*col_size +
                               t*N*row_size*col_size +
                               n*row_size*col_size)
                    P1.append(c2)
        if len(P1) > 0:
            cov_path[n, :len(P1)] = P1

    # Coverage percentage (negate because objective is -coverage)
    Cov_Percent = (-1) * (fval / Cmax) * 100.0

    return Cov_Percent, Cov_time, fval, cov_path, path_loc, S, V
