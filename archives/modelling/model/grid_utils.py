import numpy as np

def sub2ind(sz, row, col):
    """
    MATLAB-style column-major linear index:
    ind = row + (col-1)*Nx
    """
    Nx, Ny = sz
    return int(row + (col - 1) * Nx)

def ind2sub(sz, idx):
    """
    Inverse of sub2ind.
    """
    Nx, Ny = sz
    col = (idx - 1) // Nx + 1
    row = idx - (col - 1) * Nx
    return int(row), int(col)

def meshgrid_xy(x, y):
    """
    MATLAB-style meshgrid with x = rows, y = cols.
    """
    X, Y = np.meshgrid(y, x)
    return X, Y

def build_grid(x, y):
    """
    Build full grid of points.
    Returns:
      TG : array of [row, col] positions
      sz : (Nx, Ny)
    """
    X, Y = meshgrid_xy(x, y)
    Nx, Ny = len(x), len(y)
    TG = np.column_stack([Y.ravel(order='C'), X.ravel(order='C')]).astype(int)
    return TG, (Nx, Ny)

def sink_pos_from_index(sz, P_sink):
    """
    Get sink [row, col] from MATLAB-style linear index.
    """
    row, col = ind2sub(sz, P_sink)
    return np.array([row, col], dtype=int)

def compute_sets(G, P_sink, sz, Rs, Rc):
    """
    Build sensing and communication neighbor index sets like MATLAB Sensing_Gpt/Communicable_Gpt.
    Returns:
      L        : 0-based index of sink inside G
      Irs      : dict p -> sorted np.array of MATLAB linear indices (1-based) of sensing neighbors (sink excluded)
      Irs_sink : same as Irs[L]
      Irc      : dict p -> sorted np.array of MATLAB linear indices (1-based) of comm neighbors (sink excluded)
      Irc_sink : same as Irc[L]
    """
    G = np.asarray(G, dtype=int)
    Nx, Ny = sz
    sink_rc = sink_pos_from_index(sz, P_sink)  # [row, col]

    def dist2(p, q):
        dr = int(p[0]) - int(q[0])
        dc = int(p[1]) - int(q[1])
        return dr*dr + dc*dc

    Rs2 = float(Rs) * float(Rs)
    Rc2 = float(Rc) * float(Rc)

    Irs, Irc = {}, {}

    for p_idx in range(G.shape[0]):
        Gp = G[p_idx, :]  # [row, col]

        # Sensing neighbors
        s_pts = []
        for q_idx in range(G.shape[0]):
            Gq = G[q_idx, :]
            if dist2(Gp, Gq) <= Rs2:
                s_pts.append([int(Gq[0]), int(Gq[1])])
        if s_pts:
            s_unique = np.unique(np.array(s_pts, dtype=int), axis=0)
            s_lin = np.array([sub2ind(sz, r, c) for r, c in s_unique], dtype=int)
        else:
            s_lin = np.array([], dtype=int)

        Irs[p_idx] = np.sort(s_lin[s_lin != P_sink])

        # Communication neighbors
        c_pts = []
        for q_idx in range(G.shape[0]):
            Gq = G[q_idx, :]
            if dist2(Gp, Gq) <= Rc2:
                c_pts.append([int(Gq[0]), int(Gq[1])])
        if c_pts:
            c_unique = np.unique(np.array(c_pts, dtype=int), axis=0)
            c_lin = np.array([sub2ind(sz, r, c) for r, c in c_unique], dtype=int)
        else:
            c_lin = np.array([], dtype=int)

        Irc[p_idx] = np.sort(c_lin[c_lin != P_sink])

    # Locate sink inside G
    matches = np.where((G[:, 0] == sink_rc[0]) & (G[:, 1] == sink_rc[1]))[0]
    if matches.size == 0:
        raise ValueError("Sink not found in G")
    L = int(matches[0])

    Irs_sink = Irs[L]
    Irc_sink = Irc[L]

    return L, Irs, Irs_sink, Irc, Irc_sink


