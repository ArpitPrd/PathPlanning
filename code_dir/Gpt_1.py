import numpy as np

def _bresenham_line_cells(p, q):
    """
    Calculates grid cells for a line from p to q using Bresenham's algorithm.
    Coordinates are (row, col) integers. Returns a list of (r, c) tuples.
    """
    r0, c0 = int(p[0]), int(p[1])
    r1, c1 = int(q[0]), int(q[1])

    cells = []
    dr, dc = abs(r1 - r0), abs(c1 - c0)
    sr, sc = (1 if r1 >= r0 else -1), (1 if c1 >= c0 else -1)

    # This form matches a standard integer Bresenham and works for all octants.
    err = dc - dr
    while True:
        cells.append((r0, c0))
        if r0 == r1 and c0 == c1:
            break
        e2 = 2 * err
        if e2 >= -dr:
            err -= dr
            c0 += sc
        if e2 <= dc:
            err += dc
            r0 += sr
    return cells


def _los_clear(p_rc, q_rc, sz, obstacles_lin_set):
    """
    True if the open segment between p and q does NOT pass through any obstacle cell.
    Endpoints are ignored (assumed valid).
    """
    cells = _bresenham_line_cells(p_rc, q_rc)
    if len(cells) <= 2:
        return True
    # check inner cells only
    for (r, c) in cells[1:-1]:
        if np.ravel_multi_index((r, c), sz) in obstacles_lin_set:
            return False
    return True


def communicable_gpt(P_sink, G, sz, Rc, O_lin=None):
    """
    LoS-aware communication neighbors:
    only keep neighbors within Rc AND with an obstacle-free line segment.

    Inputs:
      P_sink: linear index of the sink (0-based)
      G: (M,2) array of all (row,col) integer grid cells
      sz: grid shape tuple (rows, cols)
      Rc: communication radius (in grid units)
      O_lin: iterable of obstacle linear indices (0-based). Can be None/empty.

    Returns:
      Irc: list of lists; for each G row, the sorted linear indices of its neighbors
      Irc_sink: neighbors (linear indices) of the sink row in G (includes sink)
    """
    obstacles_lin_set = set([] if O_lin is None else list(O_lin))
    sink_rc = np.array(np.unravel_index(P_sink, sz))
    
    Irc = []
    for p in range(len(G)):
        p_rc = (int(G[p, 0]), int(G[p, 1]))
        # A node cannot be its own neighbor if it's an obstacle
        if np.ravel_multi_index(p_rc, sz) in obstacles_lin_set:
            Irc.append([])
            continue
            
        neigh_lin = []
        for q in range(len(G)):
            q_rc = (int(G[q, 0]), int(G[q, 1]))
            
            # --- BUG FIX 1: Removed `if q == p: continue` ---
            # A node is always in its own communication range.
            
            # radius test (squared Euclidean)
            if (p_rc[0] - q_rc[0])**2 + (p_rc[1] - q_rc[1])**2 <= Rc**2:
                # LoS test
                if _los_clear(p_rc, q_rc, sz, obstacles_lin_set):
                    neigh_lin.append(np.ravel_multi_index(q_rc, sz))
                    
        # --- BUG FIX 2: Removed `... - {P_sink}` ---
        # Filtering the sink here breaks mobility and connectivity logic.
        # Constraint functions are responsible for special handling of the sink.
        Irc.append(sorted(list(set(neigh_lin))))

    # find sink row in G
    L = np.where((G == sink_rc).all(axis=1))[0][0]
    Irc_sink = Irc[L] # This list now correctly includes the sink itself

    return Irc, Irc_sink


def sensing_gpt(P_sink, G, sz, Rs, O_lin=None):
    """
    LoS-aware sensing neighbors:
    only keep neighbors within Rs AND with an obstacle-free line segment.

    Returns:
      L: index of the sink row in G,
      Irs: list of lists of neighbor linear indices for each G row (includes self)
      Irs_sink: sensing neighbors of the sink (linear indices)
    """
    obstacles_lin_set = set([] if O_lin is None else list(O_lin))
    sink_rc = np.array(np.unravel_index(P_sink, sz))

    Irs = []
    for p in range(len(G)):
        p_rc = (int(G[p, 0]), int(G[p, 1]))
        # A node cannot be its own neighbor if it's an obstacle
        if np.ravel_multi_index(p_rc, sz) in obstacles_lin_set:
            Irs.append([])
            continue
            
        neigh_lin = []
        for q in range(len(G)):
            q_rc = (int(G[q, 0]), int(G[q, 1]))
            
            # --- BUG FIX 1: Removed `if q == p: continue` ---
            # A node can always sense its own grid cell.
            
            # radius test (squared Euclidean)
            if (p_rc[0] - q_rc[0])**2 + (p_rc[1] - q_rc[1])**2 <= Rs**2:
                if _los_clear(p_rc, q_rc, sz, obstacles_lin_set):
                    neigh_lin.append(np.ravel_multi_index(q_rc, sz))
                    
        # --- BUG FIX 2: Removed `... - {P_sink}` ---
        Irs.append(sorted(list(set(neigh_lin))))

    L = np.where((G == sink_rc).all(axis=1))[0][0]
    Irs_sink = Irs[L]

    return L, Irs, Irs_sink