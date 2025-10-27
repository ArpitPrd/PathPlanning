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
    sr, sc = 1 if r1 >= r0 else -1, 1 if c1 >= c0 else -1

    err = dc - dr
    while True:
        cells.append((r0, c0))
        if r0 == r1 and c0 == c1: break
        e2 = 2 * err
        if e2 >= -dr: err -= dr; c0 += sc
        if e2 <= dc: err += dc; r0 += sr
    return cells

def _los_clear(p_rc, q_rc, sz, obstacles_lin_set):
    """
    Checks for Line-of-Sight (LoS) between two points.
    Returns True if the segment between p and q does not pass through an obstacle.
    Endpoints are ignored as they are assumed to be valid locations.
    """
    cells = _bresenham_line_cells(p_rc, q_rc)
    if len(cells) <= 2: return True
    
    # Check inner cells for obstacles
    for (r, c) in cells[1:-1]:
        if np.ravel_multi_index((r, c), sz) in obstacles_lin_set:
            return False
    return True

def _get_neighbors(G, sz, R, O_lin_set, P_sink):
    """
    Generic helper to find LoS-aware neighbors within a given radius R.
    """
    sink_rc = np.array(np.unravel_index(P_sink, sz))
    I_neighbors = []
    
    for p in range(len(G)):
        p_rc = (int(G[p, 0]), int(G[p, 1]))
        neigh_lin = []
        for q in range(len(G)):
            if p == q: continue
            q_rc = (int(G[q, 0]), int(G[q, 1]))
            # Check Euclidean distance (squared)
            if (p_rc[0] - q_rc[0])**2 + (p_rc[1] - q_rc[1])**2 <= R**2:
                # Check Line of Sight
                if _los_clear(p_rc, q_rc, sz, O_lin_set):
                    neigh_lin.append(np.ravel_multi_index(q_rc, sz))
        I_neighbors.append(sorted(neigh_lin))
    
    # Find sink's neighbors by its linear index
    sink_neighbors = I_neighbors[P_sink]
    return I_neighbors, sink_neighbors

def communicable_gpt(P_sink, G, sz, Rc, O_lin=None):
    """
    Finds communicable neighbors for all grid cells.
    A cell q is a communicable neighbor of p if it is within radius Rc and has LoS.
    
    Returns:
        I_neighbors (list of lists): I_neighbors[i] is the set C_i
        sink_neighbors (list): The set C_s, required for constraint (4)
    """
    O_lin_set = set(O_lin or [])
    return _get_neighbors(G, sz, Rc, O_lin_set, P_sink)

def sensing_gpt(P_sink, G, sz, Rs, O_lin=None):
    """
    Finds sensing neighbors for all grid cells.
    A cell q is a sensing neighbor of p if it is within radius Rs and has LoS.
    
    Returns:
        Irs (list of lists): Irs[i] is the set S_i, required for constraint (13)
    """
    O_lin_set = set(O_lin or [])
    # _get_neighbors returns two values; we only need the first one (Irs)
    # The MILP constraints for coverage (13, 14, 15) are defined for i in G_s_bar
    # (all points *except* the sink), so the entry for Irs[P_sink] will
    # simply not be used by those constraints.
    Irs, _ = _get_neighbors(G, sz, Rs, O_lin_set, P_sink)
    return Irs
