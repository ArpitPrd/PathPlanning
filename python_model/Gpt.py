import numpy as np


def _bresenham_line_cells(p, q):
    """
    Bresenham on a grid of (row, col) integer coordinates.
    Returns the list of grid cells on the line from p to q (inclusive).
    """
    r0, c0 = int(p[0]), int(p[1])
    r1, c1 = int(q[0]), int(q[1])

    cells = []
    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    sr = 1 if r1 >= r0 else -1
    sc = 1 if c1 >= c0 else -1

    if dc > dr:
        # shallow slope, step in c
        err = 2*dr - dc
        r = r0
        for c in range(c0, c1 + sc, sc):
            cells.append((r, c))
            if err >= 0:
                r += sr
                err -= 2*dc
            err += 2*dr
    else:
        # steep slope, step in r
        err = 2*dc - dr
        c = c0
        for r in range(r0, r1 + sr, sr):
            cells.append((r, c))
            if err >= 0:
                c += sc
                err -= 2*dr
            err += 2*dc
    return cells


def _los_clear(p_rc, q_rc, sz, obstacles_lin_set):
    """
    True if the open segment between p and q does NOT pass through any obstacle cell.
    p_rc, q_rc: (row, col), 0-based.
    We ignore endpoints (they are guaranteed non-obstacles in your model).
    """
    # cells along line (inclusive); drop endpoints
    cells = _bresenham_line_cells(p_rc, q_rc)
    if len(cells) <= 2:
        return True
    inner = cells[1:-1]

    # convert inner cells to linear indices and test membership
    for (r, c) in inner:
        lin = np.ravel_multi_index((r, c), sz)
        if lin in obstacles_lin_set:
            return False
    return True


def communicable_gpt(P_sink, G, sz, Rc, O_lin=None):
    """
    LoS-aware communication neighbors:
    only keep neighbors within Rc AND with an obstacle-free line segment.
    G: array of shape (M,2) with 0-based (row,col) valid cells (obstacles already removed).
    O_lin: iterable of obstacle linear indices (0-based). Can be None/empty.
    """
    obstacles_lin_set = set([] if O_lin is None else list(O_lin))

    # sink (0-based (r,c))
    sink_rc = np.array(np.unravel_index(P_sink, sz))

    Irc = []
    for p in range(len(G)):
        p_rc = (int(G[p, 0]), int(G[p, 1]))
        neigh_lin = []
        for q in range(len(G)):
            if q == p:
                continue
            q_rc = (int(G[q, 0]), int(G[q, 1]))
            # radius test
            if (p_rc[0] - q_rc[0])**2 + (p_rc[1] - q_rc[1])**2 <= Rc**2:
                # LoS test
                if _los_clear(p_rc, q_rc, sz, obstacles_lin_set):
                    neigh_lin.append(np.ravel_multi_index(q_rc, sz))
        # exclude sink linear index explicitly
        neigh_lin = sorted(set(neigh_lin) - {P_sink})
        Irc.append(neigh_lin)

    # find sink row in G (guaranteed present if not an obstacle)
    L = np.where((G == sink_rc).all(axis=1))[0][0]
    Irc_sink = Irc[L]

    return Irc, Irc_sink


def sensing_gpt(P_sink, G, sz, Rs, O_lin=None):
    """
    LoS-aware sensing neighbors:
    only keep neighbors within Rs AND with an obstacle-free line segment.
    Returns:
      L: index of sink row in G,
      Irs: list of lists of neighbor linear indices for each G row,
      Irs_sink: sensing neighbors of the sink (linear indices)
    """
    obstacles_lin_set = set([] if O_lin is None else list(O_lin))

    sink_rc = np.array(np.unravel_index(P_sink, sz))

    Irs = []
    for p in range(len(G)):
        p_rc = (int(G[p, 0]), int(G[p, 1]))
        neigh_lin = []
        for q in range(len(G)):
            if q == p:
                continue
            q_rc = (int(G[q, 0]), int(G[q, 1]))
            # radius test
            if (p_rc[0] - q_rc[0])**2 + (p_rc[1] - q_rc[1])**2 <= Rs**2:
                # LoS test
                if _los_clear(p_rc, q_rc, sz, obstacles_lin_set):
                    neigh_lin.append(np.ravel_multi_index(q_rc, sz))
        # exclude sink explicitly from generic lists if you want symmetry with communicable_gpt
        neigh_lin = sorted(set(neigh_lin) - {P_sink})
        Irs.append(neigh_lin)

    L = np.where((G == sink_rc).all(axis=1))[0][0]
    Irs_sink = Irs[L]

    return L, Irs, Irs_sink
