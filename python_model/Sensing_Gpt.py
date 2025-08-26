import numpy as np

def sensing_gpt(P_sink, G, sz, Rs):
    """
    Python equivalent of the MATLAB Sensing_Gpt function.

    Parameters
    ----------
    P_sink : int
        Linear index of the sink node (row-major order in Python).
    G : ndarray of shape (N,2)
        Valid node coordinates (row, col).
    sz : tuple (Nx, Ny)
        Grid size.
    Rs : float
        Sensing radius.

    Returns
    -------
    L : int
        Index of sink node inside G (0-based).
    Irs : list of lists
        Each entry contains linear indices of nodes that a given node can sense (excluding sink).
    Irs_sink : list
        List of linear indices of nodes that sink can sense.
    """

    # Convert sink index back to (row, col)
    sink = np.unravel_index(P_sink, sz)  # returns (row, col)
    sink = np.array(sink)

    Irs = []
    for p in range(len(G)):
        Gpt = G[p]
        neighbors = []
        for q in range(len(G)):
            v = (Gpt[0] - G[q, 0])**2 + (Gpt[1] - G[q, 1])**2
            if v <= Rs**2:
                neighbors.append(G[q])
        neighbors = np.unique(np.array(neighbors), axis=0)

        # Convert coordinates to linear indices
        indices = [np.ravel_multi_index((r, c), sz) for r, c in neighbors]

        # Exclude sink index
        indices = sorted(set(indices) - {P_sink})
        Irs.append(indices)

    # Find sink position inside G
    L = np.where((G == sink).all(axis=1))[0][0]
    Irs_sink = Irs[L]

    return L, Irs, Irs_sink
