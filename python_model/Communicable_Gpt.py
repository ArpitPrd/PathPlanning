# Hey copilot, can you help me write the python code for the matlab code written below

import numpy as np

def communicable_gpt(P_sink, G, sz, Rc):
    """
    Python equivalent of the MATLAB Communicable_Gpt function.

    Parameters:
    -----------
    P_sink : int
        Linear index of sink node (row-major order).
    G : ndarray of shape (N,2)
        Grid coordinates of valid nodes (row, col).
    sz : tuple (Nx, Ny)
        Size of the grid.
    Rc : float
        Communication radius.

    Returns:
    --------
    Irc : list of lists
        Each entry contains the indices of neighbors for that node (excluding sink).
    Irc_sink : list
        Communication neighbors of the sink.
    """

    # Convert sink index back to (row, col)
    sink = np.unravel_index(P_sink, sz)  # gives (row, col)
    sink = np.array(sink)

    Irc = []
    for p in range(len(G)):
        Gpt = G[p]
        neighbors = []
        for q in range(len(G)):
            v = (Gpt[0] - G[q,0])**2 + (Gpt[1] - G[q,1])**2
            if v <= Rc**2:
                neighbors.append(G[q])
        neighbors = np.unique(np.array(neighbors), axis=0)

        # Convert coordinates to linear indices
        indices = [np.ravel_multi_index((r, c), sz) for r, c in neighbors]

        # Exclude sink index
        indices = sorted(set(indices) - {P_sink})
        Irc.append(indices)

    # Find sink in updated network
    L = np.where((G == sink).all(axis=1))[0][0]
    Irc_sink = Irc[L]

    return Irc, Irc_sink



