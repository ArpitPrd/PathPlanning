import numpy as np
from .grid_utils import ind2sub

def extract_paths(model, x):
    """
    Extract agent coverage paths from the solver decision vector x.
    
    Returns:
        cov_path : ndarray of shape (N, T)
                   Each entry is a MATLAB-style linear index (1-based)
                   for the grid cell occupied by agent n at timestep t.
                   Empty (0) means no position assigned.
    """
    Nx, Ny = model['sz']
    N, T = model['N'], model['T']
    nvars_ci = model['nvars_ci']
    nvars_ct = model['nvars_ct']
    nvars_zn = model['nvars_zn']
    total = model['total']

    ss = np.round(x, 1)
    c_idx = np.where(ss > 0.0)[0]
    # keep only z-variable indices
    path_loc = c_idx[c_idx >= (nvars_ci + nvars_ct)]  

    cov_path = np.zeros((N, T), dtype=int)

    for n in range(N):          # agent index (0-based)
        for t in range(T):      # timestep index (0-based)
            Rmin = nvars_ci + nvars_ct + t*N*Nx*Ny + n*Nx*Ny
            Rmax = Rmin + Nx*Ny
            hits = [c1 - (nvars_ci + nvars_ct + t*N*Nx*Ny + n*Nx*Ny)
                    for c1 in path_loc if Rmin <= c1 < Rmax]
            if hits:
                # assume only one active position per (n,t)
                # store as 1-based index for MATLAB consistency
                cov_path[n, t] = int(hits[0]) + 1
    return cov_path


def path_total_distance(cov_path, sz):
    """
    Compute the total travel distance of all agents.
    Distance is measured in Euclidean units on the grid.
    
    Args:
        cov_path : ndarray (N, T)
                   Each entry is a 1-based linear index (MATLAB style).
        sz       : tuple (Nx, Ny), grid size
    """
    Nx, Ny = sz
    cost = 0.0
    for n in range(cov_path.shape[0]):  # over agents
        indices = [int(i) for i in cov_path[n, :] if i > 0]
        if len(indices) <= 1:
            continue
        coords = [ind2sub(sz, idx) for idx in indices]
        for i in range(len(coords) - 1):
            (r1, c1), (r2, c2) = coords[i], coords[i + 1]
            dist = ((r2 - r1) ** 2 + (c2 - c1) ** 2) ** 0.5
            cost += dist
    return cost


def coverage_percent(fval, Cmax):
    """
    Compute percentage coverage from solver objective value.
    Assumes optimization minimizes -coverage.
    
    Args:
        fval : objective function value (negative of coverage)
        Cmax : maximum possible coverage
    """
    return (-fval / float(Cmax)) * 100.0
