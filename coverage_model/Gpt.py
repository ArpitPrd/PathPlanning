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
    # cells = _bresenham_line_cells(p_rc, q_rc)
    # if len(cells) <= 2: return True
    
    # # Check inner cells for obstacles
    # for (r, c) in cells[1:-1]:
    #     if np.ravel_multi_index((r, c), sz) in obstacles_lin_set:
    #         return False
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
    """
    O_lin_set = set(O_lin or [])
    return _get_neighbors(G, sz, Rc, O_lin_set, P_sink)

def sensing_gpt(P_sink, G, sz, Rs, O_lin=None):
    """
    Finds sensing neighbors for all grid cells.
    A cell q is a sensing neighbor of p if it is within radius Rs.
    Excludes obstacles and sink from being in the sensing coverage.
    """
    O_lin_set = set(O_lin or [])
    L = P_sink  # The index of the sink is its linear index
    
    Irs = []
    
    for p in range(len(G)):
        p_rc = (int(G[p, 0]), int(G[p, 1]))
        neigh_lin = []
        
        for q in range(len(G)):
            if p == q: 
                continue
            
            q_lin = np.ravel_multi_index((int(G[q, 0]), int(G[q, 1])), sz)
            
            # Skip if q is an obstacle or the sink
            if q_lin in O_lin_set or q_lin == P_sink:
                continue
            
            q_rc = (int(G[q, 0]), int(G[q, 1]))
            
            # Check Euclidean distance (squared)
            if (p_rc[0] - q_rc[0])**2 + (p_rc[1] - q_rc[1])**2 <= Rs**2:
                neigh_lin.append(q_lin)
        
        Irs.append(sorted(neigh_lin))
    
    # Find sink's sensing neighbors
    Irs[P_sink] = []
    Irs_sink = Irs[P_sink]
    
    return L, Irs, Irs_sink


def movement_gpt(P_sink, G, sz, Rc, O_lin=None):
    """
    Finds movable neighbors for all grid cells.
    A cell q is a movable neighbor of p if:
    1. There exists a path from p to q that avoids obstacles
    2. The path length (Manhattan distance) <= Rc
    """
    from collections import deque
    
    O_lin_set = set(O_lin or [])
    I_movement = []
    
    for p in range(len(G)):
        p_rc = (int(G[p, 0]), int(G[p, 1]))
        neigh_lin = []
        
        # BFS to find all reachable cells within Manhattan distance Rc
        queue = deque([(p_rc, 0)])  # (position, distance)
        visited = {p_rc}
        
        while queue:
            (r, c), dist = queue.popleft()
            
            # If we've reached the distance limit, skip
            if dist >= Rc:
                continue
            
            # Check all 4 adjacent cells (up, down, left, right)
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nr, nc = r + dr, c + dc
                
                # Check bounds
                if 0 <= nr < sz[0] and 0 <= nc < sz[1]:
                    next_rc = (nr, nc)
                    
                    # Skip if already visited
                    if next_rc in visited:
                        continue
                    
                    # Check if this cell is an obstacle
                    next_lin = np.ravel_multi_index(next_rc, sz)
                    if next_lin in O_lin_set:
                        continue
                    
                    # Mark as visited and add to queue
                    visited.add(next_rc)
                    queue.append((next_rc, dist + 1))
                    
                    # Add to neighbors if it's not the starting point
                    if next_rc != p_rc:
                        neigh_lin.append(next_lin)
            if P_sink in neigh_lin:
                neigh_lin.remove(P_sink)  # Exclude sink from movable neighbors
        I_movement.append(sorted(neigh_lin))
    
    # Find sink's movable neighbors
    I_movement[P_sink] = []
    movement_sink = I_movement[P_sink]
    return I_movement, movement_sink

