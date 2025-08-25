import numpy as np
import scipy.sparse as sp
from scipy.optimize import linprog
import time
import matplotlib.pyplot as plt
from typing import Tuple, List, Dict, Optional
import cvxpy as cp


class UAVPathPlanner:
    """
    Multi-UAV Path Planning for Sweep Coverage with Connectivity Constraints
    Python implementation of MATLAB MILP formulation
    """
    
    def __init__(self, grid_size: Tuple[int, int] = (12, 12), 
                 sensing_radius: float = np.sqrt(2), 
                 communication_radius: float = None,
                 n_uavs: int = 3, 
                 time_steps: int = 10,
                 sink_position: int = 79,
                 obstacles: Optional[List[List[int]]] = None):
        """
        Initialize the UAV path planner
        
        Args:
            grid_size: Tuple of (Nx, Ny) grid dimensions
            sensing_radius: Sensing radius of UAVs
            communication_radius: Communication radius (default: 2 * sensing_radius)
            n_uavs: Number of UAVs
            time_steps: Maximum number of time steps
            sink_position: Index position of sink
            obstacles: List of obstacle positions [[x1,y1], [x2,y2], ...]
        """
        self.Nx, self.Ny = grid_size
        self.Rs = sensing_radius
        self.Rc = communication_radius if communication_radius else 2 * sensing_radius
        self.N = n_uavs
        self.T = time_steps
        self.P_sink = sink_position
        self.obstacles = obstacles if obstacles else []
        
        # Initialize grid and matrices
        self._setup_grid()
        self._compute_communication_matrix()
        self._compute_sensing_matrix()
    
    def _setup_grid(self):
        """Setup the grid and target points"""
        x = np.arange(1, self.Nx + 1)
        y = np.arange(1, self.Ny + 1)
        
        # Create meshgrid - note MATLAB uses (Y,X) format
        Y, X = np.meshgrid(y, x, indexing='ij')
        self.TG = np.column_stack([Y.ravel(), X.ravel()])
        
        # Convert sink position to 2D coordinates
        row, col = np.unravel_index(self.P_sink - 1, (self.Nx, self.Ny))  # -1 for 0-indexing
        self.sink = np.array([row + 1, col + 1])  # +1 to match MATLAB 1-indexing
        
        # Handle obstacles
        if self.obstacles:
            obstacle_array = np.array(self.obstacles)
            mask = ~np.array([np.any(np.all(self.TG == obs, axis=1)) for obs in obstacle_array])
            self.G = self.TG[mask]
            self.G1 = np.array([np.ravel_multi_index([g[0]-1, g[1]-1], (self.Nx, self.Ny)) + 1 
                               for g in self.G])  # Convert to linear indices (1-indexed)
            self.Cmax = self.Nx * self.Ny - len(self.obstacles) - 1
        else:
            self.G = self.TG.copy()
            self.G1 = np.arange(1, self.Nx * self.Ny + 1)  # 1-indexed linear indices
            self.Cmax = self.Nx * self.Ny - 1
    
    def _euclidean_distance_squared(self, p1: np.ndarray, p2: np.ndarray) -> float:
        """Calculate squared Euclidean distance between two points"""
        return np.sum((p1 - p2) ** 2)
    
    def _compute_communication_matrix(self):
        """Compute communication connectivity matrix"""
        self.Irc = {}
        
        for p in range(len(self.G)):
            communicable_points = []
            current_point = self.G[p]
            
            for q in range(len(self.G)):
                dist_sq = self._euclidean_distance_squared(current_point, self.G[q])
                if dist_sq <= self.Rc ** 2:
                    # Convert to linear index (1-indexed to match MATLAB)
                    idx = np.ravel_multi_index([self.G[q][0]-1, self.G[q][1]-1], (self.Nx, self.Ny)) + 1
                    communicable_points.append(idx)
            
            # Remove sink from communicable points for this location
            communicable_points = [idx for idx in communicable_points if idx != self.P_sink]
            self.Irc[p] = sorted(communicable_points)
        
        # Find sink position in G array
        sink_idx = None
        for i, point in enumerate(self.G):
            if np.array_equal(point, self.sink):
                sink_idx = i
                break
        
        if sink_idx is not None:
            self.L = sink_idx
            self.Irc_sink = self.Irc[sink_idx]
        else:
            raise ValueError("Sink position not found in grid")
    
    def _compute_sensing_matrix(self):
        """Compute sensing connectivity matrix"""
        self.Irs = {}
        
        for p in range(len(self.G)):
            sensable_points = []
            current_point = self.G[p]
            
            for q in range(len(self.G)):
                dist_sq = self._euclidean_distance_squared(current_point, self.G[q])
                if dist_sq <= self.Rs ** 2:
                    # Convert to linear index (1-indexed to match MATLAB)
                    idx = np.ravel_multi_index([self.G[q][0]-1, self.G[q][1]-1], (self.Nx, self.Ny)) + 1
                    sensable_points.append(idx)
            
            # Remove sink from sensable points for this location
            sensable_points = [idx for idx in sensable_points if idx != self.P_sink]
            self.Irs[p] = sorted(sensable_points)
        
        self.Irs_sink = self.Irs[self.L]
    
    def _create_objective_function_movement(self):
        """Create objective function for movement optimization"""
        total_vars = (1 + 2 * self.N * self.T) * self.Nx * self.Ny
        ff = np.zeros(total_vars)
        
        for t in range(self.T):
            for n in range(self.N):
                for g_idx in self.G1:
                    var_idx = (1 + self.T * self.N) * self.Nx * self.Ny + \
                             t * self.N * self.Nx * self.Ny + \
                             n * self.Nx * self.Ny + g_idx - 1
                    if var_idx < total_vars:
                        ff[var_idx] = 1
                
                # Set sink coefficient to 0
                sink_idx = (1 + self.T * self.N) * self.Nx * self.Ny + \
                          t * self.N * self.Nx * self.Ny + \
                          n * self.Nx * self.Ny + self.P_sink - 1
                if sink_idx < total_vars:
                    ff[sink_idx] = 0
        
        return ff
    
    def _create_objective_function_coverage(self):
        """Create objective function for coverage optimization"""
        total_vars = (1 + 2 * self.N * self.T) * self.Nx * self.Ny
        f = np.zeros(total_vars)
        
        for g_idx in self.G1:
            if g_idx - 1 < len(f):
                f[g_idx - 1] = 1
        
        # Set sink coefficient to 0
        if self.P_sink - 1 < len(f):
            f[self.P_sink - 1] = 0
        
        return -f  # Negative for maximization
    
    def _create_position_constraints(self):
        """Create position constraints"""
        total_vars = (1 + 2 * self.N * self.T) * self.Nx * self.Ny
        
        # Constraint F: one UAV at single point
        F = []
        for t in range(self.T):
            for n in range(self.N):
                constraint = np.zeros(total_vars)
                for g_idx in self.G1:
                    var_idx = (1 + self.T * self.N) * self.Nx * self.Ny + \
                             t * self.N * self.Nx * self.Ny + \
                             n * self.Nx * self.Ny + g_idx - 1
                    if var_idx < total_vars:
                        constraint[var_idx] = 1
                
                # Set sink coefficient to 0
                sink_idx = (1 + self.T * self.N) * self.Nx * self.Ny + \
                          t * self.N * self.Nx * self.Ny + \
                          n * self.Nx * self.Ny + self.P_sink - 1
                if sink_idx < total_vars:
                    constraint[sink_idx] = 0
                
                F.append(constraint)
        
        # Constraint E: single UAV at one point
        E = []
        for t in range(self.T):
            for p in range(len(self.G)):
                constraint = np.zeros(total_vars)
                if p != self.L:  # Skip sink location
                    for n in range(self.N):
                        var_idx = (1 + self.T * self.N) * self.Nx * self.Ny + \
                                 t * self.N * self.Nx * self.Ny + \
                                 n * self.Nx * self.Ny + self.G1[p] - 1
                        if var_idx < total_vars:
                            constraint[var_idx] = 1
                E.append(constraint)
        
        return np.array(F), np.array(E)
    
    def _create_connectivity_constraints(self):
        """Create connectivity constraints"""
        total_vars = (1 + 2 * self.N * self.T) * self.Nx * self.Ny
        
        # Constraint g: Sink connectivity
        g = []
        for t in range(self.T):
            constraint = np.zeros(total_vars)
            for n in range(self.N):
                for comm_idx in self.Irc_sink:
                    var_idx = (1 + self.T * self.N) * self.Nx * self.Ny + \
                             t * self.N * self.Nx * self.Ny + \
                             n * self.Nx * self.Ny + comm_idx - 1
                    if var_idx < total_vars:
                        constraint[var_idx] = -1
            g.append(constraint)
        
        # Constraint H: Inter-UAV connectivity
        H = []
        for t in range(self.T):
            for n in range(1, self.N):  # n=2:N in MATLAB (1-indexed)
                for p in range(len(self.G)):
                    constraint = np.zeros(total_vars)
                    if p != self.L:  # Skip sink location
                        # Current UAV position
                        var_idx = self.Nx * self.Ny + self.T * self.N * self.Nx * self.Ny + \
                                 t * self.N * self.Nx * self.Ny + \
                                 n * self.Nx * self.Ny + self.G1[p] - 1
                        if var_idx < total_vars:
                            constraint[var_idx] = 1
                        
                        # Communicable positions of previous UAVs
                        for comm_idx in self.Irc[p]:
                            prev_var_idx = self.Nx * self.Ny + self.N * self.T * self.Nx * self.Ny + \
                                          t * self.N * self.Nx * self.Ny + \
                                          (n - 1) * self.Nx * self.Ny + comm_idx - 1
                            if prev_var_idx < total_vars:
                                constraint[prev_var_idx] = -1
                    H.append(constraint)
        
        return np.array(g), np.array(H)
    
    def _create_mobility_constraints(self):
        """Create mobility constraints"""
        total_vars = (1 + 2 * self.N * self.T) * self.Nx * self.Ny
        I = []
        
        for t in range(self.T - 1):  # t=1:(T-1)
            for n in range(self.N):
                for p in range(len(self.G)):
                    constraint = np.zeros(total_vars)
                    if p != self.L:  # Skip sink location
                        # Current time step
                        var_idx = self.Nx * self.Ny + self.N * self.T * self.Nx * self.Ny + \
                                 (t + 1) * self.N * self.Nx * self.Ny + \
                                 n * self.Nx * self.Ny + self.G1[p] - 1
                        if var_idx < total_vars:
                            constraint[var_idx] = 1
                        
                        # Previous time step communicable positions
                        for comm_idx in self.Irc[p]:
                            prev_var_idx = self.Nx * self.Ny + self.N * self.T * self.Nx * self.Ny + \
                                          t * self.N * self.Nx * self.Ny + \
                                          n * self.Nx * self.Ny + comm_idx - 1
                            if prev_var_idx < total_vars:
                                constraint[prev_var_idx] = -1
                    I.append(constraint)
        
        return np.array(I)
    
    def _create_coverage_constraints(self):
        """Create coverage constraints"""
        total_vars = (1 + 2 * self.N * self.T) * self.Nx * self.Ny
        
        # Constraint K
        K = []
        for t in range(self.T):
            for n in range(self.N):
                for p in range(len(self.G)):
                    constraint = np.zeros(total_vars)
                    if p != self.L:  # Skip sink location
                        # UAV position variable
                        var_idx = self.Nx * self.Ny + \
                                 t * self.N * self.Nx * self.Ny + \
                                 n * self.Nx * self.Ny + self.G1[p] - 1
                        if var_idx < total_vars:
                            constraint[var_idx] = 1
                        
                        # Sensing positions
                        for sense_idx in self.Irs[p]:
                            sense_var_idx = self.Nx * self.Ny + self.N * self.T * self.Nx * self.Ny + \
                                           t * self.N * self.Nx * self.Ny + \
                                           n * self.Nx * self.Ny + sense_idx - 1
                            if sense_var_idx < total_vars:
                                constraint[sense_var_idx] = -1
                    K.append(constraint)
        
        # Constraint J
        J = []
        for t in range(self.T):
            for n in range(self.N):
                for q in range(len(self.G)):
                    constraint = np.zeros(total_vars)
                    # Overall coverage variable
                    if self.G1[q] - 1 < total_vars:
                        constraint[self.G1[q] - 1] = -1
                    if self.G1[self.L] - 1 < total_vars:
                        constraint[self.G1[self.L] - 1] = 0
                    
                    # Individual coverage variable
                    var_idx = self.Nx * self.Ny + \
                             t * self.N * self.Nx * self.Ny + \
                             n * self.Nx * self.Ny + self.G1[q] - 1
                    if var_idx < total_vars:
                        constraint[var_idx] = 1
                    
                    sink_var_idx = self.Nx * self.Ny + \
                                  t * self.N * self.Nx * self.Ny + \
                                  n * self.Nx * self.Ny + self.G1[self.L] - 1
                    if sink_var_idx < total_vars:
                        constraint[sink_var_idx] = 0
                    
                    J.append(constraint)
        
        # Constraint Q
        Q = []
        for q in range(len(self.G)):
            constraint = np.zeros(total_vars)
            # Overall coverage variable
            if self.G1[q] - 1 < total_vars:
                constraint[self.G1[q] - 1] = 1
            if self.G1[self.L] - 1 < total_vars:
                constraint[self.G1[self.L] - 1] = 0
            
            # Sum over all time steps and UAVs
            for t in range(self.T):
                for n in range(self.N):
                    var_idx = self.Nx * self.Ny + \
                             t * self.N * self.Nx * self.Ny + \
                             n * self.Nx * self.Ny + self.G1[q] - 1
                    if var_idx < total_vars:
                        constraint[var_idx] = -1
                    
                    sink_var_idx = self.Nx * self.Ny + \
                                  t * self.N * self.Nx * self.Ny + \
                                  n * self.Nx * self.Ny + self.G1[self.L] - 1
                    if sink_var_idx < total_vars:
                        constraint[sink_var_idx] = 0
            Q.append(constraint)
        
        return np.array(K), np.array(J), np.array(Q)
    
    def optimize_movement(self, coverage_ratio: float = 1.0):
        """
        Optimize for minimum movement with connectivity constraints
        
        Args:
            coverage_ratio: Desired coverage ratio (0-1)
            
        Returns:
            Dictionary containing solution and metrics
        """
        print("Setting up movement optimization problem...")
        
        total_vars = (1 + 2 * self.N * self.T) * self.Nx * self.Ny
        
        # Create objective function
        c = self._create_objective_function_movement()
        
        # Create constraints
        F, E = self._create_position_constraints()
        g, H = self._create_connectivity_constraints()
        I = self._create_mobility_constraints()
        K, J, Q = self._create_coverage_constraints()
        
        # Create coverage constraint
        W = np.zeros(total_vars)
        for g_idx in self.G1:
            if g_idx - 1 < total_vars:
                W[g_idx - 1] = -1
        if self.G1[self.L] - 1 < total_vars:
            W[self.G1[self.L] - 1] = 0
        
        # Combine inequality constraints
        A_ineq = np.vstack([E, g, H, I, J, Q, W.reshape(1, -1)])
        b_ineq = np.concatenate([
            np.ones(E.shape[0]),
            -np.ones(g.shape[0]),
            np.zeros(H.shape[0]),
            np.zeros(I.shape[0]),
            np.zeros(J.shape[0]),
            np.zeros(Q.shape[0]),
            [-coverage_ratio * self.Cmax]
        ])
        
        # Equality constraints
        A_eq = K
        b_eq = np.zeros(K.shape[0])
        
        # Bounds
        bounds = [(0, 1) for _ in range(total_vars)]
        
        print(f"Problem size: {total_vars} variables, {A_ineq.shape[0]} inequalities, {A_eq.shape[0]} equalities")
        
        # Solve using CVXPY for better integer programming support
        x = cp.Variable(total_vars)
        
        # Binary variables for the last part (UAV positions)
        binary_start = (1 + self.T * self.N) * self.Nx * self.Ny
        for i in range(binary_start, total_vars):
            x[i] = cp.Variable(boolean=True)
        
        objective = cp.Minimize(c.T @ x)
        constraints = [
            A_ineq @ x <= b_ineq,
            A_eq @ x == b_eq,
            x >= 0,
            x <= 1
        ]
        
        prob = cp.Problem(objective, constraints)
        
        start_time = time.time()
        try:
            prob.solve(solver=cp.GUROBI, verbose=True)
            if prob.status not in ["optimal", "optimal_inaccurate"]:
                print(f"Solver status: {prob.status}")
                return None
        except:
            try:
                prob.solve(solver=cp.SCIP, verbose=True)
            except:
                prob.solve(verbose=True)
        
        solve_time = time.time() - start_time
        
        if x.value is None:
            print("No solution found")
            return None
        
        solution = self._extract_solution(x.value)
        solution['solve_time'] = solve_time
        solution['objective_value'] = prob.value
        
        return solution
    
    def optimize_coverage(self):
        """
        Optimize for maximum coverage with connectivity constraints
        
        Returns:
            Dictionary containing solution and metrics
        """
        print("Setting up coverage optimization problem...")
        
        total_vars = (1 + 2 * self.N * self.T) * self.Nx * self.Ny
        
        # Create objective function (negative for maximization)
        c = self._create_objective_function_coverage()
        
        # Create constraints
        F, E = self._create_position_constraints()
        g, H = self._create_connectivity_constraints()
        I = self._create_mobility_constraints()
        K, J, Q = self._create_coverage_constraints()
        
        # Combine inequality constraints
        A_ineq = np.vstack([E, g, H, I, J, Q])
        b_ineq = np.concatenate([
            np.ones(E.shape[0]),
            -np.ones(g.shape[0]),
            np.zeros(H.shape[0]),
            np.zeros(I.shape[0]),
            np.zeros(J.shape[0]),
            np.zeros(Q.shape[0])
        ])
        
        # Equality constraints
        A_eq = np.vstack([F, K])
        b_eq = np.concatenate([
            np.ones(F.shape[0]),
            np.zeros(K.shape[0])
        ])
        
        # Bounds
        bounds = [(0, 1) for _ in range(total_vars)]
        
        print(f"Problem size: {total_vars} variables, {A_ineq.shape[0]} inequalities, {A_eq.shape[0]} equalities")
        
        # Solve using CVXPY
        x = cp.Variable(total_vars)
        
        # Binary variables for the last part (UAV positions)
        binary_start = (1 + self.T * self.N) * self.Nx * self.Ny
        for i in range(binary_start, total_vars):
            x[i] = cp.Variable(boolean=True)
        
        objective = cp.Minimize(c.T @ x)  # Already negative for maximization
        constraints = [
            A_ineq @ x <= b_ineq,
            A_eq @ x == b_eq,
            x >= 0,
            x <= 1
        ]
        
        prob = cp.Problem(objective, constraints)
        
        start_time = time.time()
        try:
            prob.solve(solver=cp.GUROBI, verbose=True)
            if prob.status not in ["optimal", "optimal_inaccurate"]:
                print(f"Solver status: {prob.status}")
                return None
        except:
            try:
                prob.solve(solver=cp.SCIP, verbose=True)
            except:
                prob.solve(verbose=True)
        
        solve_time = time.time() - start_time
        
        if x.value is None:
            print("No solution found")
            return None
        
        solution = self._extract_solution(x.value)
        solution['solve_time'] = solve_time
        solution['objective_value'] = prob.value
        solution['coverage_percent'] = (-prob.value / self.Cmax) * 100
        
        return solution
    
    def _extract_solution(self, solution_vector):
        """Extract UAV paths and coverage from solution vector"""
        ss = np.round(solution_vector, 1)
        
        # Find active variables
        active_indices = np.where(ss > 0.5)[0]
        
        # Extract path locations (variables after coverage variables)
        path_start = (1 + self.T * self.N) * self.Nx * self.Ny
        path_locations = [idx for idx in active_indices if idx >= path_start]
        
        # Trace routes for each UAV
        uav_paths = {}
        for n in range(self.N):
            path_for_uav = []
            for t in range(self.T):
                for path_idx in path_locations:
                    # Calculate range for this UAV and time step
                    range_min = path_start + t * self.N * self.Nx * self.Ny + n * self.Nx * self.Ny
                    range_max = range_min + self.Nx * self.Ny
                    
                    if range_min <= path_idx < range_max:
                        # Convert back to grid position
                        linear_pos = path_idx - range_min + 1  # +1 for 1-indexing
                        path_for_uav.append(linear_pos)
            
            uav_paths[f'UAV_{n+1}'] = path_for_uav
        
        # Calculate path costs
        total_cost = 0
        for n in range(self.N):
            path = uav_paths[f'UAV_{n+1}']
            if len(path) > 1:
                path_cost = 0
                for i in range(len(path) - 1):
                    # Convert linear indices to 2D coordinates
                    pos1 = np.unravel_index(path[i] - 1, (self.Nx, self.Ny))
                    pos2 = np.unravel_index(path[i+1] - 1, (self.Nx, self.Ny))
                    
                    # Calculate Euclidean distance
                    dist = np.sqrt((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2)
                    path_cost += dist
                
                total_cost += path_cost
        
        return {
            'uav_paths': uav_paths,
            'total_path_cost': total_cost,
            'solution_vector': solution_vector,
            'active_variables': len(active_indices)
        }
    
    def visualize_solution(self, solution, title="UAV Path Planning Solution"):
        """Visualize the UAV paths and coverage"""
        if solution is None:
            print("No solution to visualize")
            return
        
        fig, ax = plt.subplots(figsize=(10, 10))
        
        # Draw grid
        for i in range(self.Nx + 1):
            ax.axvline(x=i + 0.5, color='lightgray', linewidth=0.5)
        for j in range(self.Ny + 1):
            ax.axhline(y=j + 0.5, color='lightgray', linewidth=0.5)
        
        # Draw obstacles
        for obs in self.obstacles:
            ax.add_patch(plt.Rectangle((obs[1]-0.5, obs[0]-0.5), 1, 1, 
                                     facecolor='black', alpha=0.7))
        
        # Draw sink
        sink_row, sink_col = np.unravel_index(self.P_sink - 1, (self.Nx, self.Ny))
        ax.add_patch(plt.Rectangle((sink_col+0.5, sink_row+0.5), 1, 1, 
                                 facecolor='red', alpha=0.8))
        ax.text(sink_col+1, sink_row+1, 'SINK', ha='center', va='center', 
                fontweight='bold', color='white')
        
        # Draw UAV paths
        colors = ['blue', 'green', 'orange', 'purple', 'brown']
        for i, (uav_name, path) in enumerate(solution['uav_paths'].items()):
            if len(path) > 0:
                color = colors[i % len(colors)]
                
                # Convert path indices to coordinates
                coords = []
                for pos in path:
                    row, col = np.unravel_index(pos - 1, (self.Nx, self.Ny))
                    coords.append([col + 1, row + 1])  # +1 to center in grid cell
                
                if len(coords) > 1:
                    coords = np.array(coords)
                    ax.plot(coords[:, 0], coords[:, 1], 'o-', color=color, 
                           linewidth=2, markersize=8, label=uav_name)
                    
                    # Mark start and end
                    ax.plot(coords[0, 0], coords[0, 1], 's', color=color, 
                           markersize=12, markeredgecolor='black')
                    ax.plot(coords[-1, 0], coords[-1, 1], '^', color=color, 
                           markersize=12, markeredgecolor='black')
        
        ax.set_xlim(0.5, self.Ny + 0.5)
        ax.set_ylim(0.5, self.Nx + 0.5)
        ax.set_aspect('equal')
        ax.set_xlabel('Column')
        ax.set_ylabel('Row')
        ax.set_title(title)
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
        
        return fig, ax


def main():
    """
    Main function demonstrating the UAV path planning pipeline
    """
    print("Multi-UAV Path Planning with Connectivity Constraints")
    print("=" * 60)
    
    # Example 1: Movement Optimization (similar to movementOptimizationWithConnectivity.m)
    print("\n1. Movement Optimization Example")
    print("-" * 40)
    
    planner_movement = UAVPathPlanner(
        grid_size=(12, 12),
        sensing_radius=np.sqrt(2),
        communication_radius=2*np.sqrt(2),
        n_uavs=3,
        time_steps=10,
        sink_position=79,
        obstacles=[]  # No obstacles for this example
    )
    
    print(f"Grid size: {planner_movement.Nx}x{planner_movement.Ny}")
    print(f"Number of UAVs: {planner_movement.N}")
    print(f"Time steps: {planner_movement.T}")
    print(f"Sensing radius: {planner_movement.Rs}")
    print(f"Communication radius: {planner_movement.Rc}")
    print(f"Sink position: {planner_movement.P_sink}")
    
    # Solve movement optimization
    solution_movement = planner_movement.optimize_movement(coverage_ratio=1.0)
    
    if solution_movement:
        print(f"\nMovement Optimization Results:")
        print(f"Solve time: {solution_movement['solve_time']:.2f} seconds")
        print(f"Total path cost: {solution_movement['total_path_cost']:.2f}")
        print(f"Objective value: {solution_movement['objective_value']:.2f}")
        print(f"Active variables: {solution_movement['active_variables']}")
        
        print(f"\nUAV Paths:")
        for uav_name, path in solution_movement['uav_paths'].items():
            print(f"{uav_name}: {path[:10]}{'...' if len(path) > 10 else ''}")
        
        # Visualize solution
        planner_movement.visualize_solution(solution_movement, 
                                          "Movement Optimization Solution")
    
    # Example 2: Coverage Optimization (similar to coverageOptimizationWithConnectivity.m)
    print("\n2. Coverage Optimization Example")
    print("-" * 40)
    
    planner_coverage = UAVPathPlanner(
        grid_size=(6, 6),
        sensing_radius=1,
        communication_radius=2,
        n_uavs=3,
        time_steps=4,
        sink_position=1,
        obstacles=[]  # No obstacles for this example
    )
    
    print(f"Grid size: {planner_coverage.Nx}x{planner_coverage.Ny}")
    print(f"Number of UAVs: {planner_coverage.N}")
    print(f"Time steps: {planner_coverage.T}")
    print(f"Sensing radius: {planner_coverage.Rs}")
    print(f"Communication radius: {planner_coverage.Rc}")
    print(f"Sink position: {planner_coverage.P_sink}")
    
    # Solve coverage optimization
    solution_coverage = planner_coverage.optimize_coverage()
    
    if solution_coverage:
        print(f"\nCoverage Optimization Results:")
        print(f"Solve time: {solution_coverage['solve_time']:.2f} seconds")
        print(f"Total path cost: {solution_coverage['total_path_cost']:.2f}")
        print(f"Objective value: {solution_coverage['objective_value']:.2f}")
        print(f"Coverage percentage: {solution_coverage['coverage_percent']:.2f}%")
        print(f"Active variables: {solution_coverage['active_variables']}")
        
        print(f"\nUAV Paths:")
        for uav_name, path in solution_coverage['uav_paths'].items():
            print(f"{uav_name}: {path}")
        
        # Visualize solution
        planner_coverage.visualize_solution(solution_coverage, 
                                          "Coverage Optimization Solution")
    
    # Example 3: Custom scenario with obstacles
    print("\n3. Custom Scenario with Obstacles")
    print("-" * 40)
    
    planner_custom = UAVPathPlanner(
        grid_size=(8, 8),
        sensing_radius=1.5,
        communication_radius=3.0,
        n_uavs=2,
        time_steps=6,
        sink_position=29,  # Middle-ish position
        obstacles=[[3, 3], [3, 4], [4, 3], [4, 4]]  # 2x2 obstacle block
    )
    
    print(f"Grid size: {planner_custom.Nx}x{planner_custom.Ny}")
    print(f"Number of UAVs: {planner_custom.N}")
    print(f"Time steps: {planner_custom.T}")
    print(f"Obstacles: {planner_custom.obstacles}")
    
    # Solve coverage optimization with obstacles
    solution_custom = planner_custom.optimize_coverage()
    
    if solution_custom:
        print(f"\nCustom Scenario Results:")
        print(f"Solve time: {solution_custom['solve_time']:.2f} seconds")
        print(f"Coverage percentage: {solution_custom['coverage_percent']:.2f}%")
        print(f"Total path cost: {solution_custom['total_path_cost']:.2f}")
        
        # Visualize solution
        planner_custom.visualize_solution(solution_custom, 
                                        "Custom Scenario with Obstacles")


class UAVAnalyzer:
    """
    Additional analysis tools for UAV path planning results
    """
    
    @staticmethod
    def analyze_connectivity(planner, solution):
        """Analyze connectivity metrics of the solution"""
        if solution is None:
            return None
        
        connectivity_stats = {
            'communication_graph_density': [],
            'sensing_coverage_ratio': [],
            'path_efficiency': []
        }
        
        # Calculate communication graph density
        total_possible_comm = len(planner.G) * (len(planner.G) - 1) / 2
        actual_comm = sum(len(comm_list) for comm_list in planner.Irc.values()) / 2
        connectivity_stats['communication_graph_density'] = actual_comm / total_possible_comm
        
        # Calculate sensing coverage ratio
        total_sensing_pairs = sum(len(sense_list) for sense_list in planner.Irs.values())
        max_sensing_pairs = len(planner.G) * len(planner.G)
        connectivity_stats['sensing_coverage_ratio'] = total_sensing_pairs / max_sensing_pairs
        
        # Calculate path efficiency (straight line vs actual path)
        for uav_name, path in solution['uav_paths'].items():
            if len(path) > 1:
                # Calculate straight-line distance
                start_pos = np.unravel_index(path[0] - 1, (planner.Nx, planner.Ny))
                end_pos = np.unravel_index(path[-1] - 1, (planner.Nx, planner.Ny))
                straight_dist = np.sqrt((end_pos[0] - start_pos[0])**2 + 
                                      (end_pos[1] - start_pos[1])**2)
                
                # Calculate actual path distance
                actual_dist = 0
                for i in range(len(path) - 1):
                    pos1 = np.unravel_index(path[i] - 1, (planner.Nx, planner.Ny))
                    pos2 = np.unravel_index(path[i+1] - 1, (planner.Nx, planner.Ny))
                    actual_dist += np.sqrt((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2)
                
                efficiency = straight_dist / actual_dist if actual_dist > 0 else 0
                connectivity_stats['path_efficiency'].append(efficiency)
        
        return connectivity_stats
    
    @staticmethod
    def compare_solutions(solutions_dict):
        """Compare multiple solutions"""
        comparison = {}
        
        for name, solution in solutions_dict.items():
            if solution:
                comparison[name] = {
                    'solve_time': solution['solve_time'],
                    'path_cost': solution['total_path_cost'],
                    'coverage_percent': solution.get('coverage_percent', 0),
                    'active_variables': solution['active_variables']
                }
        
        return comparison
    
    @staticmethod
    def export_solution(planner, solution, filename):
        """Export solution to file"""
        if solution is None:
            print("No solution to export")
            return
        
        import json
        
        export_data = {
            'problem_parameters': {
                'grid_size': [planner.Nx, planner.Ny],
                'sensing_radius': planner.Rs,
                'communication_radius': planner.Rc,
                'n_uavs': planner.N,
                'time_steps': planner.T,
                'sink_position': planner.P_sink,
                'obstacles': planner.obstacles
            },
            'solution': {
                'uav_paths': solution['uav_paths'],
                'total_path_cost': solution['total_path_cost'],
                'solve_time': solution['solve_time'],
                'objective_value': solution['objective_value'],
                'coverage_percent': solution.get('coverage_percent', 0)
            }
        }
        
        with open(filename, 'w') as f:
            json.dump(export_data, f, indent=2)
        
        print(f"Solution exported to {filename}")


# Performance comparison function
def performance_comparison():
    """
    Compare performance across different problem sizes and parameters
    """
    print("\nPerformance Comparison Study")
    print("=" * 50)
    
    test_cases = [
        {'grid': (4, 4), 'uavs': 2, 'steps': 3, 'name': 'Small'},
        {'grid': (6, 6), 'uavs': 3, 'steps': 4, 'name': 'Medium'},
        {'grid': (8, 8), 'uavs': 3, 'steps': 5, 'name': 'Large'}
    ]
    
    results = {}
    
    for case in test_cases:
        print(f"\nTesting {case['name']} case: {case['grid']} grid, {case['uavs']} UAVs, {case['steps']} steps")
        
        planner = UAVPathPlanner(
            grid_size=case['grid'],
            sensing_radius=1,
            communication_radius=2,
            n_uavs=case['uavs'],
            time_steps=case['steps'],
            sink_position=1
        )
        
        # Test coverage optimization
        start_time = time.time()
        solution = planner.optimize_coverage()
        end_time = time.time()
        
        if solution:
            results[case['name']] = {
                'grid_size': case['grid'][0] * case['grid'][1],
                'solve_time': end_time - start_time,
                'coverage_percent': solution.get('coverage_percent', 0),
                'path_cost': solution['total_path_cost'],
                'variables': (1 + 2 * case['uavs'] * case['steps']) * case['grid'][0] * case['grid'][1]
            }
            print(f"✓ Solved in {results[case['name']]['solve_time']:.2f}s, "
                  f"Coverage: {results[case['name']]['coverage_percent']:.1f}%")
        else:
            print("✗ Failed to solve")
    
    # Display comparison table
    if results:
        print(f"\n{'Case':<10} {'Grid Size':<10} {'Variables':<12} {'Time (s)':<10} {'Coverage %':<12} {'Path Cost':<10}")
        print("-" * 70)
        for name, data in results.items():
            print(f"{name:<10} {data['grid_size']:<10} {data['variables']:<12} "
                  f"{data['solve_time']:<10.2f} {data['coverage_percent']:<12.1f} {data['path_cost']:<10.2f}")


if __name__ == "__main__":
    # Install required packages message
    print("Required packages: numpy, scipy, matplotlib, cvxpy")
    print("Install with: pip install numpy scipy matplotlib cvxpy")
    print("For better performance, also install: pip install gurobipy (requires license)")
    print("\n")
    
    # Run main examples
    try:
        main()
    except ImportError as e:
        print(f"Missing required package: {e}")
        print("Please install required packages and try again")
    except Exception as e:
        print(f"Error running main examples: {e}")
        print("This might be due to solver limitations or problem size")
    
    # Run performance comparison
    try:
        performance_comparison()
    except Exception as e:
        print(f"Error in performance comparison: {e}")
    
    print("\nPython implementation complete!")
    print("Note: For large problems, consider using commercial solvers like Gurobi or CPLEX")
    print("The MATLAB version uses CPLEX which typically provides better performance for MILP problems")