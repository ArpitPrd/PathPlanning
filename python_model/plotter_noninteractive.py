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




def plot_coverage_paths(G, uav_paths, uav_covered_nodes, sink, Rs, Nx, Ny):
    """
    Plot the grid, UAV paths, and coverage areas.
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
    
    # Colors for different UAVs
    uav_colors = ['red', 'blue', 'green', 'orange', 'purple']
    
    # Plot 1: Grid with paths and coverage
    ax1.set_title('UAV Paths and Coverage Areas', fontsize=14, fontweight='bold')
    
    # Draw grid
    for i in range(1, Nx+1):
        for j in range(1, Ny+1):
            rect = patches.Rectangle((j-0.5, i-0.5), 1, 1, linewidth=1, 
                                   edgecolor='gray', facecolor='lightgray', alpha=0.3)
            ax1.add_patch(rect)
            ax1.text(j, i, f'({i},{j})', ha='center', va='center', fontsize=8)
    
    # Highlight sink
    sink_rect = patches.Rectangle((sink[1]-0.5, sink[0]-0.5), 1, 1, 
                                 linewidth=3, edgecolor='black', facecolor='yellow', alpha=0.8)
    ax1.add_patch(sink_rect)
    ax1.text(sink[1], sink[0], 'SINK', ha='center', va='center', fontweight='bold')
    
    # Plot coverage areas for each UAV
    for uav_id, covered_nodes in uav_covered_nodes.items():
        if covered_nodes:
            color = uav_colors[uav_id % len(uav_colors)]
            for node in covered_nodes:
                if tuple(node) != tuple(sink):  # Don't cover the sink
                    coverage_rect = patches.Rectangle((node[1]-0.5, node[0]-0.5), 1, 1,
                                                    linewidth=2, edgecolor=color, 
                                                    facecolor=color, alpha=0.3)
                    ax1.add_patch(coverage_rect)
    
    # Plot UAV paths
    for uav_id, path in uav_paths.items():
        if path:
            color = uav_colors[uav_id % len(uav_colors)]
            path_y = [pos[0] for pos in path]
            path_x = [pos[1] for pos in path]
            
            # Plot path line
            ax1.plot(path_x, path_y, color=color, linewidth=3, marker='o', 
                    markersize=8, label=f'UAV {uav_id+1}', alpha=0.8)
            
            # Add arrows to show direction
            for i in range(len(path_x)-1):
                ax1.annotate('', xy=(path_x[i+1], path_y[i+1]), xytext=(path_x[i], path_y[i]),
                           arrowprops=dict(arrowstyle='->', color=color, lw=2))
            
            # Mark start and end points
            ax1.plot(path_x[0], path_y[0], marker='s', color=color, markersize=12, 
                    markeredgecolor='black', markeredgewidth=2, label=f'UAV {uav_id+1} Start')
            ax1.plot(path_x[-1], path_y[-1], marker='^', color=color, markersize=12, 
                    markeredgecolor='black', markeredgewidth=2, label=f'UAV {uav_id+1} End')
    
    ax1.set_xlim(0.5, Ny+0.5)
    ax1.set_ylim(0.5, Nx+0.5)
    ax1.set_xlabel('Column (Y)', fontweight='bold')
    ax1.set_ylabel('Row (X)', fontweight='bold')
    ax1.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect('equal')
    
    # Plot 2: Coverage summary
    ax2.set_title('Coverage Summary', fontsize=14, fontweight='bold')
    
    # Create coverage matrix
    coverage_matrix = np.zeros((Nx, Ny))
    
    # Mark covered nodes
    all_covered = set()
    for uav_id, covered_nodes in uav_covered_nodes.items():
        all_covered.update([tuple(node) for node in covered_nodes])
    
    for node in all_covered:
        if tuple(node) != tuple(sink):
            coverage_matrix[node[0]-1, node[1]-1] = 1  # Convert to 0-based indexing
    
    # Mark sink
    coverage_matrix[sink[0]-1, sink[1]-1] = 2
    
    # Create custom colormap
    colors = ['white', 'lightgreen', 'yellow']  # uncovered, covered, sink
    cmap = ListedColormap(colors)
    
    im = ax2.imshow(coverage_matrix, cmap=cmap, vmin=0, vmax=2)
    
    # Add grid and labels
    for i in range(Nx):
        for j in range(Ny):
            if coverage_matrix[i, j] == 0:
                text = 'Not\nCovered'
                color = 'black'
            elif coverage_matrix[i, j] == 1:
                text = 'Covered'
                color = 'darkgreen'
            else:
                text = 'SINK'
                color = 'black'
            
            ax2.text(j, i, text, ha='center', va='center', 
                    fontweight='bold', color=color, fontsize=9)
    
    ax2.set_xticks(range(Ny))
    ax2.set_yticks(range(Nx))
    ax2.set_xticklabels(range(1, Ny+1))
    ax2.set_yticklabels(range(1, Nx+1))
    ax2.set_xlabel('Column (Y)', fontweight='bold')
    ax2.set_ylabel('Row (X)', fontweight='bold')
    
    # Add colorbar
    cbar = plt.colorbar(im, ax=ax2, shrink=0.8)
    cbar.set_ticks([0, 1, 2])
    cbar.set_ticklabels(['Not Covered', 'Covered', 'Sink'])
    
    plt.tight_layout()
    plt.show()
    
    # Print detailed coverage statistics
    total_nodes = Nx * Ny - 1  # Exclude sink
    covered_count = len(all_covered)
    coverage_percentage = (covered_count / total_nodes) * 100
    
    print(f"\n" + "="*50)
    print("COVERAGE STATISTICS")
    print("="*50)
    print(f"Total nodes (excluding sink): {total_nodes}")
    print(f"Nodes covered: {covered_count}")
    print(f"Coverage percentage: {coverage_percentage:.2f}%")
    print(f"Sensing radius: {Rs}")
    
    # Per-UAV statistics
    print(f"\nPer-UAV Coverage:")
    for uav_id in range(len(uav_paths)):
        if uav_paths[uav_id]:
            print(f"  UAV {uav_id+1}: {len(uav_covered_nodes[uav_id])} nodes")
        else:
            print(f"  UAV {uav_id+1}: 0 nodes (no path)")
