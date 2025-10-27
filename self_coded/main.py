import cplex
import numpy 

def get_coverage_vars(N:int):
    coverage_vars = []
    for i in range(1, N+1):
        coverage_vars.append(f'cgrid_{i}')
    return coverage_vars

def get_movement_vars(N:int, K:int, G:list):
    movement_vars = []
    for n in range(1, N+1):
        for k in range(1, K+1):
            for i in G:
                for j in G:
                    movement_vars.append(f'm_{i}_{j}_{n}_{k}')
    return movement_vars

def get_coverage_grid_wise_var(N:int, K:int, G:list):
    coverage_grid_wise_vars = []
    for i in G:
        for k in range(1, K+1):
            for n in range(1, N+1):
                coverage_grid_wise_vars.append(f'cgriduav_{i}_{n}_{k}')
    return coverage_grid_wise_vars

def get_presence_vars(N:int, K:int, G:list):
    presence_vars = []
    for i in G:
        for n in range(1, N+1):
            for k in range(1, K+1):
                presence_vars.append(f'z_{i}_{n}_{k}')
    return presence_vars

# The variables are ordered as: ci, m_ijkn, c_ink, z_ink

def get_obj_func_coeffs(N:int, K:int, G:list, all_vars:list):
    coeffs = []
    for i in range(len(all_vars)):
        if all_vars[i].startswith('cgrid_'):
            coeffs.append(1.0)
        else:
            coeffs.append(0.0)
    return coeffs

def get_eq2_unique_pos_coeffs(N:int, K:int, G:list, all_vars:list):
    coeffs = []
    for i in range(len(all_vars)):
        if all_vars[i].startswith('z_'):
            coeffs.append(1.0)
        else:
            coeffs.append(0.0)
    return coeffs

def get_eq3_collission_avoidance_coeffs(N:int, K:int, G:list, all_vars:list):
    coeffs = []
    for i in range(len(all_vars)):
        if all_vars[i].startswith('