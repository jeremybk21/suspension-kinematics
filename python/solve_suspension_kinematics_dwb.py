"""
solve_suspension_kinematics_dwb.py
This function calculates the 3D position of the suspension linkage
based on the input angle of the lower control arm (LCA).
"""

import numpy as np
import warnings

def solve_suspension_kinematics_dwb(theta_LCA, suspension):
    """
    Calculate the 3D position of the suspension linkage based on the input angle 
    of the lower control arm (LCA).
    
    Parameters:
    theta_LCA (float): Angle of the lower control arm in radians
    suspension (dict): Dictionary containing suspension geometry parameters
    
    Returns:
    dict: Results containing calculated positions
    """
    results = {}
    
    # Locate the Lower Ball Joint and Strut Mount
    # Both points rotate in circles around the LCA's chassis pivot axis.
    
    # Define the rotation axis
    p1 = suspension['p_LCA_chassis_fwd']
    p2 = suspension['p_LCA_chassis_aft']
    axis_vec = (p2 - p1) / np.linalg.norm(p2 - p1)
    angle = -theta_LCA  # Make positive angles correspond to compression
    
    # --- Calculate rotated position of the Lower Ball Joint ---
    p_to_rotate_bj = suspension['p_LCA_upright_design']
    v_bj = p_to_rotate_bj - p1
    v_rot_bj = (v_bj * np.cos(angle) + 
                np.cross(axis_vec, v_bj) * np.sin(angle) + 
                axis_vec * np.dot(axis_vec, v_bj) * (1 - np.cos(angle)))
    results['p_LCA_upright'] = p1 + v_rot_bj

    # --- Calculate rotated position of the Strut Mount ---
    p_to_rotate_strut = suspension['p_strut_mount_lca_design']
    v_strut = p_to_rotate_strut - p1
    v_rot_strut = (v_strut * np.cos(angle) + 
                   np.cross(axis_vec, v_strut) * np.sin(angle) + 
                   axis_vec * np.dot(axis_vec, v_strut) * (1 - np.cos(angle)))
    results['p_strut_mount_lca'] = p1 + v_rot_strut

    # Locate the Upper Ball Joint (p_UCA_upright)
    C1 = suspension['p_UCA_chassis_fwd']
    R1 = suspension['L_UCA_fwd']
    C2 = suspension['p_UCA_chassis_aft']
    R2 = suspension['L_UCA_aft']
    C3 = results['p_LCA_upright']
    R3 = suspension['L_upright']
    
    temp1 = C2 - C1
    e_x = temp1 / np.linalg.norm(temp1)
    temp2 = C3 - C1
    i = np.dot(e_x, temp2)
    temp3 = temp2 - i * e_x
    e_y = temp3 / np.linalg.norm(temp3)
    e_z = np.cross(e_x, e_y)
    d = np.linalg.norm(C2 - C1)
    j = np.dot(e_y, temp2)
    x = (R1**2 - R2**2 + d**2) / (2 * d)
    y = (R1**2 - R3**2 + i**2 + j**2) / (2 * j) - (i/j) * x
    z_arg = R1**2 - x**2 - y**2
    
    if z_arg < 0:
        warnings.warn('Three-sphere intersection has no real solution. Check geometry/lengths.')
        z = 0
    else:
        z = np.sqrt(z_arg)
    
    sol1 = C1 + x * e_x + y * e_y + z * e_z
    sol2 = C1 + x * e_x + y * e_y - z * e_z
    
    if sol1[2] > sol2[2]:
        results['p_UCA_upright'] = sol1
    else:
        results['p_UCA_upright'] = sol2
    
    return results
