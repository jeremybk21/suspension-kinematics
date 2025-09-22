"""
solve_suspension_kinematics_ta.py
This solver finds the 3D orientation of the rigid arm by satisfying
the UL length, LL length, and a target driving angle.
"""

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R

def solve_suspension_kinematics_ta(target_angle, suspension, initial_guess):
    """
    Solve the trailing arm suspension kinematics using nonlinear least squares.
    
    Parameters:
    target_angle (float): Target angle in radians
    suspension (dict): Suspension geometry parameters
    initial_guess (array): Initial quaternion guess [w, x, y, z]
    
    Returns:
    dict: Results containing solved positions and metrics
    """
    
    def error_function(q_guess):
        return suspension_error_vector(q_guess, target_angle, suspension)
    
    # Use scipy's least_squares optimizer
    result = least_squares(error_function, initial_guess, 
                          ftol=1e-15, xtol=1e-15, gtol=1e-15)
    
    q_solution = result.x
    resnorm = np.sum(result.fun**2)
    
    # --- Post-processing: Calculate final positions from the solution quaternion ---
    q_solution = q_solution / np.linalg.norm(q_solution)
    
    # Convert quaternion to rotation matrix using scipy
    rotation = R.from_quat([q_solution[1], q_solution[2], q_solution[3], q_solution[0]])  # scipy uses [x,y,z,w]
    R_final = rotation.as_matrix()

    # Calculate all final arm points
    results = {}
    results['p_UL_A'] = suspension['p_TA_C'] + R_final @ suspension['v_pivot_to_ULA']
    results['p_LL_A'] = suspension['p_TA_C'] + R_final @ suspension['v_pivot_to_LLA']
    results['p_strut_mount_arm'] = suspension['p_TA_C'] + R_final @ suspension['v_pivot_to_strut']
    results['p_wheel_center'] = suspension['p_TA_C'] + R_final @ suspension['v_pivot_to_WC']
    
    # --- Calculate and store final results ---
    # Final angle
    final_vec = results['p_LL_A'] - suspension['p_TA_C']
    results['final_angle'] = np.arccos(np.dot(final_vec, np.array([0, 0, 1])) / np.linalg.norm(final_vec))
    
    # Resulting strut length
    results['strut_length'] = np.linalg.norm(results['p_strut_mount_arm'] - suspension['p_strut_mount_chassis'])
    
    results['resnorm'] = resnorm
    results['q_solution'] = q_solution
    
    return results

def suspension_error_vector(q_guess, target_angle, suspension):
    """
    Calculate the 3 constraint errors (UL, LL, Angle).
    
    Parameters:
    q_guess (array): Quaternion guess [w, x, y, z]
    target_angle (float): Target angle in radians
    suspension (dict): Suspension geometry parameters
    
    Returns:
    array: Error vector [err_UL, err_LL, err_angle]
    """
    
    # 1. Unpack and prepare orientation
    q_guess = q_guess / np.linalg.norm(q_guess)
    
    # Convert quaternion to rotation matrix using scipy
    rotation = R.from_quat([q_guess[1], q_guess[2], q_guess[3], q_guess[0]])  # scipy uses [x,y,z,w]
    R_guess = rotation.as_matrix()

    # 2. Calculate global positions of the relevant arm points
    p_UL_A = suspension['p_TA_C'] + R_guess @ suspension['v_pivot_to_ULA']
    p_LL_A = suspension['p_TA_C'] + R_guess @ suspension['v_pivot_to_LLA']
    
    # 3. Calculate the current values for the constraints
    current_L_UL = np.linalg.norm(p_UL_A - suspension['p_UL_C'])
    current_L_LL = np.linalg.norm(p_LL_A - suspension['p_LL_C'])
    
    vec_drive = p_LL_A - suspension['p_TA_C']
    current_angle = np.arccos(np.dot(vec_drive, np.array([0, 0, 1])) / np.linalg.norm(vec_drive))
    
    # 4. Define the error for each constraint
    err_UL = current_L_UL - suspension['L_UL']
    err_LL = current_L_LL - suspension['L_LL']
    err_angle = current_angle - target_angle
    
    # 5. Return the errors as an array
    return np.array([err_UL, err_LL, err_angle])
