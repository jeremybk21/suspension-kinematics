"""
run_kinematic_test_dwb.py
This script defines a double-wishbone suspension geometry and calculates 
the position of the linkage for a given LCA angle.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from solve_suspension_kinematics_dwb import solve_suspension_kinematics_dwb

def main():
    # Define Suspension Geometry
    # All coordinates are in meters, from a vehicle-centric origin.
    # (X: forward, Y: left, Z: up)
    suspension = {}

    # --- Chassis Hardpoints (fixed) ---
    suspension['p_LCA_chassis_fwd'] = np.array([0.047, 0.112, 0.088]) + np.array([0, 0, 0.397])
    suspension['p_LCA_chassis_aft'] = np.array([-0.246, 0.108, 0.053]) + np.array([0, 0, 0.397])
    suspension['p_UCA_chassis_fwd'] = np.array([0.161, 0.136, 0.271]) + np.array([0, 0, 0.397])
    suspension['p_UCA_chassis_aft'] = np.array([-0.087, 0.131, 0.227]) + np.array([0, 0, 0.397])
    suspension['p_LCA_upright_design'] = np.array([0.016, 0.574, -0.082]) + np.array([0, 0, 0.397])
    suspension['p_UCA_upright_design'] = np.array([-0.008, 0.571, 0.088]) + np.array([0, 0, 0.397])
    suspension['p_strut_mount_lca_design'] = np.array([-0.002, 0.432, 0.186]) + np.array([0, 0, 0.397])
    suspension['p_strut_mount_chassis'] = np.array([-0.053, 0.166, 0.574]) + np.array([0, 0, 0.397])

    # --- Calculate Link Lengths (these are the constraints) ---
    suspension['L_LCA'] = np.linalg.norm(suspension['p_LCA_upright_design'] - suspension['p_LCA_chassis_fwd'])
    suspension['L_UCA_fwd'] = np.linalg.norm(suspension['p_UCA_upright_design'] - suspension['p_UCA_chassis_fwd'])
    suspension['L_UCA_aft'] = np.linalg.norm(suspension['p_UCA_upright_design'] - suspension['p_UCA_chassis_aft'])
    suspension['L_upright'] = np.linalg.norm(suspension['p_UCA_upright_design'] - suspension['p_LCA_upright_design'])

    # Run the Kinematic Solver
    # Define the input angle for the Lower Control Arm
    # Positive angle = wheel moves up in jounce/compression
    theta_LCA = np.deg2rad(0)

    # Call the solver
    results = solve_suspension_kinematics_dwb(theta_LCA, suspension)

    # Visualize the Results
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title(f'Suspension Position at LCA Angle = {np.rad2deg(theta_LCA):.1f} deg')

    # Plot chassis hardpoints (in blue)
    chassis_pts = np.array([
        suspension['p_LCA_chassis_fwd'], 
        suspension['p_LCA_chassis_aft'],
        suspension['p_UCA_chassis_fwd'], 
        suspension['p_UCA_chassis_aft']
    ])
    ax.scatter(chassis_pts[:, 0], chassis_pts[:, 1], chassis_pts[:, 2], 
               s=100, c='blue', marker='o', label='Chassis Mounts')

    # Plot calculated upright points (in red)
    upright_pts = np.array([results['p_LCA_upright'], results['p_UCA_upright']])
    ax.scatter(upright_pts[:, 0], upright_pts[:, 1], upright_pts[:, 2], 
               s=100, c='red', marker='o', label='Ball Joints')

    # Draw the links
    # LCA (Lower Control Arm) - Green
    ax.plot([suspension['p_LCA_chassis_fwd'][0], results['p_LCA_upright'][0]],
            [suspension['p_LCA_chassis_fwd'][1], results['p_LCA_upright'][1]],
            [suspension['p_LCA_chassis_fwd'][2], results['p_LCA_upright'][2]],
            'g-', linewidth=2, label='Lower Control Arm')
    ax.plot([suspension['p_LCA_chassis_aft'][0], results['p_LCA_upright'][0]],
            [suspension['p_LCA_chassis_aft'][1], results['p_LCA_upright'][1]],
            [suspension['p_LCA_chassis_aft'][2], results['p_LCA_upright'][2]],
            'g-', linewidth=2)

    # UCA (Upper Control Arm) - Magenta
    ax.plot([suspension['p_UCA_chassis_fwd'][0], results['p_UCA_upright'][0]],
            [suspension['p_UCA_chassis_fwd'][1], results['p_UCA_upright'][1]],
            [suspension['p_UCA_chassis_fwd'][2], results['p_UCA_upright'][2]],
            'm-', linewidth=2, label='Upper Control Arm')
    ax.plot([suspension['p_UCA_chassis_aft'][0], results['p_UCA_upright'][0]],
            [suspension['p_UCA_chassis_aft'][1], results['p_UCA_upright'][1]],
            [suspension['p_UCA_chassis_aft'][2], results['p_UCA_upright'][2]],
            'm-', linewidth=2)

    # Upright
    ax.plot([results['p_LCA_upright'][0], results['p_UCA_upright'][0]],
            [results['p_LCA_upright'][1], results['p_UCA_upright'][1]],
            [results['p_LCA_upright'][2], results['p_UCA_upright'][2]],
            'r-', linewidth=3, label='Upright')

    # Strut
    ax.plot([suspension['p_strut_mount_chassis'][0], results['p_strut_mount_lca'][0]],
            [suspension['p_strut_mount_chassis'][1], results['p_strut_mount_lca'][1]],
            [suspension['p_strut_mount_chassis'][2], results['p_strut_mount_lca'][2]],
            'c-', linewidth=4, label='Strut')

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.legend()
    ax.grid(True)
    
    # Set equal aspect ratio
    ax.set_box_aspect([1,1,1])
    
    plt.show()

if __name__ == "__main__":
    main()
