"""
run_kinematic_test_ta.py
Python version of the trailing arm suspension kinematic test.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from solve_suspension_kinematics_ta import solve_suspension_kinematics_ta

def main():
    # Define Suspension Geometry
    suspension = {}
    
    # --- Chassis Hardpoints ---
    suspension['p_TA_C'] = np.array([-1.81102, 0.41275, 0.33782]) + np.array([0, 0, 0.07])
    suspension['p_UL_C'] = np.array([-2.78003, 0.12065, 0.48768]) + np.array([0, 0, 0.07])
    suspension['p_LL_C'] = np.array([-2.7813, 0.05842, 0.3556]) + np.array([0, 0, 0.07])
    suspension['p_strut_mount_chassis'] = np.array([-2.19456, 0.35306, 0.83058]) + np.array([0, 0, 0.07])

    # --- Attachment points on rigid arm (design position) ---
    p_UL_A_d = np.array([-2.76098, 0.61214, 0.40386]) + np.array([0, 0, 0.07])
    p_LL_A_d = np.array([-2.7813, 0.61468, 0.25146]) + np.array([0, 0, 0.07])
    p_strut_d = np.array([-2.36728, 0.48641, 0.3683]) + np.array([0, 0, 0.07])
    p_wc_d = np.array([-2.71526, 0.716, 0.33504]) + np.array([0, 0, 0.07])

    # --- Calculate Link Length Constraints ---
    # The strut length is now a RESULT, not a constraint.
    suspension['L_UL'] = np.linalg.norm(p_UL_A_d - suspension['p_UL_C'])
    suspension['L_LL'] = np.linalg.norm(p_LL_A_d - suspension['p_LL_C'])

    # --- Define the Rigid Arm's Shape ---
    suspension['v_pivot_to_ULA'] = p_UL_A_d - suspension['p_TA_C']
    suspension['v_pivot_to_LLA'] = p_LL_A_d - suspension['p_TA_C']
    suspension['v_pivot_to_strut'] = p_strut_d - suspension['p_TA_C']
    suspension['v_pivot_to_WC'] = p_wc_d - suspension['p_TA_C']

    # Run the Kinematic Solver
    target_angle = 1.6767480556206  # Angle between z-axis and vector from pivot to LL mount
    
    # The initial guess is no rotation
    initial_guess = np.array([1, 0, 0, 0])  # Quaternion [w, x, y, z]

    results = solve_suspension_kinematics_ta(target_angle, suspension, initial_guess)

    print(f'\nFinal solver residual = {results["resnorm"]:.6e}')
    print(f'Solved at an arm angle of {np.rad2deg(results["final_angle"]):.2f} deg')
    print(f'Resulting strut length is {results["strut_length"]:.4f} m')

    # Visualize the Results
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title('Angle-Driven Rigid Trailing Arm Kinematics')

    # Define point collections for plotting
    chassis_pts_links = np.array([
        suspension['p_UL_C'], 
        suspension['p_LL_C'], 
        suspension['p_strut_mount_chassis']
    ])
    arm_pts = np.array([
        results['p_UL_A'], 
        results['p_LL_A'], 
        results['p_strut_mount_arm']
    ])

    # Plot key points
    # Arm pivot point
    ax.scatter(suspension['p_TA_C'][0], suspension['p_TA_C'][1], suspension['p_TA_C'][2],
               s=150, c='black', marker='p', label='Arm Pivot')

    # Chassis mounting points
    ax.scatter(chassis_pts_links[:, 0], chassis_pts_links[:, 1], chassis_pts_links[:, 2],
               s=100, c='blue', marker='o', label='Chassis Mounts')

    # Arm mounting points
    ax.scatter(arm_pts[:, 0], arm_pts[:, 1], arm_pts[:, 2],
               s=100, c='red', marker='o', label='Arm Mounts')

    # Wheel center
    ax.scatter(results['p_wheel_center'][0], results['p_wheel_center'][1], results['p_wheel_center'][2],
               s=100, c='black', marker='x', linewidth=2, label='Wheel Center')

    # Draw the rigid arm body as patches with shaded faces
    # Define all vertices including the pivot point
    all_arm_vertices = np.array([
        suspension['p_TA_C'],
        arm_pts[0],  # UL mount
        arm_pts[1],  # LL mount  
        arm_pts[2]   # strut mount
    ])

    # Define faces connecting pivot to arm mount points (tetrahedron-like structure)
    faces = [
        [0, 1, 2],  # Pivot to UL and LL mounts
        [0, 2, 3],  # Pivot to LL and strut mounts
        [0, 1, 3],  # Pivot to UL and strut mounts
        [1, 2, 3]   # Triangle between the three arm mounts
    ]

    # Create the 3D polygon collection for the rigid arm body
    poly3d = [[all_arm_vertices[j] for j in face] for face in faces]
    ax.add_collection3d(Poly3DCollection(poly3d, facecolors='red', alpha=0.15, 
                                         edgecolors='red', linewidths=1.5))

    # Draw suspension links
    # Upper link
    ax.plot([suspension['p_UL_C'][0], results['p_UL_A'][0]],
            [suspension['p_UL_C'][1], results['p_UL_A'][1]],
            [suspension['p_UL_C'][2], results['p_UL_A'][2]],
            color=[0.3, 0.3, 0.3], linewidth=2)

    # Lower link
    ax.plot([suspension['p_LL_C'][0], results['p_LL_A'][0]],
            [suspension['p_LL_C'][1], results['p_LL_A'][1]],
            [suspension['p_LL_C'][2], results['p_LL_A'][2]],
            color=[0.3, 0.3, 0.3], linewidth=2)

    # Strut (shock absorber)
    ax.plot([suspension['p_strut_mount_chassis'][0], results['p_strut_mount_arm'][0]],
            [suspension['p_strut_mount_chassis'][1], results['p_strut_mount_arm'][1]],
            [suspension['p_strut_mount_chassis'][2], results['p_strut_mount_arm'][2]],
            'c-', linewidth=4, label='Strut')

    # Set labels and formatting
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.legend()
    ax.grid(True)
    
    # Set viewing angle
    ax.view_init(elev=25, azim=135)
    
    # Set equal aspect ratio
    ax.set_box_aspect([1,1,1])
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
