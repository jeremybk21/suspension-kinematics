% run_kinematic_test_TA.m
clear; clc; close all;

%% Define Suspension Geometry
suspension = struct();
% --- Chassis Hardpoints ---
suspension.p_TA_C = [-1.81102, 0.41275, 0.33782] + [0, 0, 0.07];
suspension.p_UL_C = [-2.78003, 0.12065, 0.48768] + [0, 0, 0.07];
suspension.p_LL_C = [-2.7813, 0.05842, 0.3556] + [0, 0, 0.07];
suspension.p_strut_mount_chassis = [-2.19456, 0.35306, 0.83058] + [0, 0, 0.07]; 

% --- Attachment points on rigid arm (design position) ---
p_UL_A_d  = [-2.76098, 0.61214, 0.40386] + [0, 0, 0.07];
p_LL_A_d  = [-2.7813, 0.61468, 0.25146] + [0, 0, 0.07];
p_strut_d = [-2.36728, 0.48641, 0.3683] + [0, 0, 0.07];
p_wc_d    = [-2.71526, 0.716, 0.33504] + [0, 0, 0.07]; 

% --- Calculate Link Length Constraints ---
% The strut length is now a RESULT, not a constraint.
suspension.L_UL = norm(p_UL_A_d - suspension.p_UL_C);
suspension.L_LL = norm(p_LL_A_d - suspension.p_LL_C);

% --- Define the Rigid Arm's Shape ---
suspension.v_pivot_to_ULA   = p_UL_A_d - suspension.p_TA_C;
suspension.v_pivot_to_LLA   = p_LL_A_d - suspension.p_TA_C;
suspension.v_pivot_to_strut = p_strut_d - suspension.p_TA_C;
suspension.v_pivot_to_WC    = p_wc_d - suspension.p_TA_C; 

%% Run the Kinematic Solver
target_angle = 1.6767480556206; % Angle between z-axis and vector from pivot to LL mount. 

% The initial guess is no rotation.
initial_guess = [1, 0, 0, 0]; % Quaternion

results = solveSuspensionKinematics_TA(target_angle, suspension, initial_guess);

fprintf('\nFinal solver residual = %e\n', results.resnorm);
fprintf('Solved at an arm angle of %.2f deg\n', rad2deg(results.final_angle));
fprintf('Resulting strut length is %.4f m\n', results.strut_length);

%% Visualize the Results
% Create and configure the figure
figure('Name', 'Angle-Driven Rigid Trailing Arm Kinematics');
hold on; 
grid on; 
axis equal; 
view(135, 25);

% Define point collections for plotting
chassis_pts_links = [suspension.p_UL_C; suspension.p_LL_C; suspension.p_strut_mount_chassis];
arm_pts = [results.p_UL_A; results.p_LL_A; results.p_strut_mount_arm];

% Plot key points
% Arm pivot point
scatter3(suspension.p_TA_C(1), suspension.p_TA_C(2), suspension.p_TA_C(3), ...
         150, 'k', 'p', 'filled', 'DisplayName', 'Arm Pivot');

% Chassis mounting points
scatter3(chassis_pts_links(:,1), chassis_pts_links(:,2), chassis_pts_links(:,3), ...
         100, 'b', 'filled', 'DisplayName', 'Chassis Mounts');

% Arm mounting points
scatter3(arm_pts(:,1), arm_pts(:,2), arm_pts(:,3), ...
         100, 'r', 'filled', 'DisplayName', 'Arm Mounts');

% Wheel center
scatter3(results.p_wheel_center(1), results.p_wheel_center(2), results.p_wheel_center(3), ...
         100, 'k', 'x', 'LineWidth', 2, 'DisplayName', 'Wheel Center');

% Draw the rigid arm body as patches with shaded faces
% Define all vertices including the pivot point
all_arm_vertices = [suspension.p_TA_C; arm_pts];

% Define faces connecting pivot to arm mount points (tetrahedron-like structure)
faces = [1, 2, 3;    % Pivot to UL and LL mounts
         1, 3, 4;    % Pivot to LL and strut mounts  
         1, 2, 4;    % Pivot to UL and strut mounts
         2, 3, 4];   % Triangle between the three arm mounts

% Draw all faces of the rigid arm body
patch('Vertices', all_arm_vertices, 'Faces', faces, ...
      'FaceColor', 'r', 'FaceAlpha', 0.15, ...
      'EdgeColor', 'r', 'LineWidth', 1.5, ...
      'DisplayName', 'Rigid Arm Body');

% Draw suspension links
% Upper link
line([suspension.p_UL_C(1), results.p_UL_A(1)], ...
     [suspension.p_UL_C(2), results.p_UL_A(2)], ...
     [suspension.p_UL_C(3), results.p_UL_A(3)], ...
     'Color', [0.3 0.3 0.3], 'LineWidth', 2, 'HandleVisibility', 'off');

% Lower link
line([suspension.p_LL_C(1), results.p_LL_A(1)], ...
     [suspension.p_LL_C(2), results.p_LL_A(2)], ...
     [suspension.p_LL_C(3), results.p_LL_A(3)], ...
     'Color', [0.3 0.3 0.3], 'LineWidth', 2, 'HandleVisibility', 'off');

% Strut (shock absorber)
line([suspension.p_strut_mount_chassis(1), results.p_strut_mount_arm(1)], ...
     [suspension.p_strut_mount_chassis(2), results.p_strut_mount_arm(2)], ...
     [suspension.p_strut_mount_chassis(3), results.p_strut_mount_arm(3)], ...
     'Color', 'c', 'LineWidth', 4, 'DisplayName', 'Strut');

% Configure plot appearance
xlabel('X (m)'); 
ylabel('Y (m)'); 
zlabel('Z (m)');
title(sprintf('Suspension Position at Driving Angle = %.1f deg', rad2deg(target_angle)));
legend('Location', 'northeast');
hold off;