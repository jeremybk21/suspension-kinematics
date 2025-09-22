% run_kinematic_test.m
% This script defines a double-wishbone suspension geometry and calculates 
% the position of the linkage for a given LCA angle.

clear; clc; close all;

%% Define Suspension Geometry
% All coordinates are in meters, from a vehicle-centric origin.
% (X: forward, Y: left, Z: up)
suspension = struct();

% --- Chassis Hardpoints (fixed) ---
suspension.p_LCA_chassis_fwd = [0.047, 0.112, 0.088] + [0,0,0.397];
suspension.p_LCA_chassis_aft = [-0.246, 0.108, 0.053] + [0,0,0.397];
suspension.p_UCA_chassis_fwd = [0.161, 0.136, 0.271] + [0,0,0.397];
suspension.p_UCA_chassis_aft = [-0.087, 0.131, 0.227] + [0,0,0.397];
suspension.p_LCA_upright_design = [0.016, 0.574, -0.082] + [0,0,0.397];
suspension.p_UCA_upright_design = [-0.008, 0.571, 0.088] + [0,0,0.397];
suspension.p_strut_mount_lca_design = [-0.002, 0.432, 0.186] + [0,0,0.397];
suspension.p_strut_mount_chassis = [ -0.053, 0.166, 0.574 ] + [0,0,0.397]; 

% --- Calculate Link Lengths (these are the constraints) ---
suspension.L_LCA = norm(suspension.p_LCA_upright_design - suspension.p_LCA_chassis_fwd); % Assuming an A-arm
suspension.L_UCA_fwd = norm(suspension.p_UCA_upright_design - suspension.p_UCA_chassis_fwd);
suspension.L_UCA_aft = norm(suspension.p_UCA_upright_design - suspension.p_UCA_chassis_aft);
suspension.L_upright = norm(suspension.p_UCA_upright_design - suspension.p_LCA_upright_design);

%% Run the Kinematic Solver
% Define the input angle for the Lower Control Arm
% Positive angle = wheel moves up in jounce/compression
theta_LCA = deg2rad(0);

% Call the solver
results = solveSuspensionKinematics_DWB(theta_LCA, suspension);

%% Visualize the Results
figure('Name', 'Suspension Kinematics');
hold on; grid on; axis equal; view(3);

% Plot chassis hardpoints (in blue)
chassis_pts = [suspension.p_LCA_chassis_fwd; suspension.p_LCA_chassis_aft; ...
               suspension.p_UCA_chassis_fwd; suspension.p_UCA_chassis_aft];
scatter3(chassis_pts(:,1), chassis_pts(:,2), chassis_pts(:,3), 100, 'b', 'filled', 'DisplayName', 'Chassis Mounts');

% Plot calculated upright points (in red)
upright_pts = [results.p_LCA_upright; results.p_UCA_upright];
scatter3(upright_pts(:,1), upright_pts(:,2), upright_pts(:,3), 100, 'r', 'filled', 'DisplayName', 'Ball Joints');

% Draw the links
% LCA (Lower Control Arm) - Green
plot3([suspension.p_LCA_chassis_fwd(1), results.p_LCA_upright(1)], ...
      [suspension.p_LCA_chassis_fwd(2), results.p_LCA_upright(2)], ...
      [suspension.p_LCA_chassis_fwd(3), results.p_LCA_upright(3)], ...
      'g-', 'LineWidth', 2, 'DisplayName', 'Lower Control Arm');
plot3([suspension.p_LCA_chassis_aft(1), results.p_LCA_upright(1)], ...
      [suspension.p_LCA_chassis_aft(2), results.p_LCA_upright(2)], ...
      [suspension.p_LCA_chassis_aft(3), results.p_LCA_upright(3)], ...
      'g-', 'LineWidth', 2, 'HandleVisibility', 'off');

% UCA (Upper Control Arm) - Magenta
plot3([suspension.p_UCA_chassis_fwd(1), results.p_UCA_upright(1)], ...
      [suspension.p_UCA_chassis_fwd(2), results.p_UCA_upright(2)], ...
      [suspension.p_UCA_chassis_fwd(3), results.p_UCA_upright(3)], ...
      'm-', 'LineWidth', 2, 'DisplayName', 'Upper Control Arm');
plot3([suspension.p_UCA_chassis_aft(1), results.p_UCA_upright(1)], ...
      [suspension.p_UCA_chassis_aft(2), results.p_UCA_upright(2)], ...
      [suspension.p_UCA_chassis_aft(3), results.p_UCA_upright(3)], ...
      'm-', 'LineWidth', 2, 'HandleVisibility', 'off');

% Upright
plot3([results.p_LCA_upright(1), results.p_UCA_upright(1)], ...
      [results.p_LCA_upright(2), results.p_UCA_upright(2)], ...
      [results.p_LCA_upright(3), results.p_UCA_upright(3)], 'r-', 'LineWidth', 3, 'DisplayName', 'Upright');

% Strut 
plot3([suspension.p_strut_mount_chassis(1), results.p_strut_mount_lca(1)], ...
      [suspension.p_strut_mount_chassis(2), results.p_strut_mount_lca(2)], ...
      [suspension.p_strut_mount_chassis(3), results.p_strut_mount_lca(3)], 'c-', 'LineWidth', 4, 'DisplayName', 'Strut');

xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title(['Suspension Position at LCA Angle = ', num2str(rad2deg(theta_LCA)), ' deg']);
legend;