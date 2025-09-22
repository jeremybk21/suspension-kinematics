% solveSuspensionKinematics_TA.m
function results = solveSuspensionKinematics_TA(target_angle, suspension, initial_guess)
    % This solver finds the 3D orientation of the rigid arm by satisfying
    % the UL length, LL length, and a target driving angle.

    error_func = @(q_guess) suspension_error_vector(q_guess, target_angle, suspension);

    options = optimoptions('lsqnonlin', 'Display', 'off', 'FunctionTolerance', 1e-15, 'StepTolerance', 1e-15);
    [q_solution, resnorm] = lsqnonlin(error_func, initial_guess, [], [], options);
    
    % --- Post-processing: Calculate final positions from the solution quaternion ---
    q_solution = q_solution / norm(q_solution);
    R_final = quat2rotm(q_solution);

    % Calculate all final arm points
    results.p_UL_A = suspension.p_TA_C + (R_final * suspension.v_pivot_to_ULA')';
    results.p_LL_A = suspension.p_TA_C + (R_final * suspension.v_pivot_to_LLA')';
    results.p_strut_mount_arm = suspension.p_TA_C + (R_final * suspension.v_pivot_to_strut')';
    results.p_wheel_center = suspension.p_TA_C + (R_final * suspension.v_pivot_to_WC')';
    
    % --- Calculate and store final results ---
    % Final angle
    final_vec = results.p_LL_A - suspension.p_TA_C;
    results.final_angle = acos(dot(final_vec, [0,0,1]) / norm(final_vec));
    % Resulting strut length
    results.strut_length = norm(results.p_strut_mount_arm - suspension.p_strut_mount_chassis);
    
    results.resnorm = resnorm;
    results.q_solution = q_solution;
end

function errors = suspension_error_vector(q_guess, target_angle, suspension)
    % This function calculates the 3 constraint errors (UL, LL, Angle).
    
    % 1. Unpack and prepare orientation
    q_guess = q_guess / norm(q_guess);
    R_guess = quat2rotm(q_guess);

    % 2. Calculate global positions of the relevant arm points
    p_UL_A = suspension.p_TA_C + (R_guess * suspension.v_pivot_to_ULA')';
    p_LL_A = suspension.p_TA_C + (R_guess * suspension.v_pivot_to_LLA')';
    
    % 3. Calculate the current values for the constraints
    current_L_UL = norm(p_UL_A - suspension.p_UL_C);
    current_L_LL = norm(p_LL_A - suspension.p_LL_C);
    
    vec_drive = p_LL_A - suspension.p_TA_C;
    current_angle = acos(dot(vec_drive, [0,0,1]) / norm(vec_drive));
    
    % 4. Define the error for each constraint
    err_UL = current_L_UL - suspension.L_UL;
    err_LL = current_L_LL - suspension.L_LL;
    err_angle = (current_angle - target_angle); % Weight can be added if needed
    
    % 5. Return the errors as a column vector
    errors = [err_UL; err_LL; err_angle];
end