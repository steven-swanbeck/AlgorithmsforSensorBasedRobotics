function [thetas] = J_transpose_kinematics(T_sd, theta_0, M, S_mat, w_tol, v_tol)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    if nargin < 5
        w_tol = 1e-01;
        v_tol = 1e-01;
    end

    K = eye(6); % K is a symmetric, PD matrix

    R_desired = T_sd(1:3, 1:3);
    p_desired = T_sd(1:3, 4);
%     [q_desired] = RotationMatrix2Quaternion(R_desired);
    [phi, theta, psi] = RotationMatrix2RPYAngles(R_desired);
    desired_state_vector = [p_desired; phi; theta; psi];
    
    thetas = theta_0;

    delta_theta = ones(length(theta_0), 1);

    w_mag = 1;
    v_mag = 1;

    while max(abs(delta_theta)) > 0.001
%     while w_mag > w_tol || v_mag > v_tol
        [T_sb] = FK_space(M, S_mat, thetas);
        [phi, theta, psi] = RotationMatrix2RPYAngles(T_sb(1:3, 1:3));
        current_state_vector = [T_sb(1:3, 4); phi; theta; psi];
    
        state_error = desired_state_vector - current_state_vector

        J_space = SpaceJacobian(S_mat, thetas);
        J_body = Ad(inv(T_sb)) * J_space;

%         I now think this is incorrect
%         x_dot = J_body * J_body' * K * state_error;
%         delta_theta = J_dagger(J_body) * x_dot;
%         thetas = thetas + delta_theta;
%         w_mag = norm(x_dot(1:3, :));
%         v_mag = norm(x_dot(4:6, :));

%         I think this is correct
        theta_dot = J_body' * K * state_error;
        thetas = thetas + theta_dot;
    end 
end