function [theta_bank] = DLS_inverse_kinematics(T_sd, theta_0, M, S_mat, k, w_tol, v_tol)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    if nargin < 6
        w_tol = 1e-01;
        v_tol = 1e-01;
    end

    alpha = [1e-03 1e-03 1e-03 1e-03 1e-03 1e-03]';
    theta_bank = [theta_0'];
    i = 1;

    [T_sb] = FK_space(M, S_mat, theta_bank(:,i)); % forward kinemtics to get current position of end-effector
        
    J_space = SpaceJacobian(S_mat, theta_bank(:,i)); % space Jacobian at current configuration 
    J_body = Ad(inv(T_sb)) * J_space; % body Jacobian at current configuration
    
    T_bd = T_sb \ T_sd; % calculating transform between current eef configuration and desired eef configuration
    
    nu_b = T2S(T_bd); % twist from b (error measure)

    w_mag = norm(nu_b(1:3, :));
    v_mag = norm(nu_b(4:6, :));

    nu_b_old = nu_b;
    
    while w_mag > w_tol || v_mag > v_tol
        fprintf("%i\n", i)

        if J_isotropy(J_body, "linear") < 0.1 || J_ellipsoid_volume(J_body, "linear") < 0.1 || J_isotropy(J_body, "angular") < 0.1 || J_ellipsoid_volume(J_body, "angular") < 0.1
            theta_bank(:, i + 1) = theta_bank(:, i) + J_star(J_body, k) * (alpha .* nu_b); % updating thetas
        else
            theta_bank(:, i + 1) = theta_bank(:, i) + J_dagger(J_body) * (alpha .* nu_b); % updating thetas
        end

        [T_sb] = FK_space(M, S_mat, theta_bank(:,i)); % forward kinemtics to get current position of end-effector
        
        J_space = SpaceJacobian(S_mat, theta_bank(:,i)); % space Jacobian at current configuration 
        J_body = Ad(inv(T_sb)) * J_space; % body Jacobian at current configuration
        
        T_bd = T_sb \ T_sd; % calculating transform between current eef configuration and desired eef configuration
        
        nu_b = T2S(T_bd) % twist from b (error measure)

        w_mag = norm(nu_b(1:3, :));
        v_mag = norm(nu_b(4:6, :));
        fprintf("------------------\n");

%         Learning rate update
        delta_nu_b = abs(nu_b_old - nu_b);
        for j = 1:length(delta_nu_b)
            if abs(delta_nu_b / nu_b_old) > 0.1
                alpha(j) = alpha(j) / 10;
            elseif abs(delta_nu_b / nu_b_old) < 0.00001
                alpha(j) = alpha(j) * 10;
            end
        end

        nu_b_old = nu_b;
        i = i + 1;
    end

    fprintf("Exited IK solver with %.3f angular velocity and %.3f linear velocity.\n", w_mag, v_mag);
end