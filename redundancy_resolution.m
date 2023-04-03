function [theta_0, x, y, z] = redundancy_resolution(T_sd, theta_0, M, S_mat, k0, should_plot, M_intermediates, alpha_0, w_tol, v_tol, fading_memory_size, fading_memory_tolerance)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    if nargin < 9
        w_tol = 1e-02;
        v_tol = 1e-02;
    end
    if nargin < 8
        alpha_0 = 1e-02;
    end
    if nargin < 7
        should_plot = false;
    end
    if nargin < 11
        fading_memory_size = 10;
    end
    if nargin < 12
        fading_memory_tolerance = 1e-03;
    end

    w_mag = w_tol + 1;
    v_mag = v_tol + 1;

    [T_0] = FK_space(M, S_mat, theta_0);
    x = [];
    y = [];
    z = [];
    previous_delta_thetas = zeros(size(S_mat,2), fading_memory_size);
    previous_thetas = zeros(size(S_mat,2), 2);
    previous_ws = zeros(1, 2);
    count = 0;

    while w_mag > w_tol || v_mag > v_tol
        if should_plot == true
            cla;
            [T_sb] = FK_space(M, S_mat, theta_0, false, true, M_intermediates, false);
            x = [x, T_sb(1, 4)];
            y = [y, T_sb(2, 4)];
            z = [z, T_sb(3, 4)];
            plot3(x, y, z, ':k')
            plotFrame_T(T_0, 'start', 0.1)
            plotFrame_T(T_sd, 'goal', 0.1)
            pause(0.03);
        else
            [T_sb] = FK_space(M, S_mat, theta_0);
        end
    
        J_s = SpaceJacobian(S_mat, theta_0);
        J_b = Ad(inv(T_sb)) * J_s;

%         if should_plot == true
%             ellipsoid_plot_angular(J_s, T_sb(1:3,4), 0.25, false);
%             ellipsoid_plot_linear(J_s, T_sb(1:3,4), 0.5, false);
%             pause(0.03);
%         end
        
        J_diff = J_s - Ad(T_sb) * J_b;
        
        T_bd = T_sb \ T_sd;
        nu_b = T2S(T_bd);

        % Using redundancy resolution if able to calculate the derivative
        if count > 2
            d_theta = previous_thetas(:,2) - previous_thetas(:,1);
            w = sqrt(det(J_b * J_b'));
            d_w = previous_ws(2) - previous_ws(1);
            dw_dtheta = d_w ./ d_theta';
            q0_dot = k0 * dw_dtheta';
        else
            w = 0;
            q0_dot = zeros(7,1);
        end

        standard_update = alpha_0 * J_dagger(J_b) * nu_b;
        additional_update = (eye(size(J_b' * J_b, 2)) - J_dagger(J_b) * J_b) * q0_dot;
        delta_theta = standard_update + additional_update;
        theta_0 = theta_0 + delta_theta';
        
        count = count + 1;

        % Storing w and thetas for derivative calculation
        w_old = w;
        if count <= 2
            previous_thetas(:, count) = theta_0;
            previous_ws(count) = w;
        else
            previous_thetas(:, 1) = previous_thetas(:, 2); 
            previous_thetas(:, 2) = theta_0;
            previous_ws(1) = previous_ws(2);
            previous_ws(2) = w;
        end

        % Fading memomry implementation
        if count <= fading_memory_size
            previous_delta_thetas(:, count) = delta_theta;
        else
            for i = 2:fading_memory_size
                previous_delta_thetas(:, i - 1) = previous_delta_thetas(:, i); 
            end
            previous_delta_thetas(:, fading_memory_size) = delta_theta;

            % now checking to see if average of previous thetas 
            means = zeros(size(S_mat,2), 1);
            for i = 1:size(S_mat,2)
                means(i) = mean(previous_delta_thetas(i, :));
            end
            if max(means) < fading_memory_tolerance
                w_mag = w_tol;
                v_mag = v_tol;
            end
        end
    end
end