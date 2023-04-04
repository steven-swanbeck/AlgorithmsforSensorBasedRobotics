function [theta_0, x, y, z] = J_inverse_kinematics(T_sd, theta_0, M, S_mat, should_plot, M_intermediates, produce_gif, gif_name, alpha_0, w_tol, v_tol, fading_memory_size, fading_memory_tolerance)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    if nargin < 10
        w_tol = 1e-02;
        v_tol = 1e-02;
    end
    if nargin < 9
        alpha_0 = 1e-02;
    end
    if nargin < 6
        should_plot = false;
    end
    if nargin < 12
        fading_memory_size = 10;
    end
    if nargin < 13
        fading_memory_tolerance = 1e-03;
    end
    if nargin < 7
        produce_gif = false;
    end
    if nargin < 8
        gif_name = "ik";
    end

    w_mag = w_tol + 1;
    v_mag = v_tol + 1;

    [T_0] = FK_space(M, S_mat, theta_0);
    x = [];
    y = [];
    z = [];
    previous_delta_thetas = zeros(size(S_mat,2), fading_memory_size);
    count = 0;

    if produce_gif == true
        filename = strcat(gif_name, ".gif");
    end

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
            
            % creating gif
            if produce_gif == true
                frame = getframe(gcf);
                im = frame2im(frame);
                [imind,cm] = rgb2ind(im,256);
                if count == 0
                    imwrite(imind,cm,filename,'gif', 'Loopcount',inf,...
                    'DelayTime',0.1);
                else
                    imwrite(imind,cm,filename,'gif','WriteMode','append',...
                    'DelayTime',0.1);
                end
            end

        else
            [T_sb] = FK_space(M, S_mat, theta_0);
        end
    
        J_s = SpaceJacobian(S_mat, theta_0);
        J_b = Ad(inv(T_sb)) * J_s;
        
        J_diff = J_s - Ad(T_sb) * J_b;
        
        T_bd = T_sb \ T_sd;
        nu_b = T2S(T_bd);
        delta_theta = alpha_0 * J_dagger(J_b) * nu_b;
        theta_0 = theta_0 + delta_theta';
        
        count = count + 1;

        % Fading memory implementation
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