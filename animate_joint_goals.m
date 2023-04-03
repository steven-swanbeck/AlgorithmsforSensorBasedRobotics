function animate_joint_goals(robot_name, theta_series, show_ellipsoids, num_steps, delay, should_create_plot)
%Animates the motion of a known robot given a series of joint goals to
%reach
% Accepts a string for the name of the robot, which must be known to the
% "instantiate_robot()" function. Theta_series is a matrix of column
% vectors representing the desired joint values of the robot.
% show_ellipsoids will toggle the visible, possibly erroneous ellipsoids
% onto the plot, defaults to false. num_steps is the number of linear
% interpolation steps taken between joint values, defaults to 100. delay is
% time delay between updates of the plot, defaults to 0.05s.
% should_create_plot decides whether a new figure will be created, defaults
% to true.
    if nargin < 6
        should_create_plot = true;
    end
    if nargin < 5
        delay = 0.05;
    end
    if nargin < 4
        num_steps = 100;
    end
    if nargin < 3
        show_ellipsoids = false;
    end

    [M, ~, S_mat, ~, M_intermediates] = instantiate_robot(robot_name);

    if should_create_plot == true
        figure
        view(3)
        box on
        hold on
        grid on
        axis equal;
        xlabel('x'), ylabel('y'), zlabel('z');
        xlim([-1, 1]);
        ylim([-1, 1]);
        zlim([-0.5, 1.5]);
        view([30, 15]);
    end
    
    for i = 1:size(theta_series, 2) - 1
        current_thetas = theta_series(:, i);
        delta_theta = zeros(size(theta_series, 1), 1);
        for j = 1:size(theta_series, 1)
            delta_theta(j) = (theta_series(j, i + 1) - theta_series(j, i)) / num_steps;
        end
        for j = 1:num_steps
            cla;
            [FK_solution_space, ~, ~] = FK_space(M, S_mat, current_thetas, false, true, M_intermediates, false);
            J_space = SpaceJacobian(S_mat, current_thetas);
            ellipsoid_plot_angular(J_space, FK_solution_space(1:3,4), 0.25, show_ellipsoids);
            ellipsoid_plot_linear(J_space, FK_solution_space(1:3,4), 0.5, show_ellipsoids);
            pause(delay);
    
            current_thetas = current_thetas + delta_theta;
        end
    end
end