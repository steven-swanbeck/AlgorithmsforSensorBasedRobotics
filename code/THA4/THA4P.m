clc; clear; format compact; clf; close all;


[M, theta0, S_mat, B_mat, M_intermediates, joint_limits] = instantiate_robot("franka");
% p_goal = [0.45 0.15 0.6]';
% p_goal = [0.4 0 0.5]';
% p_goal = [0.3 -0.2 0.5]';
% p_goal = [0.7 -0.2 0.5]';
% p_goal = [0.3 0.4 0.9]';
p_goal = [0.3 0.2 0]';
p_tip = [0 0 0.1]';
tol = 3e-3; % m

[~] = FK_space(M, S_mat, theta0, false, true, M_intermediates);

[thetas, iteration] = IK_constrained(M, S_mat, p_tip, theta0, joint_limits, p_goal, tol);

[F_space2, ~] = FK_space(M, S_mat, thetas, false, true, M_intermediates, false);
% plot3(p_goal(1), p_goal(2), p_goal(3), '*c')

[x1, y1, z1] = sphere();
r = 0.003;
x2 = x1 * r;
y2 = y1 * r;
z2 = z1 * r;
surf(x2 + p_goal(1), y2 + p_goal(2), z2 + p_goal(3), 'EdgeColor','none', 'FaceColor','m', 'FaceAlpha', 0.3)

% plot3(coords(1, :), coords(2, :), coords(3, :), ':k')

animate_joint_goals("franka", [theta0' thetas'], true, "THA4", false, 30);