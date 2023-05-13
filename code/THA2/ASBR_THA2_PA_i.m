% Format workspace
clc; clear; format compact; clf; close all;

disp('--------i)--------')

% Format workspace
clc; clear; format compact; clf; close all;
[M, thetas, S_mat, B_mat, M_intermediates] = instantiate_robot();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Test case 1
% theta_0 = [2*pi/3 pi/6 0 -pi/4 pi/4 -pi/2 0];
% theta_d = [0 pi/2 0 0 0 0 0];
% [FK_solution_space] = FK_space(M, S_mat, theta_d);
% target_orientation = FK_solution_space(1:3, 1:3);
% target_position = FK_solution_space(1:3, 4);
% test_name = "Test1";

% % Test case 2
% theta_0 = [0 -pi/4 0 -3*pi/4 0 pi/2 pi/4];
% target_orientation = [0 1 0; 1 0 0; 0 0 -1];
% target_position = [0.3, 0.3, 0]';
% test_name = "Test2";

% % Test case 3
% rng(10)
% theta_0 = 2 * pi * rand(1, 7);
% theta_d = [0 0 0 0 0 0 0];
% [FK_solution_space] = FK_space(M, S_mat, theta_d);
% target_orientation = FK_solution_space(1:3, 1:3);
% target_position = FK_solution_space(1:3, 4);
% test_name = "Test3";

% % Test case 4
% rng(51)
% theta_0 = 2 * pi * rand(1, 7);
% theta_d = [0 0 0 0 0 0 0];
% [FK_solution_space] = FK_space(M, S_mat, theta_d);
% target_orientation = FK_solution_space(1:3, 1:3);
% target_position = FK_solution_space(1:3, 4);
% test_name = "Test4";

% Test case 5
rng(67)
theta_0 = 2 * pi * rand(1, 7);
theta_d = [0 0 0 0 0 0 0];
[FK_solution_space] = FK_space(M, S_mat, theta_d);
target_orientation = FK_solution_space(1:3, 1:3);
target_position = FK_solution_space(1:3, 4);
test_name = "Test5";
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

root = "jt_ik_";
gif_name = strcat(root, test_name);

% Target pose
T_sd = [target_orientation target_position;
    zeros(1,3), 1];

% Creating plot
[T_sb] = FK_space(M, S_mat, theta_0, false, true, M_intermediates);
pause(1);

% IK using pseudo inverse 
alpha = 1e-02 * eye(6);
[theta_f, x, y, z] = J_transpose_kinematics(T_sd, theta_0, M, S_mat, alpha, true, M_intermediates, true, gif_name);

% Checking solution
[FK_actual] = FK_space(M, S_mat, theta_f);
FK_diff = T_sd - FK_actual

% Final Comparison Figure
cla;
plot3(x, y, z, '-', 'Color', '#00FFFF', 'LineWidth', 3)
[FK_solution_space] = FK_space2(M, S_mat, theta_0, true, M_intermediates, 0.3, false, false);
[FK_solution_space] = FK_space2(M, S_mat, theta_f, true, M_intermediates, 0.3, false, false);
title(strcat('Jacobian Transpose IK', {' '}, test_name))
saveas(gcf, strcat(gif_name, ".png"))