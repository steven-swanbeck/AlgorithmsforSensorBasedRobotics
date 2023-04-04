% Format workspace
clc; clear; format compact; clf; close all;
set(0,'defaultTextInterpreter','latex');
set(0, 'defaultAxesTickLabelInterpreter','latex'); 
set(0, 'defaultLegendInterpreter','latex');
set(0,'defaultAxesFontSize',18);
set(0, 'DefaultLineLineWidth', 2);
set(groot, 'defaultFigureUnits', 'pixels', 'defaultFigurePosition', [440   278   560   420]);

disp('--------Comparison of IK Methods--------')
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

% Target pose
T_sd = [target_orientation target_position;
    zeros(1,3), 1];

% Creating plot
[T_sb] = FK_space(M, S_mat, theta_0, false, true, M_intermediates);
pause(1);

% IK using pseudo inverse 
[theta_f, x1, y1, z1] = J_inverse_kinematics(T_sd, theta_0, M, S_mat, true, M_intermediates);

% IK using transpose
K = 1e-02 * eye(6);
[theta_f, x2, y2, z2] = J_transpose_kinematics(T_sd, theta_0, M, S_mat, K, true, M_intermediates);

% IK using RR 
k0 = 0.1;
[theta_f, x3, y3, z3] = redundancy_resolution(T_sd, theta_0, M, S_mat, k0, true, M_intermediates);

% IK using DLS
k = 0.5;
[theta_f, x4, y4, z4] = DLS_inverse_kinematics(T_sd, theta_0, M, S_mat, k, true, M_intermediates);

% Comparison
cla;
plot3(x1, y1, z1, '-', 'Color', "#000000", 'LineWidth', 3)
plot3(x2, y2, z2, '-', 'Color', '#00FFFF', 'LineWidth', 3)
plot3(x3, y3, z3, '-', 'Color', '#FFFF00', 'LineWidth', 3)
plot3(x4, y4, z4, '-', 'Color', '#FF00FF', 'LineWidth', 3)
[FK_solution_space] = FK_space2(M, S_mat, theta_0, true, M_intermediates, 0.3, false, false);
[FK_solution_space] = FK_space2(M, S_mat, theta_f, true, M_intermediates, 0.3, false, false);
title(strcat('IK Method Comparison', {' '}, test_name))
legend('Pseudo-Inverse', 'Transpose', 'Redundancy Resolution', 'Damped Least-Squares')
legend('location', 'best')
