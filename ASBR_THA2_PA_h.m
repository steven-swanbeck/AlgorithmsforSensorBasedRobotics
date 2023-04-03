% Format workspace
clc; clear; format compact; clf; close all;

disp('--------h)--------')
[M, thetas, S_mat, B_mat, M_intermediates] = instantiate_robot();

% theta_0 = [2*pi/3 pi/6 -pi/7 -pi/4 pi/4 -pi/2 pi];
% theta_d = [0 pi/2 0 0 0 0 0]; % matrix is singular when set to zero, use this as demonstration case for later

% rng(67)
% rng(1)
rng(51)
theta_0 = 2 * pi * rand(1, 7);
theta_d = [0 -pi/2 0 0 0 0 0]; % matrix is singular when set to zero, use this as demonstration case for later


[FK_solution_space, T_bank_space, T_total_bank_space] = FK_space(M, S_mat, theta_d);
% [FK_solution_space, T_bank_space, T_total_bank_space] = FK_space(M, S_mat, theta_0, false, true, M_intermediates);
% [FK_solution_space, T_bank_space, T_total_bank_space] = FK_space(M, S_mat, theta_d, false, true, M_intermediates, false);

% Target pose
target_orientation = FK_solution_space(1:3, 1:3);
target_position = FK_solution_space(1:3, 4);
T_sd = [target_orientation target_position;
    zeros(1,3), 1];

% Creating plot
[T_sb] = FK_space(M, S_mat, theta_0, false, true, M_intermediates);
pause(1);

% IK using pseudo inverse 
[theta_f] = J_inverse_kinematics(T_sd, theta_0, M, S_mat, true, M_intermediates)

% Checking solution
[FK_actual] = FK_space(M, S_mat, theta_f);
FK_diff = T_sd - FK_actual