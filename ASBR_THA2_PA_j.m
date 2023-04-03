% Format workspace
clc; clear; format compact; clf; close all;

disp('--------i)--------')
[M, thetas, S_mat, B_mat, M_intermediates] = instantiate_robot();

% % Test case 1
% theta_0 = [2*pi/3 pi/6 -pi/7 -pi/4 pi/4 -pi/2 pi];
% theta_d = [0 0 0 0 0 0 0]; % matrix is singular when set to zero, use this as demonstration case for later

% theta_0 = [2*pi/3 pi/6 -pi/7 -pi/4 pi/4 -pi/10 0];
% theta_d = [0 0 0 0 0 0 0]; % matrix is singular when set to zero, use this as demonstration case for later

rng(67)
theta_0 = 2 * pi * rand(1, 7);
theta_d = [0 0 0 0 0 0 0];


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

% IK using RR 
k0 = 0.01;
[theta_f] = redundancy_resolution(T_sd, theta_0, M, S_mat, k0, true, M_intermediates)