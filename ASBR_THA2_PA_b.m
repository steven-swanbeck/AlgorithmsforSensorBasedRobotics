% Format workspace
clc; clear; format compact; clf; close all;

disp('--------b)--------')
disp('Numeric Space FK:')

% Instantiating the robot object numerically
[M, thetas, S_mat, B_mat, M_intermediates] = instantiate_robot();

% Performing space FK numerically
[FK_solution_space, T_bank_space, T_total_bank_space] = FK_space(M, S_mat, thetas, false, true, M_intermediates);
FK_solution_space