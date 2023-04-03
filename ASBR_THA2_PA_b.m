% Format workspace
clc; clear; format compact; clf; close all;

disp('--------b)--------')
disp('Numeric Space FK:')

% Instantiating the robot object numerically
[M, thetas, S_mat, B_mat, M_intermediates] = instantiate_robot();

% % Test cases
% thetas = zeros(1, 7)
% thetas = thetas
thetas = [4.7418, 1.7343, 4.2707, 4.1161, 1.0217, 0.7477, 3.1313];

% Performing space FK numerically
[FK_solution_space, T_bank_space, T_total_bank_space] = FK_space(M, S_mat, thetas, false, true, M_intermediates);
FK_solution_space