% Format workspace
clc; clear; format compact; clf; close all;

disp('--------c)--------')
disp('Symbolic Body FK:')

% Instantiating the robot object symbolically
[M_s, thetas_s, S_mat_s, B_mat_s] = instantiate_robot("franka", true);

% Performing space FK symbolically
[FK_solution_body_s, T_bank_body_s, T_total_bank_body_s] = FK_body(M_s, B_mat_s, thetas_s, true);
simplify(FK_solution_body_s)

fprintf('\n')
disp('Numeric Body FK:')

% Instantiating the robot object numerically
[M, thetas, S_mat, B_mat, M_intermediates] = instantiate_robot("franka", false);

% Performing space FK numerically
[FK_solution_body, T_bank_body, T_total_bank_body] = FK_body(M, B_mat, thetas, false, true, M_intermediates);
FK_solution_body