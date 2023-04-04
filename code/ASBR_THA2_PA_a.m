% Format workspace
clc; clear; format compact; clf; close all;

disp('--------a)--------')
disp('Symbolic Space FK:')

% Instantiating the robot object symbolically
[M, thetas, S_mat, B_mat] = instantiate_robot("franka", true);

% Performing space FK symbolically
[FK_solution_space, T_bank_space, T_total_bank_space] = FK_space(M, S_mat, thetas, true);
simplify(FK_solution_space)