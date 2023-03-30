% Format workspace
clc; clear; format compact; clf; close all;

disp('--------h)--------')
[M, thetas, S_mat, B_mat, M_intermediates] = instantiate_robot();
[FK_solution_space, T_bank_space, T_total_bank_space] = FK_space(M, S_mat, thetas, false, true, M_intermediates);

target_orientation = FK_solution_space(1:3, 1:3);
target_position = FK_solution_space(1:3, 4);
T_sd = [target_orientation target_position;
    zeros(1,3), 1];

theta0 = zeros(1,7) + 0.5;

% IK using pseudo inverse 
[theta_bank] = J_inverse_kinematics(T_sd, theta0, M, S_mat)