% Format workspace
clc; clear; format compact; clf; close all;

disp('--------h)--------')
[M, thetas, S_mat, B_mat, M_intermediates] = instantiate_robot();

rng(10) % great comparison for DLS!!!, decent for RR as well
% rng(51) % decent for some stuff
% rng(101)
% rng(32)
% rng(5260)
% rng(67)
theta_0 = 2 * pi * rand(1, 7);
theta_d = [0 0 0 0 0 0 0];

[FK_solution_space, T_bank_space, T_total_bank_space] = FK_space(M, S_mat, theta_d);

% Target pose
target_orientation = FK_solution_space(1:3, 1:3);
target_position = FK_solution_space(1:3, 4);
T_sd = [target_orientation target_position;
    zeros(1,3), 1];

% Creating plot
[T_sb] = FK_space(M, S_mat, theta_0, false, true, M_intermediates);
pause(1);

% IK using pseudo inverse 
[theta_f, x1, y1, z1] = J_inverse_kinematics(T_sd, theta_0, M, S_mat, true, M_intermediates);

% IK using transpose
K = 1e-02 * eye(6);
[theta_f, x2, y2, z2] = J_transpose_kinematics(T_sd, theta_0, M, S_mat, true, M_intermediates, K);

% IK using RR 
k0 = 0.03; % with rng(67), k0 = 0.03 will produce decent results
[theta_f, x3, y3, z3] = redundancy_resolution(T_sd, theta_0, M, S_mat, k0, true, M_intermediates);

% IK using DLS
k = 0.5;
[theta_f, x4, y4, z4] = DLS_inverse_kinematics(T_sd, theta_0, M, S_mat, k, true, M_intermediates);

% Comparison
[FK_solution_space, T_bank_space, T_total_bank_space] = FK_space(M, S_mat, theta_0, false, true, M_intermediates);
[FK_solution_space, T_bank_space, T_total_bank_space] = FK_space(M, S_mat, theta_d, false, true, M_intermediates, false);
plot3(x1, y1, z1, '-k')
plot3(x2, y2, z2, '-r')
plot3(x3, y3, z3, '-g')
plot3(x4, y4, z4, '-b')
