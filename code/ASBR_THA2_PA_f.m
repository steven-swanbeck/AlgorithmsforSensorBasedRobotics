% Format workspace
clc; clear; format compact; clf; close all;

disp('--------f)--------')
disp('Judging singularity numerically:')
[M, thetas, S_mat, B_mat, M_intermediates] = instantiate_robot();
[FK_solution_space, T_bank_space, T_total_bank_space] = FK_space(M, S_mat, thetas, false, true, M_intermediates);
J_space = SpaceJacobian(S_mat, thetas);

[is_singular, result] = singularity(J_space)

fprintf("\n")
disp('Judging singularity analytically:')
[M, thetas, S_mat, B_mat] = instantiate_robot("franka", true);
J_space = simplify(SpaceJacobian(S_mat, thetas, true));

% [is_singular, result] = singularity(J_space, true)