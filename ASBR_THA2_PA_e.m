% Format workspace
clc; clear; format compact; clf; close all;

disp('--------e)--------')
[M, thetas, S_mat, B_mat, M_intermediates] = instantiate_robot();

disp('Numeric Space Jacobian:')
J_space = SpaceJacobian(S_mat, thetas)

fprintf("\n")
disp('Numeric Body Jacobian:')
J_body = BodyJacobian(B_mat, thetas)

fprintf("\n")
disp('Demonstrating equivalency:')
[FK_solution_space, T_bank_space, T_total_bank_space] = FK_space(M, S_mat, thetas, false, false, M_intermediates);
J_diff = J_space - Ad(FK_solution_space) * J_body