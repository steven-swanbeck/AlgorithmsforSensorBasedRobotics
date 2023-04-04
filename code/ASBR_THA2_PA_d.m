% Format workspace
clc; clear; format compact; clf; close all;

disp('--------d)--------')
[M, thetas, S_mat, B_mat] = instantiate_robot("franka", true);

disp('Symbolic Space Jacobian:')
J_space = simplify(SpaceJacobian(S_mat, thetas, true))

fprintf('\n')
disp('Symbolic Body Jacobian:')
J_body = simplify(BodyJacobian(B_mat, thetas, true))
