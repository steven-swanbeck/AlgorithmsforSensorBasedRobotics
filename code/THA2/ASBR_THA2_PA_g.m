% Format workspace
clc; clear; format compact; clf; close all;

disp('--------g)--------')
disp('Part a.')
[M, thetas, S_mat, B_mat, M_intermediates] = instantiate_robot();

% % Test cases
test_name = "Test3";
% thetas = zeros(1, 7)
% thetas = thetas
% thetas = [4.7418, 1.7343, 4.2707, 4.1161, 1.0217, 0.7477, 3.1313];
thetas = [0 0 0 ((atan(.316/.0825)+atan(.384/.0825))-pi) 0 0 0];

[FK_solution_space, T_bank_space, T_total_bank_space] = FK_space(M, S_mat, thetas, false, true, M_intermediates);
J_space = SpaceJacobian(S_mat, thetas);

% Plotting only ellpsoids without scaling down
[eigenvalues, eigenvectors] = ellipsoid_plot_angular(J_space, FK_solution_space(1:3,4), 1, true, false)
[eigenvalues, eigenvectors] = ellipsoid_plot_linear(J_space, FK_solution_space(1:3,4), 1, true, false)

disp("The rotation strategy we came up with that fit within MATLAB's built-in functions used axis-angle rotation derived from a pseudo basis and led to some incorrect ellipses being plotted, so we created these alternative functions that plot the axes of the ellipsoids and are accurately rotated for all cases.")
% Plotting only axes with scaling down
[FK_solution_space, T_bank_space, T_total_bank_space] = FK_space(M, S_mat, thetas, false, true, M_intermediates);
ellipsoid_plot_angular(J_space, FK_solution_space(1:3,4), 0.25, false);
ellipsoid_plot_linear(J_space, FK_solution_space(1:3,4), 0.5, false);

% Plotting both axes and ellipsoids
[FK_solution_space, T_bank_space, T_total_bank_space] = FK_space(M, S_mat, thetas, false, true, M_intermediates);
ellipsoid_plot_angular(J_space, FK_solution_space(1:3,4), 0.25);
ellipsoid_plot_linear(J_space, FK_solution_space(1:3,4), 0.5);

fprintf('\n')
disp('Part b.')
isotropy_angular_space = J_isotropy(J_space, "angular")
isotropy_linear_space = J_isotropy(J_space, "linear")

condition_angular_space = J_condition(J_space, "angular")
condition_linear_space = J_condition(J_space, "linear")

ellipsoid_volume_angular_space = J_ellipsoid_volume(J_space, "angular")
ellipsoid_volume_linear_space = J_ellipsoid_volume(J_space, "linear")

root = "ellipsoids_";
gif_name = strcat(root, test_name);
title(strcat('Manipulability Ellipsoids for', {' '}, test_name))
saveas(gcf, strcat(gif_name, ".png"))