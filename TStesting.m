clc; clear; format compact; clf; close all;

% theta = pi/6;
% theta2 = pi/3;
% 
% Tsb = [cos(theta) -sin(theta) 0 1;
%     sin(theta) cos(theta) 0 2;
%     0 0 1 0;
%     0 0 0 1];
% 
% Tsc = [cos(theta2) -sin(theta2) 0 2;
%     sin(theta2) cos(theta2) 0 1;
%     0 0 1 0;
%     0 0 0 1];
% 
% T = Tsc * inv(Tsb);
% 
% [S, theta] = T2S(T)


% TTTTT
[M, thetas, S_mat, B_mat, M_intermediates] = instantiate_robot();

% theta0 = zeros(1,7) + 0.05;
% theta_0 = [0 pi/3 0 0 0 0 0];
% theta_d = [0 pi/2 0 0 0 0 0]; % matrix is singular when set to zero, use this as demonstration case for later

theta_0 = [2*pi/3 pi/6 0 -pi/4 pi/4 -pi/2 0];
theta_d = [0 pi/2 0 0 0 0 0]; % matrix is singular when set to zero, use this as demonstration case for later

[FK_solution_space, T_bank_space, T_total_bank_space] = FK_space(M, S_mat, theta_0, false, true, M_intermediates);
[FK_solution_space, T_bank_space, T_total_bank_space] = FK_space(M, S_mat, theta_d, false, true, M_intermediates, false);

target_orientation = FK_solution_space(1:3, 1:3);
target_position = FK_solution_space(1:3, 4);
T_sd = [target_orientation target_position;
    zeros(1,3), 1];

[T_sb] = FK_space(M, S_mat, theta_0, false, true, M_intermediates);
pause(1);

% for i = 1:30000
%     alpha_0 = 1e-2;
%     rate = 1;
% %     [T_sb] = FK_space(M, S_mat, theta_0)
%     cla;
%     [T_sb] = FK_space(M, S_mat, theta_0, false, true, M_intermediates, false);
%     pause(0.1);
% 
%     J_s = SpaceJacobian(S_mat, theta_0);
%     J_b = Ad(inv(T_sb)) * J_s;
%     
%     J_diff = J_s - Ad(T_sb) * J_b;
%     
%     T_bd = T_sb \ T_sd;
%     nu_b = T2S(T_bd);
%     delta_theta = alpha_0 * J_dagger(J_b) * nu_b;
%     theta_0 = theta_0 + delta_theta';
%     
% end
% 
% theta_0

[theta_f] = J_inverse_kinematics(T_sd, theta_0, M, S_mat, true, M_intermediates)
% [FK_solution_space, T_bank_space, T_total_bank_space] = FK_space(M, S_mat, theta_f, false, true, M_intermediates, false);