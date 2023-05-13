clc; clear; format compact; close all;

% Hand-Eye Calibration

disp('----------------1)----------------')
% 1)
% a)
[q_Robot_config, q_camera_config, t_Robot_config, t_camera_config] = data_quaternion();

% [M_i] = M_quat(q_camera_config(1,:)', q_Robot_config(1,:)');

[M_set] = M_set(q_camera_config, q_Robot_config);
[U, S, V] = svd(M_set);

% U * S * V' - M_set
% norm(V(:,4))
% norm(V(4,:))

quat = V(:,end)

