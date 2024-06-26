clc; clear; format compact;

% USING ORIGINAL DATA
[q_Robot_config, q_camera_config, t_Robot_config, t_camera_config] = data_quaternion();

% % USING NOISY DATA
% [q_Robot_config, q_camera_config, t_Robot_config, t_camera_config] = data_quaternion_noisy();

% % LOOKING ONLY AT PART OF NOISY DATA
% q_Robot_config = q_Robot_config(1:5, :);
% q_camera_config = q_camera_config(1:5, :);
% t_Robot_config = t_Robot_config(1:5, :);
% t_camera_config = t_camera_config(1:5, :);

[T, P_all, Ta, Tb] = EyeinHand_Calibration(q_Robot_config, q_camera_config, t_Robot_config, t_camera_config);

norms = zeros(1, size(P_all, 2));
for i = 1:size(P_all, 2)
    norms(i) = norm(P_all(:, i));
end

Tx = cell(1, size(P_all, 2));
for i = 1:size(Tx, 2)
    Tx{i} = [T(1:3, 1:3) P_all(:, i); zeros(1, 3) 1];
end

Ra = cell(1, size(P_all, 2));
for i = 1:size(Ta, 2)
    Ra{i} = Ta{i}(1:3, 1:3);
end
Rb = cell(1, size(P_all, 2));
for i = 1:size(Tb, 2)
    Rb{i} = Tb{i}(1:3, 1:3);
end

error = [];
for i = 1:size(Tx, 2)
%     disp('------')
%     a = Ta{i} * Tx{i};
%     b = Tx{i} * Tb{i};
%     a - b
%     a = Ra{i} * Tx{i}(1:3, 1:3);
%     b = Tx{i}(1:3, 1:3) * Rb{i};

%     a = Ra{i} * T(1:3, 1:3);
%     b = T(1:3, 1:3) * Rb{i};
%     b - a

    a = Ta{i} * T;
    b = T * Tb{i};
    error = [error; (b - a)];
end

T

mse = 1/16 * sum(sum(error.^2))
mse = mean(mean(error.^2)) % 0.0097 for T (which is lower than any of the calculated Tx{i}