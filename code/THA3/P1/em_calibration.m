function [T_bank, T0] = opt_calibration(raw)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
    T_bank = cell(1, size(raw, 2));

%     our defined transformation between first frame and sensor w/
%     arbitrary orientation
    R0 = eye(3);
    a_sum = [0, 0, 0];
    for i = 1:size(raw{1}, 1)
        a_sum = a_sum + raw{1}(i, :);
    end
    a_bar = a_sum ./ size(raw{1}, 1);
    P0 = a_bar;
    T0 = [R0 P0';
          zeros(1,3), 1];
    T_bank{1} = T0;

%     calculating Ts between first frame and all following frames using
%     registration, then transforming into sensor frame and accumulating A
%     and b matrices
    for i = 1:size(raw, 2) - 1
        T_bank{i + 1} = T_bank{1} * correspondence_registration(raw{i}, raw{i + 1});
    end
end