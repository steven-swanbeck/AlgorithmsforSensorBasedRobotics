function [b_tip, b_post, T_bank] = pivot_calibration(raw)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    
    T_bank = cell(1, size(raw, 2));
    A = zeros(3 * size(raw, 2), 6);
    b = zeros(3 * size(raw, 2), 1);

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
    A(1:3, :) = [T_bank{1}(1:3, 1:3) -eye(3)];
    b(1:3) = -T_bank{1}(1:3, 4);

%     calculating Ts between first frame and all following frames using
%     registration, then transforming into sensor frame and accumulating A
%     and b matrices
    for i = 1:size(raw, 2) - 1
        T_bank{i + 1} = T_bank{1} * correspondence_registration(raw{i}, raw{i + 1});
        A(3 * i + 1:3 * i + 3, :) = [T_bank{i + 1}(1:3, 1:3) -eye(3)];
        b(3 * i + 1:3 * i + 3) = -T_bank{i + 1}(1:3, 4);
    end
    
%     solving b_tip and b_post
    x = (A' * A) \ (A' * b);
%     x = lsqr(A, b);
    b_tip = x(1:3);
    b_post = x(4:6);
end