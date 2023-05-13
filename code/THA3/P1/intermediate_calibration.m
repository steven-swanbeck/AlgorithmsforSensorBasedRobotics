function [output, T0] = em_calibration(raw)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
    output = cell(size(raw, 2));

    
    R0 = eye(3);
    a_sum = [0, 0, 0];
    for i = 1:size(raw{1}, 1)
        a_sum = a_sum + raw{1}{1}(i, :);
    end
    a_bar = a_sum ./ size(raw{1}{1}, 1);
    P0 = a_bar;
    T0 = [R0 P0';
          zeros(1,3), 1];
    T_bank{1} = T0;
    A(1:3, :) = [T_bank{1}(1:3, 1:3) -eye(3)];
    b(1:3) = -T_bank{1}(1:3, 4);

%     for i = 1:size(raw, 2)
%         
%     end
end