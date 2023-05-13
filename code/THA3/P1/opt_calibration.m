function [T_bank] = opt_calibration(calbody, calreading)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
    T_bank = cell(1, size(calreading, 2));

%     calculating Ts between frames using registration
    for i = 1:size(calreading, 2)
        T_bank{i} = correspondence_registration(calbody, calreading{i});
    end
end