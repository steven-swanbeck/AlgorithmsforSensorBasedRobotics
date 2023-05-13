clc; clear; format compact;

[package1] = read_txt('pa1-debug-a-calbody.txt'); % works (contains di, ai, ci)
[package2] = read_txt('pa1-debug-a-calreadings.txt'); % works (contains n_frames, each of di, ai, ci)
[package3] = read_txt('pa1-debug-a-empivot.txt'); % works (contains n_frames, each of gi)
[package4] = read_txt('pa1-debug-a-optpivot.txt'); % works (contains n_frames, each of di, hi)
[package5] = read_txt('pa1-debug-a-output1.txt');  % works (contains n_frames, each of ci)

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 3)
% Repackaging the original object for ease of use
em_package = cell(1, size(package2, 2));
for i = 1:size(package2, 2)
    em_package{i} = package2{i}{1};
end

calobject_package = cell(1, size(package2, 2));
for i = 1:size(package2, 2)
    calobject_package{i} = package2{i}{2};
end

calobject_package2 = cell(1, size(package2, 2));
for i = 1:size(package2, 2)
    calobject_package2{i} = package2{i}{3};
end

% Calibration using the optical markers
[T_bank_em] = opt_calibration(package1{1}, em_package);
FD_bank = T_bank_em

[T_bank_cal] = opt_calibration(package1{2}, calobject_package);
FA_bank = T_bank_cal

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% verifying transforms are calculated properly
% calobject_package{1} - package1{2}

FD_bank{1} * [package1{1}(1, :) 1]'
em_package{1}(1, :)'

FA_bank{1} * [package1{2}(1, :) 1]'
calobject_package{1}(1, :)'

% they are!
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T_comb = cell(size(FD_bank));
for i = 1:size(FD_bank, 2)
    T_comb{i} = FD_bank{i} \ FA_bank{i};
end
% ***note that the translation of T_comb{1} is same as answer for first c
% in frame 1, what does this mean?

calobject_package2{1} % these are the 'answers' in the document, need to check to see what the deal is here

% Computing c_expected
c_expected = cell(1, size(package2, 2));
for i = 1:size(package2, 2)
    c_sub = zeros(size(package2{1}{3}));
    for j = 1:size(package2{1}{3}, 1)
%         inter = FD_bank{i} \ FA_bank{i} * [calobject_package2{i}(j, :) 1]';
        inter = T_comb{i} * [calobject_package2{i}(j, :) 1]';
        c_sub(j, :) = inter(1:3)';
    end
    c_expected{i} = c_sub;
end

c_expected{1} - calobject_package2{1} % this is not zero, hmmmmm (but also the file doesn't seem realistic)