clc; clear; format compact; clf; close all;

% [package1] = read_txt('pa1-debug-a-calbody.txt'); % works (contains di, ai, ci)
% [package2] = read_txt('pa1-debug-a-calreadings.txt'); % works (contains n_frames, each of di, ai, ci)
% [package3] = read_txt('pa1-debug-a-empivot.txt'); % works (contains n_frames, each of gi)
% [package4] = read_txt('pa1-debug-a-optpivot.txt'); % works (contains n_frames, each of di, hi)
% [package5] = read_txt('pa1-debug-a-output1.txt');  % works (contains n_frames, each of ci)

[package1] = read_txt('pa1-unknown-k-calbody.txt'); % works (contains di, ai, ci)
[package2] = read_txt('pa1-unknown-k-calreadings.txt'); % works (contains n_frames, each of di, ai, ci)
[package3] = read_txt('pa1-unknown-k-empivot.txt'); % works (contains n_frames, each of gi)
[package4] = read_txt('pa1-unknown-k-optpivot.txt'); % works (contains n_frames, each of di, hi)
[package5] = read_txt('pa1-debug-a-output1.txt');  % works (contains n_frames, each of ci)

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Checking functionality of correspondence registration algorithm
a_set = package3{1};
b_set = package3{2};

[T, a_bar, b_bar] = correspondence_registration(a_set, b_set);



a1 = [a_set(4, :) 1]';
b1 = [b_set(4, :) 1]';
T * a1;
T \ b1;

% Checking equivalency of centroids
T(1:3,1:3)*a_bar' + T(1:3,4);
b_bar;
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 3)
disp('------------3)---------------')
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
FD_bank = T_bank_em;

% % Verification
% a_set = package1{1};
% b_set = em_package{8};
% plotBody(a_set, b_set)
% a_set_p = [];
% for i = 1:size(a_set, 1)
%     temp = T_bank_em{8} * [a_set(i, :) 1]';
%     a_set_p = [a_set_p; temp(1:3)'];
% end
% plotBody(a_set_p, b_set)

[T_bank_cal] = opt_calibration(package1{2}, calobject_package);
FA_bank = T_bank_cal;

% % Verification
% a_set = package1{2};
% b_set = calobject_package{8};
% plotBody(a_set, b_set)
% a_set_p = [];
% for i = 1:size(a_set, 1)
%     temp = T_bank_cal{8} * [a_set(i, :) 1]';
%     a_set_p = [a_set_p; temp(1:3)'];
% end
% plotBody(a_set_p, b_set)

% Computing c_expected
c_expected = cell(1, size(package2, 2));
for i = 1:size(package2, 2)
    c_sub = zeros(size(package2{1}{3}));
    for j = 1:size(package2{1}{3}, 1)
%         inter = FD_bank{i} \ FA_bank{i} * [calobject_package2{i}(j, :) 1]';
        inter = FD_bank{i} \ FA_bank{i} * [package1{3}(j, :) 1]';
        c_sub(j, :) = inter(1:3)';
    end
    c_expected{i} = c_sub;
end
c_expected{1} - calobject_package2{1};

% c_expected = cell(1, size(package2, 2));
% for i = 1:size(package2, 2)
%     c_sub = zeros(size(package2{1}{3}));
%     for j = 1:size(package2{1}{3}, 1)
%         temp = inv(T_bank_em{i}) * T_bank_cal{i} * [package1{3}(j, :) 1]';
%         c_sub(j, :) = temp(1:3);
%     end
%     c_expected{i} = c_sub;
% end
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 4)
disp('------------4)---------------')
[b_tip_em, b_post_em, ~] = pivot_calibration(package3)
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 5)
disp('------------5)---------------')
% first transform optical probe measurements into em frame
em_package2 = cell(1, size(package4, 2));
for i = 1:size(package4, 2)
    em_package2{i} = package4{i}{1};
end
[T_bank_em2] = opt_calibration(package1{1}, em_package2);

h_em = cell(1, size(package4, 2));
for i = 1:size(package4, 2)
    h_id = zeros(size(package4{i}{2}));
    for j = 1:size(package4{i}{2}, 1)
        p_i = T_bank_em2{i} \ [package4{i}{2}(j, :) 1]';
%         p_i = [package4{i}{2}(j, :) 1]'
        h_id(j, :) = p_i(1:3)';
    end
    h_em{i} = h_id;
end

% now perform pivot calibration in same way as before
[b_tip_opt, b_post_opt, ~] = pivot_calibration(h_em)
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Write to output file

% creating new package object (in same form as read package5)
% repackage = cell(size(package5));
repackage = cell(1, size(package2, 2) + 2);
repackage{1} = b_tip_em';
repackage{2} = b_tip_opt';

NC = size(c_expected{1}, 1);
bookmark = 0;
for i = 3:size(repackage, 2)
    repackage{i} = c_expected{i - 2};
%     repackage{i} = calobject_package2{i - 2}; % this should be wrong, but it matches the provided debug output;
end

write_txt(repackage, 'pa1-unknown-k-output1');
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Comparison
fprintf('\n\n\n')
disp('------------Comparison of Results---------------')
for i = 1:size(repackage, 2)
    fprintf('-----%i-----\n', i)
    diff = round(repackage{i}(:, :) - package5{i}(:, :), 2);
end
diff;
max(max(abs(diff)))
mean(mean(diff.^2))