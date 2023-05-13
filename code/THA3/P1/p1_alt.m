clc; clear; format compact; clf; close all;

[package1] = read_txt('pa1-debug-g-calbody.txt'); % works (contains di, ai, ci)
[package2] = read_txt('pa1-debug-g-calreadings.txt'); % works (contains n_frames, each of di, ai, ci)
[package3] = read_txt('pa1-debug-g-empivot.txt'); % works (contains n_frames, each of gi)
[package4] = read_txt('pa1-debug-g-optpivot.txt'); % works (contains n_frames, each of di, hi)
[package5] = read_txt('pa1-debug-g-output1.txt');  % works (contains n_frames, each of ci)


% Checking functionality of correspondence registration algorithm
a_set = package3{3}; % NOTICE THESE ARE DISTORTED FOR SOME (LIKE G)
b_set = package3{4};
a_set = package4{3}{2}; % AND THESE ARE NOT
b_set = package4{4}{2};

[T, a_bar, b_bar] = correspondence_registration(a_set, b_set);

a1 = [a_set(3, :) 1]';
b1 = [b_set(3, :) 1]';
b1 - T * a1
a1 - T \ b1

% Checking equivalency of centroids
T(1:3,1:3)*a_bar' + T(1:3,4) - b_bar'


plotBody(a_set, b_set)

a_set_p = [];
for i = 1:size(a_set, 1)
    temp = T * [a_set(i, :) 1]';
    a_set_p = [a_set_p; temp(1:3)'];
end
 
plotBody(a_set_p, b_set)


% disp('------------3)---------------')
% % Repackaging the original object for ease of use
% em_package_opt = cell(1, size(package2, 2));
% for i = 1:size(package2, 2)
%     em_package_opt{i} = package2{i}{1};
% end
% 
% calobject_package_opt = cell(1, size(package2, 2));
% for i = 1:size(package2, 2)
%     calobject_package_opt{i} = package2{i}{2};
% end
% 
% calobject_package_em = cell(1, size(package2, 2));
% for i = 1:size(package2, 2)
%     calobject_package_em{i} = package2{i}{3};
% end
% 
% % Calibration using the optical markers
% [T_bank_em] = opt_calibration(package1{1}, em_package_opt);
% FD_bank = T_bank_em
% 
% [T_bank_cal] = opt_calibration(package1{2}, calobject_package_opt);
% FA_bank = T_bank_cal
% 
% % Computing c_expected
% c_expected = cell(1, size(package2, 2));
% for i = 1:size(package2, 2)
%     c_sub = zeros(size(package2{1}{3}));
%     for j = 1:size(package2{1}{3}, 1)
% %         inter = FD_bank{i} \ FA_bank{i} * [calobject_package2{i}(j, :) 1]';
% %         inter = FD_bank{i} \ FA_bank{i} * [package1{3}(j, :) 1]';
%         inter = FD_bank{i} \ FA_bank{i} * [package1{3}(j, :) 1]';
%         c_sub(j, :) = inter(1:3)';
%     end
%     c_expected{i} = c_sub;
% end
% 
% % c_expected{1} - calobject_package2{1}
% 
% c_expected{1}