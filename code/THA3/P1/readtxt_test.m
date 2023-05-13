clc; clear all; format compact; 

% [package1] = read_txt('pa1-debug-a-calbody.txt'); % works (contains di, ai, ci)
[package2] = read_txt('pa1-debug-a-calreadings.txt'); % works (contains n_frames, each of di, ai, ci)
% [package3] = read_txt('pa1-debug-a-empivot.txt'); % works (contains n_frames, each of gi)
% [package4] = read_txt('pa1-debug-a-optpivot.txt'); % works (contains n_frames, each of di, hi)
% [package5] = read_txt('pa1-debug-a-output1.txt');  % works (contains n_frames, each of ci)

% problem 1, part 3
% find centroid of optimal tracking markers on em base, use them with
% single point to define translation between base and tracker, defining its
% frame
[package1] = read_txt('pa1-debug-a-calbody.txt');



% a) calculating FD
d_set = package1{1};

d_sum = [0, 0, 0];
for i = 1:size(d_set, 1)
    d_sum = d_sum + d_set(i, :);
end
d_bar = d_sum ./ size(d_set, 1);

d_tilda = {};
for i = 1:size(d_set, 1)
    d_tilda{i} = d_set(i, :) - d_bar;
end

pD = {};
TD = {};
for i = 1:size(d_set, 1)
    pD{i} = (d_set(i, :) - d_tilda{i})';
    TD{i} = [eye(3) pD{i};
            zeros(1, 3) 1];
end

% these are all the same, so we'll say
FD = TD{1}



% b) calculating FA
a_set = package1{2};

a_sum = [0, 0, 0];
for i = 1:size(a_set, 1)
    a_sum = a_sum + a_set(i, :);
end
a_bar = a_sum ./ size(a_set, 1);

a_tilda = {};
for i = 1:size(a_set, 1)
    a_tilda{i} = a_set(i, :) - a_bar;
end

pA = {};
TA = {};
for i = 1:size(a_set, 1)
    pA{i} = (a_set(i, :) - a_tilda{i})';
    TA{i} = [eye(3) pA{i};
            zeros(1, 3) 1];
end

% these are all the same, so we'll say
FA = TA{1}



% c) calculating c_expected
c_set = package1{3};
c_expected = zeros(size(c_set));
for i = 1:size(c_set, 1)
    c_ex = FD \ FA * [c_set(i, :) 0]';
    c_expected(i, :) = c_ex(1:3)';
end

c_set - c_expected
