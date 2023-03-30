% Setup
clc; clear; format compact; 
clf; close all;

set(0,'defaultTextInterpreter','latex');
set(0, 'defaultAxesTickLabelInterpreter','latex'); 
set(0, 'defaultLegendInterpreter','latex');
set(0,'defaultAxesFontSize',18);
set(0, 'DefaultLineLineWidth', 2);
set(groot, 'defaultFigureUnits', 'pixels', 'defaultFigurePosition', [440   278   560   420]);


L1 = 0.333; % m
L2 = 0.316; % m
L3 = 0.384; % m
L4 = 0.088; % m
L5 = 0.107; % m
a = 0.0825; % m

% Base configuration
M_rot = [1 0 0;
        0 -1 0;
        0 0 -1];
M_trans = [L4;
            0;
            (L1 + L2 + L3 - L5)];
M = [M_rot M_trans;
    zeros(1,3) 1];

% Fixed frame twists
S1 = [0 0 1 0 0 0]';
S2 = [0 1 0 -L1 0 0]';
S3 = [0 0 1 0 0 0]';
S4 = [0 -1 0 (L1 + L2) 0 -a]';
S5 = [0 0 1 0 0 0]';
S6 = [0 -1 0 (L1 + L2 + L3) 0 0]';
S7 = [0 0 -1 0 L4 0]';

S_mat = [S1 S2 S3 S4 S5 S6 S7];

% Body frame twists
B1 = [0 0 -1 0 -L4 0]';
B2 = [0 -1 0 L2 + L3 - L5 0 L4]';
B3 = [0 0 -1 0 -L4 0]';
B4 = [0 1 0 L5 - L3 0 -L4 + a]';
B5 = [0 0 -1 0 -L4 0]';
B6 = [0 1 0 L5 0 -L4]';
B7 = [0 0 1 0 0 0]';

B_mat = [B1 B2 B3 B4 B5 B6 B7];

% Adding M-type configuration of each joint to enable plotting
M1 = [eye(3) [0 0 L1]';
    zeros(1,3), 1];

M2_rot = [1 0 0;
          0 0 1;
          0 -1 0];
M2_trans = [0 0 L1]';
M2 = [M2_rot M2_trans;
    zeros(1,3), 1];

M3 = [eye(3) [0 0 L1 + L2]';
    zeros(1,3), 1];

M4_rot = [1 0 0;
          0 0 -1;
          0 1 0];
M4_trans = [a 0 L1 + L2]';
M4 = [M4_rot M4_trans;
    zeros(1,3), 1];

M5 = [eye(3) [0 0 L1 + L2 + L3]';
    zeros(1,3), 1];

M6_rot = [1 0 0;
          0 0 -1;
          0 1 0];
M6_trans = [0 0 L1 + L2 + L3]';
M6 = [M6_rot M6_trans;
    zeros(1,3), 1];

M7_rot = [1 0 0;
          0 -1 0;
          0 0 -1];
M7_trans = [L4 0 L1 + L2 + L3]';
M7 = [M7_rot M7_trans;
    zeros(1,3), 1];

M_intermediates = {M1, M2, M3, M4, M5, M6, M7};

% Joint Angles
thetas1 = [0 -pi/4 0 -3*pi/4 0 pi/2 pi/4]; % PANDA NORMAL CONFIG
thetas2 = [0 -pi/3 0 -pi/4 0 pi/2 pi/4];
thetas3 = zeros(1,7);


% Series of poses to visit
theta_series = [thetas1' thetas2' thetas3' thetas2' thetas1'];
% theta_series = [thetas1' thetas3' thetas1'];

figure
view(3)
box on
hold on
grid on
axis equal;
xlabel('x'), ylabel('y'), zlabel('z');
xlim([-1, 1]);
ylim([-1, 1]);
zlim([0, 1.5]);
view([30, 15]);

num_steps = 100;
delay = 0.1;

for i = 1:size(theta_series, 2) - 1
    current_thetas = theta_series(:, i);
    delta_theta = zeros(size(theta_series, 1), 1);
    for j = 1:size(theta_series, 1)
        delta_theta(j) = (theta_series(j, i + 1) - theta_series(j, i)) / num_steps;
        delta_theta;
    end
    for j = 1:num_steps
        cla;
        [FK_solution_space, T_bank_space, T_total_bank_space] = FK_space(M, S_mat, current_thetas, false, true, M_intermediates, false);
        J_space = SpaceJacobian(S_mat, current_thetas);
        ellipsoid_plot_angular(J_space, FK_solution_space(1:3,4), 0.25);
        ellipsoid_plot_linear(J_space, FK_solution_space(1:3,4), 0.5);
        pause(delay);

        current_thetas = current_thetas + delta_theta;
    end
end
