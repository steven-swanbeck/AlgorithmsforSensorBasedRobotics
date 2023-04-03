% Format workspace
clc; clear; format compact; clf; close all;

set(0,'defaultTextInterpreter','latex');
set(0, 'defaultAxesTickLabelInterpreter','latex'); 
set(0, 'defaultLegendInterpreter','latex');
set(0,'defaultAxesFontSize',18);
set(0, 'DefaultLineLineWidth', 2);
set(groot, 'defaultFigureUnits', 'pixels', 'defaultFigurePosition', [440   278   560   420]);

disp('--------m)--------')
disp('Graphical simulation of the robot')

% Series of joint space configurations for the robot to reach
thetas1 = [0 -pi/4 0 -3*pi/4 0 pi/2 pi/4]; % PANDA NORMAL CONFIG
thetas2 = [0 -pi/3 0 -pi/4 0 pi/2 pi/4];
thetas3 = zeros(1,7);
thetas4 = 2 * pi * rand(1,7);

% theta_series = [thetas1' thetas2' thetas3' thetas4' thetas2' thetas1'];
theta_series = [thetas1' 2 * pi * rand(1,7)' 2 * pi * rand(1,7)' thetas1'];

animate_joint_goals("franka", theta_series)