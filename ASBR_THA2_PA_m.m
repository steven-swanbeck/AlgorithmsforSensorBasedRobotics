% Format workspace
clc; clear; format compact; clf; close all;

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