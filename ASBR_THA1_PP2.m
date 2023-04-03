clc; clear; format compact;
% Problem 2
disp('-------------- Problem 2 --------------')

% a) convert axis-angle representation into equivalent rotation matrix
disp('-------------- a) --------------')

% Test cases
w = [0.7071 0 0.7071]';
theta = 3.1416;
R = AxisAngle2RotationMatrix(w, theta)

w = [0 0.866 0.5]';
theta = 30*pi/180;
R = AxisAngle2RotationMatrix(w, theta)

w = [0 0 1]';
theta = 1.5708;
R = AxisAngle2RotationMatrix(w, theta)

% b) convert quaternion representation into equivalent rotation matrix
disp('-------------- b) --------------')

% Test cases
q = [0.9659 0 0 0.2588]'
R = Quaternion2RotationMatrix(q)

q = [0.5 0.5 0.5 0.5]'
R = Quaternion2RotationMatrix(q)

q = [1 0 0 0]'
R = Quaternion2RotationMatrix(q)