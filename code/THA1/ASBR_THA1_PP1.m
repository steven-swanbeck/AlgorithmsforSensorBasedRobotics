clc; clear; format compact;

% -------------------------------------------------------------------------
% Problem 1
disp('-------------- Problem 1 --------------')

% a) converting rotation matrix to axis-angle representation
disp('-------------- a) --------------')

% Test case 1: Identity rotation
disp('---Case 1---')
R = eye(3);
[w, theta] = RotationMatrix2AxisAngle(R)

% Test case 2: trace(R) == -1 (all forms)
disp('---Case 2---')
R = [0 1 0; 1 0 0; 0 0 -1];
[w, theta] = RotationMatrix2AxisAngle(R)

R = [-1 0 0; 0 0 -1; 0 1 0];
[w, theta] = RotationMatrix2AxisAngle(R)

R = [0 0 1; 0 -1 0; 1 0 0];
[w, theta] = RotationMatrix2AxisAngle(R)

% Test case 3: arbitrary rotation with known expected result
disp('---Case 3---')
angle = pi/2;
R = [cos(angle) -sin(angle) 0; sin(angle) cos(angle) 0; 0 0 1];
[w, theta] = RotationMatrix2AxisAngle(R)

% b) converting rotation matrix to quaternion representation
disp('-------------- b) --------------')

% Test 1: Identity rotation
disp('---Case 1---')
R = eye(3)
[q] = RotationMatrix2Quaternion(R)

% Test 2: arbitrary rotation with known expected result
disp('---Case 2---')
angle = pi/6;
R = [cos(angle) -sin(angle) 0; sin(angle) cos(angle) 0; 0 0 1]
[q] = RotationMatrix2Quaternion(R)

% c) converting rotation matrix to ZYZ and roll-pitch-yaw representation
disp('-------------- c) --------------')

% ZYZ
disp('-------------- ZYZ --------------')

% Test 1: Identity rotation
disp('---Case 1---')
R = eye(3);
[phi, theta, psi] = RotationMatrix2ZYZAngles(R)

% Test 2: theta = 0, pi
disp('---Case 2---')
phi = pi/6
theta = 0
psi = -pi/2
R = [cos(phi)*cos(theta)*cos(psi) - sin(phi)*sin(psi) -1*cos(phi)*cos(theta)*sin(psi) - sin(phi)*cos(psi) cos(phi)*sin(theta);
    sin(phi)*cos(theta)*cos(psi) + cos(phi)*sin(psi) -1*sin(phi)*cos(theta)*sin(psi) + cos(phi)*cos(psi) sin(phi)*sin(theta);
    -1*sin(theta)*cos(psi) sin(theta)*sin(psi) cos(theta)];

[phi, theta, psi] = RotationMatrix2ZYZAngles(R)

% Test 3: arbitrary rotation with known expected result
disp('---Case 3---')
phi = pi/6
theta = pi/4
psi = -pi/2
R = [cos(phi)*cos(theta)*cos(psi) - sin(phi)*sin(psi) -1*cos(phi)*cos(theta)*sin(psi) - sin(phi)*cos(psi) cos(phi)*sin(theta);
    sin(phi)*cos(theta)*cos(psi) + cos(phi)*sin(psi) -1*sin(phi)*cos(theta)*sin(psi) + cos(phi)*cos(psi) sin(phi)*sin(theta);
    -1*sin(theta)*cos(psi) sin(theta)*sin(psi) cos(theta)];

[phi, theta, psi] = RotationMatrix2ZYZAngles(R)

% RPY
disp('-------------- RPY --------------')

% Test 1: theta = pi/2
disp('---Case 1---')
roll = pi/4
pitch = -pi/2
yaw = -pi/3
R = [cos(roll)*cos(pitch) cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw) cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw);
    sin(roll)*cos(pitch) sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw) sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw);
    -1*sin(pitch) cos(pitch)*sin(yaw) cos(pitch)*cos(yaw)];

[roll, pitch, yaw] = RotationMatrix2RPYAngles(R)

% Test 2: arbitrary rotation with known expected result
disp('---Case 2---')
roll = pi/4
pitch = -pi/3
yaw = -pi/3
R = [cos(roll)*cos(pitch) cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw) cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw);
    sin(roll)*cos(pitch) sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw) sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw);
    -1*sin(pitch) cos(pitch)*sin(yaw) cos(pitch)*cos(yaw)];

[roll, pitch, yaw] = RotationMatrix2RPYAngles(R)