clc; clear; format compact;

% syms x y z w
% A = [x y; z w]
% % A = [1 2; 3 4];
% [V, D] = eig(A)

% 3a
syms w1 w2 w3 theta

w_hat = [0 -w3 w2; w3 0 -w1; -w2 w1 0]
[V, D] = eig(w_hat)
disp('------------------------------------------------------')

% 3b
% syms w1 w2 w3 theta 
R = eye(3) + w_hat * sin(theta) + (w_hat^2) * (1 - cos(theta))
[V, D] = eig(R)
disp('------------------------------------------------------')

test1 = D(1,1) * V(:,1);
test2 = R * V(:,1);

disp('----------------------------4-------------------------')
% 4
% a
syms w1 w2 w3 r11 r12 r13 r21 r22 r23 r31 r32 r33

R = [r11 r12 r13; r21 r22 r23; r31 r32 r33];
w = [w1; w2; w3];

R_w = R * w;
% R_w_hat = screwsymmetricmatrix(R_w)
R_w_hat = [0 -1*R_w(3) R_w(2); R_w(3) 0 -1*R_w(1); -1*R_w(2) R_w(1) 0]

w_hat = [0 -1*w3 w2; w3 0 -1*w1; -1*w2 w1 0];
R*w_hat*transpose(R);
R_w_hat_R_T = simplify(R*w_hat*transpose(R))
disp('------------------------------------------------------')



disp('--------------------------5a---------------------------')
% 5
% a)
% Closure
syms q0 q1 q2 q3 p0 p1 p2 p3;

c0 = q0*p0 - q1*p1 - q2*p2 - q3*p3;
c1 = q0*p1 + p0*q1 + q2*p3 - q3*p2;
c2 = q0*p2 + p0*q2 - q1*p3 + q3*p1;
c3 = q0*p3 + p0*q3 + q1*p2 - q2*p1;

simplify(expand(c0^2 + c1^2 + c2^2 + c3^2))

q0 = sym('q0','real');
q1 = sym('q1','real');
q2 = sym('q2','real');
q3 = sym('q3','real');
p0 = sym('p0','real');
p1 = sym('p1','real');
p2 = sym('p2','real');
p3 = sym('p3','real');
w0 = sym('w0','real');
w1 = sym('w1','real');
w2 = sym('w2','real');
w3 = sym('w3','real');

Q = [q0 q1 q2 q3];
P = [p0 p1 p2 p3];
W = [w0 w1 w2 w3];

PW = quatmultiply(P, W);
Q_PW = quatmultiply(Q, PW);
test1 = simplify(expand(Q_PW))

QP = quatmultiply(Q, P);
QP_W = quatmultiply(QP, W);
test2 = simplify(expand(QP_W))

tf = zeros(4,1,'logical');
for i = 1:4
    tf(i) = isequal(test1(i), test2(i));
end
tf


disp('--------------------------5b---------------------------')
% b)
% syms q0 q1 q2 q3 x1 x2 x3
q0 = sym('q0','real');
q1 = sym('q1','real');
q2 = sym('q2','real');
q3 = sym('q3','real');
x1 = sym('x1','real');
x2 = sym('x2','real');
x3 = sym('x3','real');

Q = [q0 q1 q2 q3];
X = [0 x1 x2 x3];

q = [q1 q2 q3]';
x = [x1 x2 x3]';

Q_star = [q0 -1*q'];

m1 = quatmultiply(Q, X);
m2 = quatmultiply(m1, Q_star);
test1 = simplify(expand(m2))

% show vector part equals formula
vector_part = (q0^2 - dot(q, q))*x + 2*(q0*cross(q, x) + dot(x, q)*q);
test2 = simplify(expand(vector_part))

tf = zeros(3,1,'logical');
for i = 1:3
    tf(i) = isequal(test1(i + 1), test2(i));
end
tf
disp('------------------------------------------------------')

disp('-------------------------6a?----------------------------')
% 6
syms phi theta psi;

Rx = [1 0 0; 
    0 cos(phi) -sin(phi);
    0 sin(phi) cos(phi)];

Ry = [cos(theta) 0 sin(theta);
    0 1 0;
    -sin(theta) 0 cos(theta)];

Rz = [cos(psi) -sin(psi) 0;
    sin(psi) cos(psi) 0;
    0 0 1];

R = Rx * Ry * Rz

disp('------------------------or-6a?----------------------------')
% a)
% Rxyz(psi)

Rx_psi = [1 0 0; 
    0 cos(psi) -sin(psi);
    0 sin(psi) cos(psi)];

Ry_psi = [cos(psi) 0 sin(psi);
    0 1 0;
    -sin(psi) 0 cos(psi)];

Rz_psi = [cos(psi) -sin(psi) 0;
    sin(psi) cos(psi) 0;
    0 0 1];

Rxyz_psi = Rx_psi * Ry_psi * Rz_psi

simplify(expand(Rxyz_psi));

disp('-------------------------6b----------------------------')
% b)
% Inverse Rxyz-1
disp('------------------------------------------------------')


disp('-------------------------7----------------------------')
Tsa = [0 -1 0 3;
    0 0 -1 0;
    1 0 0 0;
    0 0 0 1];

Tsa_inv = [0 0 1 0;
    -1 0 0 -3;
    0 -1 0 0;
    0 0 0 1];

Vs = [3, 2, 1, -1, -2, -3];

Vs_mat = [0 -1 2 -1;
    1 0 -3 -2;
    -2 3 0 -3;
    0 0 0 1];

Va_mat = Tsa_inv * Vs_mat * Tsa

% a)
figure
plot3(0, 0, 0, '.k', 1, 0, 0,'.k', 0, 1, 0,'.k', 0, 0, 1,'.k', markersize=10)
view(3)
box on
hold on
grid on
xlabel('x'), ylabel('y'), zlabel('z');

% {s}
plotFrame_T(eye(4), 's');

% {a}
x_ax = [0 0 1]';
y_ax = [-1 0 0]';
z_ax = [0 -1 0]';
R = [x_ax y_ax z_ax];
p = [3 0 0]';
Ta = [R p; zeros(1,3) 1];
plotFrame_T(Ta, 'a');

% {b}
x_ax = [1 0 0]';
y_ax = [0 0 -1]';
z_ax = [0 1 0]';
R = [x_ax y_ax z_ax];
p = [0 2 0]';
Tb = [R p; zeros(1,3) 1];
plotFrame_T(Tb, 'b');



% P7, Part i
Ti = [0 -1 0 3; 
    0 0 -1 0; 
    1 0 0 0; 
    0 0 0 1];

Ri = Ti(1:3, 1:3);
Pi = Ti(1:3, 4);

theta = acos((trace(Ri) - 1) / 2)
% theta = 2 * pi / 3;
% theta = 4 * pi / 3;

w_hat = 1 / (2*sin(theta)) * (Ri - Ri')

w = [w_hat(3,2), w_hat(2,3), w_hat(2,1)]'

G = (1/theta) * eye(3) - 1/2 * w_hat + (1/theta - 1/2 * cot(theta/2)) * w_hat^2

v = G * Pi

S = [w;v]

R = eye(3) + sin(theta) * screwsymmetricmatrix(w) + (1 - cos(theta)) * screwsymmetricmatrix(w)^2;
P = (eye(3) * theta + (1 - cos(theta)) * screwsymmetricmatrix(w) + (theta - sin(theta)) * screwsymmetricmatrix(w)^2) * v;
T = [R P; 0 0 0 1]

% h = (-4*pi + 3*sqrt(3)) / (6*pi)
h = w' * v
s_hat = w

q = Axis2SkewSymmetricMatrix(s_hat) * (v - h * s_hat)

% q = [1.5, 0, 0]';

% check to make sure point works
w = s_hat;
v = cross(-s_hat * 1, q) + h * s_hat * 1;

S = [w; v]

figure
plot3(0, 0, 0, '.k', 1, 0, 0,'.k', 0, 1, 0,'.k', 0, 0, 1,'.k', markersize=10)
view(3)
box on
hold on
grid on
xlabel('x'), ylabel('y'), zlabel('z');

% {s}
plotFrame_T(eye(4), 's');

% S
plotScrewAxis_qsh(q, s_hat, h)



% P7, Part j
S_theta = [0, 1, 2, 3, 0, 0]';

theta = sqrt(5);
S = S_theta ./ theta;

w = S(1:3);
v = S(4:6);

h = w' * v;
s_hat = w;
q = Axis2SkewSymmetricMatrix(s_hat) * (v - h * s_hat);

T = S2T(S, theta);

figure
plot3(0, 0, 0, '.k', 1, 0, 0,'.k', 0, 1, 0,'.k', 0, 0, 1,'.k', markersize=10)
view(3)
box on
hold on
grid on
xlabel('x'), ylabel('y'), zlabel('z');

% {s}
plotFrame_T(eye(4), 's');

% {b}
plotFrame_T(T, 'b');

% S
plotScrewAxis_qsh(q, s_hat, h)
disp('------------------------------------------------------')



disp('-------------------------8----------------------------')
plc = 1/sqrt(3);
w_hat = [0, -plc, -plc;
    plc, 0, -plc;
    plc, plc, 0];

w_hat^2
disp('------------------------------------------------------')