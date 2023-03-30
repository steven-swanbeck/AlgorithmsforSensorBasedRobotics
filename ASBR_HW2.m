clc; clear; format compact;

disp('-------------3-------------')
% 3

disp('-----------a-----------')
% a)
% syms L
% L = sym('L','real');
L = 3

S1 = [0 0 1 -L 0 0]';
S2 = [1 0 0 0 0 L]';
S3 = [0 0 1 0 0 0]';
S4 = [0 0 0 0 1 0]';

S_mat = [S1 S2 S3 S4];
thetas = [0 0 pi/2 L]';

Jb = BodyJacobian(S_mat, thetas)

disp('-----------b-----------')
% b)
theta_dots = [1 1 1 1]';
Vb = Jb * theta_dots;

p_dot = Vb(4:6)



disp('-------------4-------------')
% 4

disp('-----------a-----------')
% a)
L = 1

S1 = [0 0 1 0 0 0]';
S2 = [0 1 0 0 0 0]';
S3 = [-1 0 0 0 0 0]';
S4 = [-1 0 0 0 0 -L]';
S5 = [-1 0 0 0 0 -2*L]';
S6 = [0 1 0 0 0 0]';

S_mat = [S1 S2 S3 S4 S5 S6];
thetas = [0 0 0 0 0 0]';

Js = SpaceJacobian(S_mat, thetas)

disp('-----------b-----------')
% b)
% Kinematic singularities
