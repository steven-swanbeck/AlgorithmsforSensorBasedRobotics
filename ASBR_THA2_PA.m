% Setup
clc; clear; format compact; 
clf; close all;

set(0,'defaultTextInterpreter','latex');
set(0, 'defaultAxesTickLabelInterpreter','latex'); 
set(0, 'defaultLegendInterpreter','latex');
set(0,'defaultAxesFontSize',18);
set(0, 'DefaultLineLineWidth', 2);
set(groot, 'defaultFigureUnits', 'pixels', 'defaultFigurePosition', [440   278   560   420]);



% Hand calc parts
disp('-----------Preliminaries-----------')
% robot = "panda" % can be panda or test
robot = "test"

if robot == "panda"
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
        zeros(1,3) 1]
    
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
    thetas = [0 -pi/4 0 -3*pi/4 0 pi/2 pi/4]; % PANDA NORMAL CONFIG
%     thetas = [0 -pi/3 0 -3*pi/4 0 pi/2 pi/4];
%     thetas = zeros(1,7);

    

%     Symbolic Definitions
    L1_s = sym('L1_s', 'real');
    L2_s = sym('L2_s', 'real');
    L3_s = sym('L3_s', 'real');
    L4_s = sym('L4_s', 'real');
    L5_s = sym('L5_s', 'real');
    a_s = sym('a_s', 'real');

    theta1_s = sym('theta1_s', 'real');
    theta2_s = sym('theta2_s', 'real');
    theta3_s = sym('theta3_s', 'real');
    theta4_s = sym('theta4_s', 'real');
    theta5_s = sym('theta5_s', 'real');
    theta6_s = sym('theta6_s', 'real');
    theta7_s = sym('theta7_s', 'real');
    
    % Base configuration
    M_rot_s = [1 0 0;
            0 -1 0;
            0 0 -1];
    M_trans_s = [L4_s;
                0;
                (L1_s + L2_s + L3_s - L5_s)];
    M_s = [M_rot_s M_trans_s;
        zeros(1,3) 1]
    
    % Fixed frame twists
    S1_s = [0 0 1 0 0 0]';
    S2_s = [0 1 0 -L1_s 0 0]';
    S3_s = [0 0 1 0 0 0]';
    S4_s = [0 -1 0 (L1_s + L2_s) 0 -a_s]';
    S5_s = [0 0 1 0 0 0]';
    S6_s = [0 -1 0 (L1_s + L2_s + L3_s) 0 0]';
    S7_s = [0 0 -1 0 L4_s 0]';
    
    S_mat_s = [S1_s S2_s S3_s S4_s S5_s S6_s S7_s];
    
    % Body frame twists
    B1_s = [0 0 -1 0 -L4_s 0]';
    B2_s = [0 -1 0 L2_s + L3_s - L5_s 0 L4_s]';
    B3_s = [0 0 -1 0 -L4_s 0]';
    B4_s = [0 1 0 L5_s - L3_s 0 -L4_s + a_s]';
    B5_s = [0 0 -1 0 -L4_s 0]';
    B6_s = [0 1 0 L5_s 0 -L4_s]';
    B7_s = [0 0 1 0 0 0]';
    
    B_mat_s = [B1_s B2_s B3_s B4_s B5_s B6_s B7_s];

    % Joint Angles
    thetas_s = [theta1_s, theta2_s, theta3_s, theta4_s, theta5_s, theta6_s, theta7_s];

elseif robot == "test"
    L1 = 1; % m
    L2 = 1; % m
    
    % Base configuration
    M_rot = [1 0 0;
            0 1 0;
            0 0 1];
    M_trans = [L1+L2;
                0;
                0];
    M = [M_rot M_trans;
        zeros(1,3) 1];
    
    % Fixed frame twists
    S1 = [0 0 1 0 0 0]';
    S2 = [0 0 1 0 -L1 0]';
    S3 = [0 0 1 0 -L1-L2 0]';
    
    
    S_mat = [S1 S2 S3];
    
    % Body frame twists
    B1 = [0 0 1 0 L1+L2 0]';
    B2 = [0 0 1 0 L2 0]';
    B3 = [0 0 1 0 0 0]';
    
    
    B_mat = [B1 B2 B3];
    
    % Adding M-type configuration of each joint to enable plotting
    M1 = [eye(3) [0 0 0]';
        zeros(1,3), 1];
    
    M2_rot = [1 0 0;
              0 1 0;
              0 0 1];
    M2_trans = [L1 0 0]';
    M2 = [M2_rot M2_trans;
        zeros(1,3), 1];
    
    M3_rot = [1 0 0;
              0 1 0;
              0 0 1];
    M3_trans = [L1+L2 0 0]';
    M3 = [M3_rot M3_trans;
        zeros(1,3), 1];
    
    M_intermediates = {M1, M2, M3};
    
    % Joint Angles
    thetas = [0 pi/4 0]; % primary config
%     thetas = [0 3*pi/4 0]; % primary config
    % thetas = zeros(1,3);



%     symbolic
    L1_s = sym('L1_s', 'real');
    L2_s = sym('L2_s', 'real');
    
    % Base configuration
    M_rot_s = [1 0 0;
            0 1 0;
            0 0 1];
    M_trans_s = [L1_s+L2_s;
                0;
                0];
    M_s = [M_rot_s M_trans_s;
        zeros(1,3) 1];
    
    % Fixed frame twists
    S1_s = [0 0 1 0 0 0]';
    S2_s = [0 0 1 0 -L1_s 0]';
    S3_s = [0 0 1 0 -L1_s-L2_s 0]';
    
    
    S_mat_s = [S1_s S2_s S3_s];
    
    % Body frame twists
    B1_s = [0 0 1 0 L1_s+L2_s 0]';
    B2_s = [0 0 1 0 L2_s 0]';
    B3_s = [0 0 1 0 0 0]';
    
    
    B_mat_s = [B1_s B2_s B3_s];
    
    % Adding M-type configuration of each joint to enable plotting
    M1_s = [eye(3) [0 0 0]';
        zeros(1,3), 1];
    
    M2_rot_s = [1 0 0;
              0 1 0;
              0 0 1];
    M2_trans_s = [L1_s 0 0]';
    M2_s = [M2_rot_s M2_trans_s;
        zeros(1,3), 1];
    
    M3_rot_s = [1 0 0;
              0 1 0;
              0 0 1];
    M3_trans_s = [L1_s+L2_s 0 0]';
    M3_s = [M3_rot_s M3_trans_s;
        zeros(1,3), 1];
    
    M_intermediates_s = {M1_s, M2_s, M3_s};
    
    % Joint Angles
    theta1_s = sym('theta1_s', 'real');
    theta2_s = sym('theta2_s', 'real');
    theta3_s = sym('theta3_s', 'real');
    thetas_s = [theta1_s, theta2_s, theta3_s];
end
fprintf("\n\n\n")



% a) Forward Kinematics using the space form of the exponential products
disp('-----------a) Space form of exponential products FK-----------')
disp('Show equation written out here')
[FK_solution_space_s, T_bank_space_s, T_total_bank_space_s] = FK_space(M_s, S_mat_s, thetas_s, true);
FK_solution_space_s
fprintf("\n\n\n")



% b) Forward Kinematics using the space form of the exponential products
disp('-----------b) Space FK-----------')
% [FK_solution_space, T_bank_space, T_total_bank_space] = FK_space(M, S_mat, thetas)
[FK_solution_space, T_bank_space, T_total_bank_space] = FK_space(M, S_mat, thetas, false, true, M_intermediates);
FK_solution_space
fprintf("\n\n\n")



% c) Forward Kinematics using the body form of the exponential products
disp('-----------c) Body FK-----------')
disp('Show equation written out here')
[FK_solution_body_s, T_bank_body_s, T_total_bank_body_s] = FK_body(M_s, B_mat_s, thetas_s, true);
FK_solution_body_s
[FK_solution_body, T_bank_body, T_total_bank_body] = FK_body(M, B_mat, thetas, false, true, M_intermediates);
FK_solution_body
fprintf("\n")

% Check if symbolic and real FK solutions are equivalent
disp('Showing equivalency of both symbolic and real final FK solutions for space vs. body framing:')
FK_syms_diff = simplify(FK_solution_body_s - FK_solution_space_s)
FK_real_diff = FK_solution_body - FK_solution_space
fprintf("\n\n\n")



% d) Space and body symbolic form of Jacobian
disp('-----------d) Jacobians-----------')
disp('Show equations written out here')
J_space_s = SpaceJacobian(S_mat_s, thetas_s, true)
J_body_s = BodyJacobian(B_mat_s, thetas_s, true)
% fprintf("\n")
% 
% disp('Showing equivalency of symbolic Jacobian solutions for space vs. body framing:')
% J_diff_s = simplify(J_space_s - Ad(FK_solution_space_s, true) * J_body_s)
fprintf("\n\n\n")



% e) Space and body form of Jacobian from functions
disp('-----------e) Space and Body Jacobian Functions-----------')
J_space = SpaceJacobian(S_mat, thetas)
J_body = BodyJacobian(B_mat, thetas)
fprintf("\n")

% Check to make sure these are equivalent
disp('Showing equivalency of real Jacobian solutions for space vs. body framing:')
J_diff = J_space - Ad(FK_solution_space) * J_body
fprintf("\n\n\n")



% f) Singularity configurations
disp('-----------f) Singularity conditions-----------')
disp('Solve determinant symbolically')
% solve(rank(J_space_s) < 6, thetas_s)
% svd(J_space_s) % singular value decomposition, diagonal entries of
% resultant are the singular values of the input, number of non-singular
% values is equal to rank -> get system of equations and solve such that
% fewer than 6 of the 7 produce nonsingular results to find singularity
% configurations -> this takes a long time to run
% eqns = svd(J_space)
% eqns = svd(J_space_s)

% simplify(J_space_s*J_space_s')
% det(J_space_s * J_space_s')
% solve(det(J_space_s * J_space_s') == 0, thetas_s)

J_simp = simplify(J_space_s)
% !!!!!!!!!!!!!!!!!!!!!! THESE TWO LINES TAKE FOREVER TO RUN, STILL NEED TO
% SEE IF  THEY WORK AS INTENDED !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
% J_simp_sqrd = simplify(J_simp * J_simp')
% simplify(det(J_simp_sqrd))

fprintf("\n\n\n")



% g) Manipulability ellipsoids, isotropy, condition number, and ellipsoid
% volume
disp('-----------g) More Jacobian stuffs-----------')
disp('a)')
[FK_solution_space, T_bank_space, T_total_bank_space] = FK_space(M, S_mat, thetas, false, true, M_intermediates);
ellipsoid_plot_angular(J_space, FK_solution_space(1:3,4), false)
ellipsoid_plot_linear(J_space, FK_solution_space(1:3,4), false)

disp('b)')
isotropy_angular_space = J_isotropy(J_space, "angular")
isotropy_linear_space = J_isotropy(J_space, "linear")

condition_angular_space = J_condition(J_space, "angular")
condition_linear_space = J_condition(J_space, "linear")

ellipsoid_volume_angular_space = J_ellipsoid_volume(J_space, "angular")
ellipsoid_volume_linear_space = J_ellipsoid_volume(J_space, "linear")
fprintf("\n\n\n")



% h) Inverse kinematics
disp('-----------h) Inverse kinematics-----------')
% target_orientation = FK_solution_space(1:3, 1:3);
% target_position = FK_solution_space(1:3, 4);



% target_orientation = eye(3);
% target_position = FK_solution_space(1:3, 4) + [0.1 0 0.1]';
% T_sd = [target_orientation target_position;
%     zeros(1,3), 1];
% [theta_bank] = J_inverse_kinematics(T_sd, thetas, M, S_mat)



% T_sd = [-0.5 -0.866 0 0.366;
%         0.866 -0.5 0 1.366;
%         0 0 1 0;
%         0 0 0 1];
% theta0 = [0, pi/6, 0]; 
% [theta_bank] = J_inverse_kinematics(T_sd, theta0, M, S_mat, 0.001, 1e-04);



fprintf("\n\n\n")


% J transpose kinematics
% target_orientation = eye(3);
% target_position = FK_solution_space(1:3, 4) + [0.1 0 0.1]';
% T_sd = [target_orientation target_position;
%         zeros(1,3), 1];

T_sd = [-0.5 -0.866 0 0.366;
        0.866 -0.5 0 1.366;
        0 0 1 0;
        0 0 0 1];
theta0 = [0, pi/6, 0]; 

% target_orientation = eye(3) * FK_solution_space(1:3, 1:3);
% target_position = FK_solution_space(1:3, 4) + [0.5 0.1 0]';
% T_sd = [target_orientation target_position;
%         zeros(1,3), 1];

[thetas] = J_transpose_kinematics(T_sd, theta0, M, S_mat, 0.001, 1e-04)
