function [M, thetas, S_mat, B_mat, M_intermediates, joint_limits] = instantiate_robot(robot_name, is_symbolic)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    if nargin < 1
        robot_name = "franka";
    end
    if nargin < 2
        is_symbolic = false;
    end

    if robot_name == "franka"
        if is_symbolic == false
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
            thetas = [0 -pi/4 0 -3*pi/4 0 pi/2 pi/4]; % PANDA NORMAL CONFIG

            % Joint Limits
            joint_limits_lower = [-2.8973 -1.7628 -2.8973 -3.0718 -2.8973 -0.0175 -2.8973]';
            joint_limits_upper = [2.8973 1.7628 2.8973 -0.0698 2.8973 3.7525 2.8973]';
            joint_limits = [joint_limits_lower joint_limits_upper];

        else
            L1 = sym('L1', 'real');
            L2 = sym('L2', 'real');
            L3 = sym('L3', 'real');
            L4 = sym('L4', 'real');
            L5 = sym('L5', 'real');
            a = sym('a', 'real');
        
            theta1 = sym('theta1', 'real');
            theta2 = sym('theta2', 'real');
            theta3 = sym('theta3', 'real');
            theta4 = sym('theta4', 'real');
            theta5 = sym('theta5', 'real');
            theta6 = sym('theta6', 'real');
            theta7 = sym('theta7', 'real');
            
            % Base configuration
            M_rot_s = [1 0 0;
                    0 -1 0;
                    0 0 -1];
            M_trans_s = [L4;
                        0;
                        (L1 + L2 + L3 - L5)];
            M = [M_rot_s M_trans_s;
                zeros(1,3) 1];
            
            % Fixed frame twists
            S1_s = [0 0 1 0 0 0]';
            S2_s = [0 1 0 -L1 0 0]';
            S3_s = [0 0 1 0 0 0]';
            S4_s = [0 -1 0 (L1 + L2) 0 -a]';
            S5_s = [0 0 1 0 0 0]';
            S6_s = [0 -1 0 (L1 + L2 + L3) 0 0]';
            S7_s = [0 0 -1 0 L4 0]';
            
            S_mat = [S1_s S2_s S3_s S4_s S5_s S6_s S7_s];
            
            % Body frame twists
            B1_s = [0 0 -1 0 -L4 0]';
            B2_s = [0 -1 0 L2 + L3 - L5 0 L4]';
            B3_s = [0 0 -1 0 -L4 0]';
            B4_s = [0 1 0 L5 - L3 0 -L4 + a]';
            B5_s = [0 0 -1 0 -L4 0]';
            B6_s = [0 1 0 L5 0 -L4]';
            B7_s = [0 0 1 0 0 0]';
            
            B_mat = [B1_s B2_s B3_s B4_s B5_s B6_s B7_s];

            % Joint Angles
            thetas = [theta1, theta2, theta3, theta4, theta5, theta6, theta7];
        end
    
    elseif robot_name == "test"
        if is_symbolic == false
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
        
        elseif is_symbolic == true
            L1 = sym('L1', 'real');
            L2 = sym('L2', 'real');
            
            % Base configuration
            M_rot_s = [1 0 0;
                    0 1 0;
                    0 0 1];
            M_trans_s = [L1+L2;
                        0;
                        0];
            M = [M_rot_s M_trans_s;
                zeros(1,3) 1];
            
            % Fixed frame twists
            S1_s = [0 0 1 0 0 0]';
            S2_s = [0 0 1 0 -L1 0]';
            S3_s = [0 0 1 0 -L1-L2 0]';
            
            
            S_mat = [S1_s S2_s S3_s];
            
            % Body frame twists
            B1_s = [0 0 1 0 L1+L2 0]';
            B2_s = [0 0 1 0 L2 0]';
            B3_s = [0 0 1 0 0 0]';
            
            
            B_mat = [B1_s B2_s B3_s];
            
            % Adding M-type configuration of each joint to enable plotting
            M1_s = [eye(3) [0 0 0]';
                zeros(1,3), 1];
            
            M2_rot_s = [1 0 0;
                      0 1 0;
                      0 0 1];
            M2_trans_s = [L1 0 0]';
            M2_s = [M2_rot_s M2_trans_s;
                zeros(1,3), 1];
            
            M3_rot_s = [1 0 0;
                      0 1 0;
                      0 0 1];
            M3_trans_s = [L1+L2 0 0]';
            M3_s = [M3_rot_s M3_trans_s;
                zeros(1,3), 1];
            
            M_intermediates = {M1_s, M2_s, M3_s};
            
            % Joint Angles
            theta1 = sym('theta1', 'real');
            theta2 = sym('theta2', 'real');
            theta3 = sym('theta3', 'real');
            thetas = [theta1, theta2, theta3];
        end
    end
end