function [thetas, i] = IK_constrained(M, S_mat, p_tip, thetas, joint_limits, p_goal, tol, iterations)
%Accepts a goal position, tool tip position expressed in end-effector
%frame, list of joint angles, M matrix of robot, matrix of screw axis
%column vectors corresponding to the robot, spherical radius tolerance from
%goal, and number of iterations and returns the set of joint angles that
%satisfy the constraints and the iteration of convergence.
%   Detailed explanation goes here
    if nargin < 8
        iterations = 100;
    end
    p_t = ones(3, 1);
    coords = [];
    i = 1;
    while norm(p_t - p_goal) > tol
        J = SpaceJacobian(S_mat, thetas);
        [F_space, ~] = FK_space(M, S_mat, thetas);

%         p_tip = [0 0 0.1]';
        t4 = F_space * [p_tip; 1];
        t = t4(1:3);

        A = -Axis2SkewSymmetricMatrix(t) * J(1:3, :) + J(4:6, :);
        tol_vec = [0.0015 0.0015 0.0015]';
        b = p_goal - t + sqrt(tol_vec);

        lb = joint_limits(:,1) - thetas';
        ub = joint_limits(:,2) - thetas';

%         C = eye(7);
%         d = zeros(7, 1);
        C = -Axis2SkewSymmetricMatrix(t) * J(1:3, :) + J(4:6, :);
        d = p_goal - t;

        x = lsqlin(C, d, A, b, [], [], lb, ub);
        thetas = thetas + x';

        [F_space2, ~] = FK_space(M, S_mat, thetas + x');
        new_p_tip = F_space2 * [p_tip; 1];
        p_t = new_p_tip(1:3);

        coords = [coords p_t];
        if i > iterations
            break
        end
        i = i + 1;
    end
end