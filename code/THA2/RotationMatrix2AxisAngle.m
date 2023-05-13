function [w, theta] = RotationMatrix2AxisAngle(R)
% Converts 3x3 rotation matrix into equivalent axis representation
%   Accepts 3x3 orthonormal matrix and returns a theta value in the range
%   [0, pi] radians and a column vector w representing the unit rotation
%   axis
    if R == eye
        disp('[UNDEFINED WARNING] infinitely many representations for identity rotation')
        theta = 0;
        w = [NaN NaN NaN]';
    elseif trace(R) == -1
        theta = pi;
        if R(3, 3) ~= -1
            w = 1 / (sqrt(2 * (1 + R(3, 3)))) * [R(1, 3); R(2, 3); 1 + R(3, 3)];
        elseif R(2, 2) ~= -1
            w = 1 / (sqrt(2 * (1 + R(2, 2)))) * [R(1, 2); 1 + R(2, 2); R(3, 2)];
        elseif R(1, 1) ~= -1
            w = 1 / (sqrt(2 * (1 + R(1, 1)))) * [1 + R(1, 1); R(2, 1); R(3, 1)];
        end
    else
        theta = acos(0.5 * (trace(R) - 1));
        w_hat = 1 / (2 * sin(theta)) * (R - R');
        w = [w_hat(3, 2) w_hat(1, 3) w_hat(2, 1)]';
    end
end