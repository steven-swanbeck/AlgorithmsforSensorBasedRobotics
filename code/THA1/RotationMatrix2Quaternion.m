function [q] = RotationMatrix2Quaternion(R)
%receives a 3x3 orthonormal rotation matrix and returns the equivalent
%unit quaternion
%   accepts 3x3 matrix and returns a 4x1 column vector of quaternion
%   components [q0 q1 q2 q3]'
    q = zeros(4,1);
    q(1) = 1/2 * sqrt(R(1,1) + R(2,2) + R(3,3) + 1);
    q(2) = 1/2 * sign(R(3,2) - R(2,3)) * sqrt(R(1,1) - R(2,2) - R(3,3) + 1);
    q(3) = 1/2 * sign(R(1,3) - R(3,1)) * sqrt(R(2,2) - R(3,3) - R(1,1) + 1);
    q(4) = 1/2 * sign(R(2,1) - R(1,2)) * sqrt(R(3,3) - R(1,1) - R(2,2) + 1);
end