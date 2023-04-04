function [R] = Quaternion2RotationMatrix(q)
%converts 4x1 column vector representing unit quaternion components [q0 q1 q2
%q3]' into equivalent 3x3 rotation matrix
    R = zeros(3);
    R(1,1) = q(1)^2 + q(2)^2 - q(3)^2 - q(4)^2;
    R(1,2) = 2 * (q(2)*q(3) - q(1)*q(4));
    R(1,3) = 2 * (q(1)*q(3) + q(2)*q(4));
    R(2,1) = 2 * (q(1)*q(4) + q(2)*q(3));
    R(2,2) = q(1)^2 - q(2)^2 + q(3)^2 - q(4)^2;
    R(2,3) = 2 * (q(3)*q(4) - q(1)*q(2));
    R(3,1) = 2 * (q(2)*q(4) - q(1)*q(3));
    R(3,2) = 2 * (q(1)*q(2) + q(3)*q(4));
    R(3,3) = q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2;
end