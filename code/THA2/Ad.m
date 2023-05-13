function [Ad] = Ad(T, is_symbolic)
%Receives a 4x4 transformation matrix and returns the equivalent 6x6
%adjoint matrix representation
    if nargin < 2
        is_symbolic = false;
    end

    R = T(1:3, 1:3);
    P = T(1:3, 4);

    Ad = [R, zeros(3);
        Axis2SkewSymmetricMatrix(P, is_symbolic) * R, R];
end