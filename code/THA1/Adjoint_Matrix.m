
function [M] = Adjoint_Matrix(T)
%Converts 4x4 Matrix T into 6x6 Adjoint matrix M
    M = zeros(6);
    R = T(1:3,1:3);
    p = T(1:3,4);
    rp = Axis2SkewSymmetricMatrix(p)*R;
    M(1:3,1:3) = R;
    M(4:6,1:3) = rp;
    M(4:6,4:6) = R;
end