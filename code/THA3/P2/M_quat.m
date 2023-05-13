function [M_ab] = M_quat(q1, q2)
%Accepts two unit quaternions and produces the M submatrix associated with
%them necessary for hand-eye calibration.
    sa = q1(1);
    va = q1(2:4);
    sb = q2(1);
    vb = q2(2:4);

    ds = sa - sb;
    dv = (va - vb);

    quad1 = ds;
    quad2 = -dv';
    quad3 = dv;
    quad4 = ds * eye(3) + Axis2SkewSymmetricMatrix(va + vb);

    M_ab = [quad1 quad2; quad3 quad4];
end