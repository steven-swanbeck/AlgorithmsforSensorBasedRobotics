function [S] = T2S(T)
%converts 4x4 transformation matrix into equivalent screw axis S = [w;v]
%and theta represnetation
    R = T(1:3, 1:3);
    if R == eye(3)
        S = [0;0;0;T(1,4);T(2,4);T(3,4)];
    else
        [w, theta] = RotationMatrix2AxisAngle(R);
        w_hat = Axis2SkewSymmetricMatrix(w);
        v = (eye(3)/theta-w_hat/2+((1/theta)-cot(theta/2)/2)*w_hat^2)*[T(1,4);T(2,4);T(3,4)];
        S = [w;v];
    end
end