function [S, theta] = T2S(T, is_symbolic)
%converts 4x4 transformation matrix into equivalent screw axis S = [w;v]
%and theta representation
    if nargin < 2
        is_symbolic = false;
    end

    if is_symbolic == true
        S = sym(zeros(6,1));
    else
        S = zeros(6,1);
    end

    R = T(1:3, 1:3);
    p = T(1:3, 4);
    if R == eye(3)
        w = zeros(3,1);
        v = p / norm(p);
    else
        [w, theta] = RotationMatrix2AxisAngle(R);
        w_hat = Axis2SkewSymmetricMatrix(w);
        v = (eye(3)/theta-w_hat/2+((1/theta)-cot(theta/2)/2)*w_hat^2)*[T(1,4);T(2,4);T(3,4)];

        [w, theta] = RotationMatrix2AxisAngle(R);
        w_hat = Axis2SkewSymmetricMatrix(w);
        G_inv = (eye(3) / theta) - (0.5 * w_hat) + (1 / theta - 0.5 * cot(theta / 2)) * w_hat * w_hat;
        v = G_inv * p;
    end
    S = [w; v];
end