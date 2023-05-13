function [w_hat] = Axis2SkewSymmetricMatrix(w)
%converts a unit axis into an equivalent 3x3 screw-symmetrix matrix
    w_hat = zeros(3);
    w_hat(1, 2) = -1 * w(3);
    w_hat(1, 3) = w(2);
    w_hat(2, 1) = w(3);
    w_hat(2, 3) = -1 * w(1);
    w_hat(3, 1) = -1 * w(2);
    w_hat(3, 2) = w(1);
end