function [w_hat] = Axis2SkewSymmetricMatrix(w, is_symbolic)
%converts a unit axis into an equivalent 3x3 screw-symmetrix matrix
    if nargin < 2
        is_symbolic = false;
    end

    if is_symbolic == true
        w_hat = sym(zeros(3));
    else
        w_hat = zeros(3);
    end
    w_hat(1, 2) = -1 * w(3);
    w_hat(1, 3) = w(2);
    w_hat(2, 1) = w(3);
    w_hat(2, 3) = -1 * w(1);
    w_hat(3, 1) = -1 * w(2);
    w_hat(3, 2) = w(1);
end