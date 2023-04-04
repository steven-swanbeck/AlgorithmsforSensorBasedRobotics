function [T] = S2T(S,theta,is_symbolic)
%converts screw axis S = [w;v] and theta representation into an equivalent
%4x4 transformation matrix 
    if nargin < 3
        is_symbolic = false;
    end
    
%     Checking for symbolic returns
    if is_symbolic == true
        T = sym(zeros(4));
    else
        T = zeros(4);
    end
    w = S(1:3);
    v = S(4:6);
    w_hat = Axis2SkewSymmetricMatrix(w, is_symbolic);
    trans_vect = (eye(3)*theta+(1-cos(theta))*w_hat+(theta-sin(theta))*w_hat^2)*v;
    rot_mat = eye(3)+w_hat*sin(theta)+w_hat^2*(1-cos(theta));
    T(1:3, 1:3) = rot_mat;
    T(1:3, 4) = trans_vect;
    T(4,4) = 1;
end