function [T] = S2T(S,theta)
%converts screw axis S = [w;v] and theta representation into an equivalent
%4x4 transformation matrix 
    T = zeros(4);
    w = S(1:3);
    v = S(4:6);
    w_hat = Axis2SkewSymmetricMatrix(w);
    trans_vect = (eye(3)*theta+(1-cos(theta))*w_hat+(theta-sin(theta))*w_hat^2)*v;
    rot_mat = eye(3)+w_hat*sin(theta)+w_hat^2*(1-cos(theta));
    T(1:3, 1:3) = rot_mat;
    T(1:3, 4) = trans_vect;
    T(4,4) = 1;
end