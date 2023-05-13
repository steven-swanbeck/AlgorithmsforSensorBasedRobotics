function [R] = AxisAngle2RotationMatrix(w, theta)
%converts an angle theta and axis w into an equivalent orthonormal rotation matrix
%   receives a 3x1 unit column vector w representing an axis of rotation
%   and an angle theta in radians and returns an equivalent 3x3 orthonormal rotation matrix
    w_hat = Axis2SkewSymmetricMatrix(w);
    R = eye(3) + w_hat * sin(theta) + w_hat * w_hat * (1 - cos(theta));
end