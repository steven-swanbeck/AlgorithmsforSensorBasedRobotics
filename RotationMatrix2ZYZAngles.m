function [phi, theta, psi] = RotationMatrix2ZYZAngles(R, side)
%converts an orthonormal 3x3 rotation matrix into ZYZ equivalent rotation angle components
%   Returns a vector of Euler angles [phi, theta, psi] where phi is the initial rotation
%   about the body Z axis, theta is the secondary rotation about the body Y
%   axis, and psi is the tertiary rotation about the body Z axis when
%   provided a 3x3 orthonormal rotation matrix.
    if nargin < 2
        side = 'upper';
    end
    if R == eye(3)
        disp('[UNDEFINED WARNING] infinitely many representations for identity rotation.')
        phi = NaN;
        theta = NaN;
        psi = NaN;
    else
        if strcmp(side, 'upper')
            phi = atan2(R(2, 3), R(1, 3));
            theta = atan2(sqrt(R(1, 3)^2 + R(2, 3)^2), R(3, 3));
            psi = atan2(R(3, 2), -1 * R(3, 1));
        elseif strcmp(side, 'lower')
            phi = atan2(-1 * R(2, 3), -1 * R(1, 3));
            theta = atan2(-1 * sqrt(R(1, 3)^2 + R(2, 3)^2), R(3, 3));
            psi = atan2(-1 * R(3, 2), R(3, 1));
        else
            disp("[UNKNOWN SIDE ARGUMENT] use either 'upper' for quadrants 1 and 2 or 'lower' for quadrants 3 and 4")
        end
    end
    if abs(R(3,3)) == 1
        disp('[SINGULARITY WARNING] theta is 0 or pi, resulting in singularity')
        phi = NaN;
        theta = NaN;
        psi = NaN;
    end
end