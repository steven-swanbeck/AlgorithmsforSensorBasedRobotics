function [roll, pitch, yaw] = RotationMatrix2RPYAngles(R, side)
%converts an orthonormal rotation matrix into RPY equivalent rotation angle components
%   Returns a vector of Euler angles [phi, theta, psi] where phi is the initial rotation
%   about the body fixed Z axis, theta is the secondary rotation about the body fixed Y
%   axis, and psi is the tertiary rotation about the body fixed X axis when
%   provided a 3x3 orthonormal rotation matrix.
    if nargin < 2
            side = 'right';
    end
    if strcmp(side, 'right')
        roll = atan2(R(2, 1), R(1, 1));
        pitch = atan2(-1 * R(3, 1), sqrt(R(3, 2)^2 + R(3, 3)^2));
        yaw = atan2(R(3, 2), R(3, 3));
    elseif strcmp(side, 'left')
        roll = atan2(-1 * R(2, 1), -1 * R(1, 1));
        pitch = atan2(-1 * R(3, 1), -1 * sqrt(R(3, 2)^2 + R(3, 3)^2));
        yaw = atan2(-1 * R(3, 2), -1 * R(3, 3));
    else
        disp("[UNKNOWN SIDE ARGUMENT]  use either 'right' for quadrants 4 and 1 or 'left' for quadrants 2 and 3")
    end
    if abs(R(3,1)) == 1
        disp('[SINGULARITY WARNING] theta is pi/2, resulting in singularity')
        roll = NaN;
        pitch = NaN;
        yaw = NaN;
    end
end