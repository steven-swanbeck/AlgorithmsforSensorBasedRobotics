function [J_star] = J_star(J, k)
%Calculates the damped least-squares inverse using a damping factor k
    if nargin < 2
        k = 1;
    end
    n = size(J*J',1);
    J_star = J' / (J*J' + k^2*eye(n));
end