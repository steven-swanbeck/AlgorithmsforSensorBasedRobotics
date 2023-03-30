function [ellipsoid_volume] = J_ellipsoid_volume(J, type)
%Calculates the volume of a Jacobian manipulability ellipsoid. 'type' is either "angular" for the
%angular velcoity components or "linear" for the linear velcoity components.
    J_w = J(1:3, :);
    J_v = J(4:6, :);
    if type == "angular"
        A = J_w * J_w';
    elseif type == "linear"
        A = J_v * J_v';
    end
    ellipsoid_volume = sqrt(det(A));

%     [V, D] = eig(A);
%     eigenvalues = diag(D);
%     ellipsoid_volume = sqrt(prod(eigenvalues));
end