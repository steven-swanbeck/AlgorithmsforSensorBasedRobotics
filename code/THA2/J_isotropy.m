function [mu] = J_isotropy(J, type)
%Calculates the isotropy of a Jacobian. 'type' is either "angular" for the
%isotropy of the angular velcoity components or "linear" for the isotropy
%of the linear velcoity components.
    J_w = J(1:3, :);
    J_v = J(4:6, :);
    if type == "angular"
        A = J_w * J_w';
    elseif type == "linear"
        A = J_v * J_v';
    end
    [V, D] = eig(A);
    eigenvalues = diag(D);

    mu = sqrt(max(eigenvalues) / min(eigenvalues));
end