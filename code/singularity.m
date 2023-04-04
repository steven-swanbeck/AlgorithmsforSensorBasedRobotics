function [is_singular, result] = singularity(J, is_symbolic)
%Determines if robot is at singular configuration
%   Takes in symoblic or numeric Jacobian and corresponding is_symbolic
%   boolean argument. Outputs a boolean and the result of the calculation.
    if nargin < 2
        is_symbolic = false;
    end

    if is_symbolic == true
        J_simp = simplify(J);
        J_simp_sqrd = simplify(J_simp * J_simp');
        result = simplify(det(J_simp_sqrd));
        is_singular = "Unknown. Need to solve for thetas that lead to singularity configuration.";
    else
        result = det(J * J');
        if result == 0
            is_singular = true;
        else
            is_singular = false;
        end
    end
end