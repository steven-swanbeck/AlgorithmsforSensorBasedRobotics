function [J_dagger] = J_dagger(J)
%Calculates the Moore-Penrose pseudoinverse of a Jacobian
% If pseudoinverse is unnecessary, will return proper inverse of input
% Jacobian
    m = size(J, 1);
    n = size(J, 2);
    
%     if J is square -> true inverse
    if n == m
        J_dagger = inv(J);
%     redundant or 'fat' case -> right inverse
    elseif n > m
        J_dagger = J' / (J * J');
%     'tall' case -> left inverse
    elseif n < m
        J_dagger = (J' * J) \ J';
    end
end