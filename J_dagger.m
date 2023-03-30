function [J_dagger] = J_dagger(J)
%Calculates the Moore-Penrose pseudoinverse of a Jacobian
% If pseudoinverse is unnecessary, will return proper inverse of input
% Jacobian
    m = size(J, 1);
    n = size(J, 2);
    
%     redundant or 'fat' case -> right inverse
    if n > m
        J_dagger = J' / (J * J');
%     'tall' case -> left inverse
    elseif n < m
        J_dagger = (J' * J) \ J';
%     if J is square -> true inverse
    elseif n == m
        J_dagger = inv(J);
    end
end