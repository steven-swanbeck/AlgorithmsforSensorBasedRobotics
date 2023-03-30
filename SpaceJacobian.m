function [Js] = SpaceJacobian(S_mat, thetas, is_symbolic)
%Converts matrix of column vectors corresponding to space frame screw axes
%and a vector of theta values corresponding to each joint and returns the
%space Jacobian
    if nargin < 3
        is_symbolic = false;
    end

    S_bank = cell(1, size(S_mat, 2));
    for i = 1:size(S_mat, 2)
        S_bank{i} = S_mat(:,i);
    end
    
%     symbolic variation
    if is_symbolic == true
        Js = sym(zeros(6, size(S_mat,2)));
    else
        Js = zeros(6, size(S_mat,2));
    end
    
    Js(:,1) = S_bank{1};
    for i = 2:size(S_mat, 2)
        Ti = eye(4);
        for j = 1:i - 1
            Ti = Ti * S2T(S_bank{j}, thetas(j), is_symbolic);
%             fprintf('column: %i, matrix: %i\n', i, j); % debug stream
        end
        Js(:,i) = Ad(Ti, is_symbolic) * S_bank{i};
    end
end