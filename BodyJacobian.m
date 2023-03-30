function [Jb] = BodyJacobian(B_mat, thetas, is_symbolic)
%Converts matrix of column vectors corresponding to body frame screw axes
%and a vector of theta values corresponding to each joint and returns the
%body Jacobian
     if nargin < 3
        is_symbolic = false;
     end

    B_bank = cell(1, size(B_mat, 2));
    for i = 1:size(B_mat, 2)
        B_bank{i} = B_mat(:,i);
    end
    
    %     symbolic variation
    if is_symbolic == true
        Jb = sym(zeros(6, size(B_mat,2)));
    else
        Jb = zeros(6, size(B_mat,2));
    end

    for i = size(B_mat, 2) - 1:-1:1
        Ti = eye(4);
        for j = size(B_mat,2):-1:i + 1
            Ti = Ti * S2T(-1 * B_bank{j}, thetas(j), is_symbolic);
%             fprintf('column: %i, iteration: %i\n', i, j); % debug stream
        end
        Jb(:,i) = Ad(Ti, is_symbolic) * B_bank{i};
    end
    Jb(:,end) = B_bank{end};
end