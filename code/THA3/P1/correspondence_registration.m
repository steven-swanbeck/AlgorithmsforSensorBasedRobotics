function [T, a_bar, b_bar] = correspondence_registration(a_set, b_set)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

    % step 1: calculating centroids
    a_sum = [0, 0, 0];
    for i = 1:size(a_set, 1)
        a_sum = a_sum + a_set(i, :);
    end
    a_bar = a_sum ./ size(a_set, 1);

    b_sum = [0, 0, 0];
    for i = 1:size(b_set, 1)
        b_sum = b_sum + b_set(i, :);
    end
    b_bar = b_sum ./ size(b_set, 1);

    % step 2: calculating deviations from centroids
    a_tilda = zeros(size(a_set));
    for i = 1:size(a_set, 1)
        a_tilda(i, :) = a_set(i, :) - a_bar;
    end

    b_tilda = zeros(size(b_set));
    for i = 1:size(b_set, 1)
        b_tilda(i, :) = b_set(i, :) - b_bar;
    end

    % step 3: find R that minimizes (using quaternion approach here)
    M = [];
    for i = 1:length(a_tilda)
        M = [M; 
            0 b_tilda(i, :) - a_tilda(i, :);
            (b_tilda(i, :) - a_tilda(i, :))' Axis2SkewSymmetricMatrix(b_tilda(i, :) + a_tilda(i, :))];
    end

    [U, S, V] = svd(M);

    q = V(:, 4);
    R = Quaternion2RotationMatrix(q);

% %     Instead trying eigenvalue decomposition
%     H = zeros(3);
%     for i = 1:length(a_tilda)
%         H = H + [a_tilda(i, 1) * b_tilda(i, 1) a_tilda(i, 1) * b_tilda(i, 2) a_tilda(i, 1) * b_tilda(i, 3);
%                 a_tilda(i, 2) * b_tilda(i, 1) a_tilda(i, 2) * b_tilda(i, 2) a_tilda(i, 2) * b_tilda(i, 3);
%                 a_tilda(i, 3) * b_tilda(i, 1) a_tilda(i, 3) * b_tilda(i, 2) a_tilda(i, 3) * b_tilda(i, 3)];
%     end
%     
%     delta = [H(2,3) - H(3,2) H(3,1) - H(1,3) H(1,2) - H(2,1)]';
%     G = [trace(H) delta';
%         delta H + H' - trace(H) * eye(3)];
% 
%     [V, D] = eig(G);
% 
%     G - V * D * V';
% 
%     q_e = V(:, end);
%     norm(V(:,end));
%     R = Quaternion2RotationMatrix(q_e);


    % step 4: find p
    p = b_bar' - R * a_bar';

    % step 5: putting it together
    T = [R, p; zeros(1,3) 1];
end