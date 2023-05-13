function [M] = M_set(quat_set1, quat_set2)
%Accepts two matrices of arbitrary length with quaternions as rows and produces the full M matrix used
%for hand-eye calibration problems.
    if size(quat_set1, 1) ~= size(quat_set2)
        disp('[WARNING] Different numbers of quaternion measurements provided.')
        num_quats = min(size(quat_set1, 1), size(quat_set2));
    else
        num_quats = size(quat_set1, 1);
    end

%     M = zeros(num_quats * 4, 4);
    M = [];
    for i=1:num_quats
        [M_i] = M_quat(quat_set1(i, :)', quat_set2(i, :)');
        M = [M; M_i];
    end
end