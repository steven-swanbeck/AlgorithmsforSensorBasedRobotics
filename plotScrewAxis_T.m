function plotScrewAxis_T(T, C)
%Plots an equivalent screw axis in an existing figure given a transformation matrix
    % length of frame vectors
    if nargin < 2
        C = 1;
    end
    
    S = T2S(T);
    w = S(1:3);
    v = S(4:6);
    
    h = w' * v;
    s_hat = w;
    q = Axis2SkewSymmetricMatrix(s_hat) * (v - h * s_hat);

    plotScrewAxis_qsh(q, s_hat, h, C);
end