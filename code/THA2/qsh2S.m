function [S] = qsh2S(q, s_hat, h)
%Receives a point q, unit vector s_hat, and pitch h and returns a Screw
%axis of the form [w; v]. Assumes theta_dot has magnitude 1.
%   Receives point q along the screw axis as a 3x1 column vector, s_hat as a 3x1 unit column
%   vector for direction of the screw axis, and single value h for the
%   pitch of the screw axis
    w = s_hat;
    v = cross(-1 * s_hat, q) + h * s_hat;
    S = [w; v];
end