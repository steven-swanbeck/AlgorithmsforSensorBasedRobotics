function [outputArg1,outputArg2] = plotBody(a_set, b_set)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
    figure
    view(3)
    xlabel('x'), ylabel('y'), zlabel('z')
    box on
    axis equal
    hold on
    plot3(a_set(:,1), a_set(:,2), a_set(:,3), 'r');
    plot3(b_set(:,1), b_set(:,2), b_set(:,3), 'b');
end