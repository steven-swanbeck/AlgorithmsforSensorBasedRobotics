function plotFrame_T(T, label, delta)
%Plots coordinate frame correspinding to rigid body transformation matrix T
%in an existing 3D plot
%   label is a string that will be attached to the frame and delta scales
%   the lengths of the axes. Default for delta is 1, corresponding to one
%   distance unit.

    % Default argument sets delta if not provided
    if nargin < 3
        delta = 1;
    end
    origin = T(1:3, 4)';

    % x-axisdelta
    line('XData', [origin(1) origin(1) + delta * T(1,1)], 'YData', [origin(2) origin(2) + delta * T(2,1)],...
        'ZData', [origin(3) origin(3) + delta * T(3,1)], 'Color','r','LineWidth',3);
    
    % y-axis
    line('XData', [origin(1) origin(1) + delta * T(1,2)], 'YData', [origin(2) origin(2) + delta * T(2,2)],...
        'ZData', [origin(3) origin(3) + delta * T(3,2)], 'Color','g','LineWidth',3);
   
    % z-axis
    line('XData', [origin(1) origin(1) + delta * T(1,3)], 'YData', [origin(2) origin(2) + delta * T(2,3)],...
        'ZData', [origin(3) origin(3) + delta * T(3,3)], 'Color','b','LineWidth',3);
   
    % frame label
%     text(origin(1) - delta * 0.2 * T(1,1), origin(2) - delta * 0.2 * T(2,2), origin(3) - delta * 0.2 * T(3,3), horzcat('$\lbrace ', label, ' \rbrace$'));
    text(origin(1) - delta * 0.3 * T(1,1), origin(2) - delta * 0.3 * T(2,2), origin(3) - delta * 0.3 * T(3,3), label);
    
    % axes labels
    text(origin(1) + delta * T(1,1), origin(2) + delta * T(2,1), origin(3) + delta * T(3,1), '$x$');
    text(origin(1) + delta * T(1,2), origin(2) + delta * T(2,2), origin(3) + delta * T(3,2), '$y$');
    text(origin(1) + delta * T(1,3), origin(2) + delta * T(2,3), origin(3) + delta * T(3,3), '$z$');
end