function plotFrame_T(T, label)
    origin = T(1:3, 4)';
    % x-axis
    line('XData', [origin(1) origin(1) + T(1,1)], 'YData', [origin(2) origin(2) + T(2,1)],...
        'ZData', [origin(3) origin(3) + T(3,1)], 'Color','r','LineWidth',3);
    % y-axis
    line('XData', [origin(1) origin(1) + T(1,2)], 'YData', [origin(2) origin(2) + T(2,2)],...
        'ZData', [origin(3) origin(3) + T(3,2)], 'Color','g','LineWidth',3);
    % z-axis
    line('XData', [origin(1) origin(1) + T(1,3)], 'YData', [origin(2) origin(2) + T(2,3)],...
        'ZData', [origin(3) origin(3) + T(3,3)], 'Color','b','LineWidth',3);
    % frame label
    text(origin(1) - 0.2 * T(1,1), origin(2) - 0.2 * T(2,2), origin(3) - 0.2 * T(3,3), horzcat('$\lbrace ', label, ' \rbrace$'));
    % axes labels
    text(origin(1) + T(1,1), origin(2) + T(2,1), origin(3) + T(3,1), '$x$');
    text(origin(1) + T(1,2), origin(2) + T(2,2), origin(3) + T(3,2), '$y$');
    text(origin(1) + T(1,3), origin(2) + T(2,3), origin(3) + T(3,3), '$z$');
end