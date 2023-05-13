function plotScrewAxis_qsh(q, s_hat, h, C)
    origin = q';
    % length of frame vectors
    if nargin < 4
        C = 1;
    end
    % x-axis
    line('XData', [origin(1) origin(1) + C * s_hat(1)], 'YData', [origin(2) origin(2) + C * s_hat(2)],...
        'ZData', [origin(3) origin(3) + C * s_hat(3)], 'Color','k','LineWidth',3);
    line('XData', [origin(1) origin(1) - C * s_hat(1)], 'YData', [origin(2) origin(2) - C * s_hat(2)],...
        'ZData', [origin(3) origin(3) - C * s_hat(3)], 'Color','k','LineWidth',3);
    text(origin(1) - 0.1, origin(2) - 0.1, origin(3) + 0.1, strcat('$S$', ', h = ', string(h)));
end