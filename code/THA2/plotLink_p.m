function plotLink_p(p0, p1, alpha)
%Plots a link between two points. Useful for visualizing manipualtor
%links. alpha is link transparency and defaults to 0.5.
    if nargin < 3
        alpha = 0.5;
    end
    line('XData', [p0(1) p1(1)], 'YData', [p0(2) p1(2)],...
        'ZData', [p0(3) p1(3)], 'Color',[0.4, 0.4, 0.4, alpha],'LineWidth',10);
end