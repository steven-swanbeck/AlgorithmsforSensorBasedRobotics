function ellipsoid_plot_linear_axes(J, C, delta, should_create_plot)
%

    % Default argument sets delta if not provided
    if nargin < 3
        delta = 1;
    end
    if nargin < 4
        should_create_plot = false;
    end
    
    J_v = J(4:6, :);
    A = J_v * J_v';
    [V, D] = eig(A);
    eigenvalues = diag(D);

    if cross(V(:, 1), V(:, 2)) == -1 * V(:, 3)
        V(:,3) = -1 * V(:, 3);
    end

    if should_create_plot == true
        figure
        view(3)
        box on
        hold on
        grid on
        axis equal;
        xlabel('x'), ylabel('y'), zlabel('z');
    end
    %Change from here
    origin = C;


    % x-axisdelta
    line('XData', [origin(1) origin(1) + delta * sqrt(eigenvalues(1))* V(1,1)], 'YData', [origin(2) origin(2) + delta *sqrt(eigenvalues(1))* V(2,1)],...
        'ZData', [origin(3) origin(3) + delta *sqrt(eigenvalues(1))* V(3,1)], 'Color','cyan','LineWidth',3);
    line('XData', [origin(1) origin(1) - delta * sqrt(eigenvalues(1))* V(1,1)], 'YData', [origin(2) origin(2) - delta *sqrt(eigenvalues(1))* V(2,1)],...
        'ZData', [origin(3) origin(3) - delta *sqrt(eigenvalues(1))* V(3,1)], 'Color','cyan','LineWidth',3);
    
    % y-axis
    line('XData', [origin(1) origin(1) + delta *sqrt(eigenvalues(2))* V(1,2)], 'YData', [origin(2) origin(2) + delta *sqrt(eigenvalues(2))* V(2,2)],...
        'ZData', [origin(3) origin(3) + delta *sqrt(eigenvalues(2))* V(3,2)], 'Color','cyan','LineWidth',3);
    line('XData', [origin(1) origin(1) - delta *sqrt(eigenvalues(2))* V(1,2)], 'YData', [origin(2) origin(2) - delta *sqrt(eigenvalues(2))* V(2,2)],...
        'ZData', [origin(3) origin(3) - delta *sqrt(eigenvalues(2))* V(3,2)], 'Color','cyan','LineWidth',3);
   
    % z-axis
    line('XData', [origin(1) origin(1) + delta *sqrt(eigenvalues(3))* V(1,3)], 'YData', [origin(2) origin(2) + delta *sqrt(eigenvalues(3))* V(2,3)],...
        'ZData', [origin(3) origin(3) + delta *sqrt(eigenvalues(3))* V(3,3)], 'Color','cyan','LineWidth',3);
    line('XData', [origin(1) origin(1) - delta *sqrt(eigenvalues(3))* V(1,3)], 'YData', [origin(2) origin(2) - delta *sqrt(eigenvalues(3))* V(2,3)],...
        'ZData', [origin(3) origin(3) - delta *sqrt(eigenvalues(3))* V(3,3)], 'Color','cyan','LineWidth',3);
   
    text(origin(1) + 0.5*delta, 0, origin(3) + 0.5*delta, strcat('$\delta_{lin}$ = ', string(delta)), 'Color','cyan');

end