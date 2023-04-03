function [eigenvalues, V] = ellipsoid_plot_angular(J, origin, delta, show_ellipsoids, show_axes, should_create_plot)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    if nargin < 3
        delta = 1;
    end
    if nargin < 4
        show_ellipsoids = true;
    end
    if nargin < 5
        show_axes = true;
    end
    if nargin < 6
        should_create_plot = false;
    end

    J_w = J(1:3, :);
    A = J_w * J_w';
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

    if show_ellipsoids == true
        [x, y, z] = ellipsoid(origin(1), origin(2), origin(3), delta * sqrt(eigenvalues(1)), delta * sqrt(eigenvalues(2)), delta * sqrt(eigenvalues(3)));
        hMesh = surf(x, y, z, 'FaceAlpha', 0.05, 'EdgeAlpha', 0.05, 'FaceColor','magenta','EdgeColor','k');
        
        % checking if eigenvector matrix is I because axis angle rotation representation is undefined in that case 
        if not(isequal(V, eye(3)))
            [w, theta] = RotationMatrix2AxisAngle(V); % NEED TO REVIEW THIS, SEE IF VALID CONVERSION (ie. IF WORKING AS INTENDED)
    %         rotate(hMesh, w + origin, theta * 180 / pi, origin);
            rotate(hMesh, w, theta * 180 / pi, origin);
        end
    end

    if show_axes == true
        % Plotting ellipsoid axes and adding scaling label
        line('XData', [origin(1) origin(1) + delta * sqrt(eigenvalues(1))* V(1,1)], 'YData', [origin(2) origin(2) + delta *sqrt(eigenvalues(1))* V(2,1)],...
            'ZData', [origin(3) origin(3) + delta *sqrt(eigenvalues(1))* V(3,1)], 'Color','magenta','LineWidth',3);
        line('XData', [origin(1) origin(1) - delta * sqrt(eigenvalues(1))* V(1,1)], 'YData', [origin(2) origin(2) - delta *sqrt(eigenvalues(1))* V(2,1)],...
            'ZData', [origin(3) origin(3) - delta *sqrt(eigenvalues(1))* V(3,1)], 'Color','magenta','LineWidth',3);
        
        line('XData', [origin(1) origin(1) + delta *sqrt(eigenvalues(2))* V(1,2)], 'YData', [origin(2) origin(2) + delta *sqrt(eigenvalues(2))* V(2,2)],...
            'ZData', [origin(3) origin(3) + delta *sqrt(eigenvalues(2))* V(3,2)], 'Color','magenta','LineWidth',3);
        line('XData', [origin(1) origin(1) - delta *sqrt(eigenvalues(2))* V(1,2)], 'YData', [origin(2) origin(2) - delta *sqrt(eigenvalues(2))* V(2,2)],...
            'ZData', [origin(3) origin(3) - delta *sqrt(eigenvalues(2))* V(3,2)], 'Color','magenta','LineWidth',3);
       
        line('XData', [origin(1) origin(1) + delta *sqrt(eigenvalues(3))* V(1,3)], 'YData', [origin(2) origin(2) + delta *sqrt(eigenvalues(3))* V(2,3)],...
            'ZData', [origin(3) origin(3) + delta *sqrt(eigenvalues(3))* V(3,3)], 'Color','magenta','LineWidth',3);
        line('XData', [origin(1) origin(1) - delta *sqrt(eigenvalues(3))* V(1,3)], 'YData', [origin(2) origin(2) - delta *sqrt(eigenvalues(3))* V(2,3)],...
            'ZData', [origin(3) origin(3) - delta *sqrt(eigenvalues(3))* V(3,3)], 'Color','magenta','LineWidth',3);
    end
   
    text(origin(1) + 0.5*delta, 0, origin(3) + 0.5*delta, strcat('$\delta_{ang}$ = ', string(delta)), 'Color','magenta');

end