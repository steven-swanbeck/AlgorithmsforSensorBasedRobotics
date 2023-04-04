function [FK_solution, T_bank, T_total_bank] = FK_body(M, B_mat, thetas, is_symbolic, should_plot, M_intermediates, should_create_plot)
%Accepts a base configuration transformation matrix M, a matrix of column vectors representing the screw axes, and a vector of theta values corresponding to the rotation about each screw axis 
%   Should_plot will determine whether a plot is created for the frames or
%   not. If it and all following arguments are not set, this will default
%   to false. If set true, the user must also provide configurations
%   analogous to M corresponding to each joint location as a cell array of transformation matrices. If
%   should_create_plot is not provided, it will default true, and a new,
%   self-contained figure is created. To override this behavior, provide
%   false and the output will be plotted on the current figure in focus.
    
    % Pseudo-default arguments
    if nargin < 4
        is_symbolic = false;
    end
    if nargin < 5
        should_plot = false;
    elseif nargin < 7
        should_create_plot = true;
    end

    B_bank = cell(1, size(B_mat, 2));
    for i = 1:size(B_mat, 2)
        B_bank{i} = B_mat(:,i);
    end
    
    % To allow plotting base configuration frames
    T_bank_base = cell(1, size(B_mat, 2));
    for i = 1:size(B_mat, 2)
        T_bank_base{i} = S2T(B_bank{i}, 0, is_symbolic);
    end

    % Transformations using the input angles
    T_bank = cell(1, size(B_mat, 2));
    for i = 1:size(B_mat, 2)
        T_bank{i} = S2T(B_bank{i}, thetas(i), is_symbolic);
    end

    % Performing body forward kinematics
    T_total = eye(4);
    T_total_bank = cell(1, size(B_mat, 2));
    for i = 1:size(T_bank,2)
        T_total = T_total * T_bank{i};
        T_total_bank{i} = T_total;
    end
    
    % Final solution calculation
    FK_solution = M * T_total;

    % Plot of new frames 
    if should_plot == true   
        if should_create_plot == true
            figure
            view(3)
            box on
            hold on
%             grid on
            axis equal;
            xlabel('x'), ylabel('y'), zlabel('z');
            view([30, 15]);

        end
        plotFrame_T(eye(4), '$\lbrace s \rbrace$', 0.1)
        stored_origin = [0, 0, 0];
        for i = 1:length(thetas)
            T_total = M_intermediates{i} * T_total_bank{i};
            plotFrame_T(T_total, strcat('$\lbrace b_', string(i) , '\rbrace$'), 0.1);
            plotLink_p(stored_origin, T_total(1:3, 4));
            stored_origin = T_total(1:3, 4);
        end
        Tf = M * T_total_bank{end};
        plotFrame_T(Tf, '$\lbrace f \rbrace$', 0.1)
        plotLink_p(stored_origin, Tf(1:3, 4));

        plotScrewAxis_T(Tf, 0.25);
    end
end