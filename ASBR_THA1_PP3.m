clc; clear; format compact; clf; close all;

set(0,'defaultTextInterpreter','latex');
set(0, 'defaultAxesTickLabelInterpreter','latex'); 
set(0, 'defaultLegendInterpreter','latex');
set(0,'defaultAxesFontSize',18);
set(0, 'DefaultLineLineWidth', 2);
set(groot, 'defaultFigureUnits', 'pixels', 'defaultFigurePosition', [440   278   560   420]);

% givens
q = [0, 2, 0]';
s_hat = [0, 0, 1]';
h = 2;
theta = pi;
T = [1 0 0 2;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];

% 1) convert qsh to S = [w; v]
S = qsh2S(q, s_hat, h)
w= S(1:3);
v = S(4:6);

% 2) matrix exponential to get T (repeat for each theta)
theta = linspace(0, theta, 5);
T_stor_s = cell(1, length(theta));
T_stor_b = cell(1, length(theta));
for i = 1:length(theta)
    T_stor_s{i} = S2T(S, theta(i));
    T_stor_b{i} = T_stor_s{i} * T;
end

% 3) compute the final transform
T1 = T_stor_s{end} * T

% 4) find transform to get back to the origin
To = inv(T1)

% 5) use matrix logarithm to convert To to screw axis
So = T2S(To)

s_hato = So(1:3) / vecnorm(So(1:3))

% Finding Point q to fully define axis
v = So(4:6);
h = So(1:3)' * So(4:6)
s_hat = s_hato;
q = Axis2SkewSymmetricMatrix(s_hat) * (v - h * s_hat)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot results
figure
plot3(0, 0, 0, '.k', 1, 0, 0,'.k', 0, 1, 0,'.k', 0, 0, 1,'.k', markersize=10)
view(3)
box on
hold on
grid on
xlabel('x'), ylabel('y'), zlabel('z');

% origin
origin = [0 0 0];
% length of frame vectors
delta = 1;
% x-axis
line('XData', [origin(1) origin(1) + delta], 'YData', [origin(2) origin(2)],...
    'ZData', [origin(3) origin(3)], 'Color','r','LineWidth',3);
% y-axis
line('XData', [origin(1) origin(1)], 'YData', [origin(2) origin(2) + delta],...
    'ZData', [origin(3) origin(3)], 'Color','g','LineWidth',3);
% z-axis
line('XData', [origin(1) origin(1)], 'YData', [origin(2) origin(2)],...
    'ZData', [origin(3) origin(3) + delta], 'Color','b','LineWidth',3);
text(origin(1) - 0.2, origin(2) - 0.2, origin(3) - 0.2, '$\lbrace s \rbrace$');
text(origin(1) + delta, origin(2), origin(3), '$x$');
text(origin(1), origin(2) + delta, origin(3), '$y$');
text(origin(1), origin(2), origin(3) + delta, '$z$');

for i = 1:length(theta)
    origin = T_stor_b{i}(1:3, 4)';
    % x-axis
    line('XData', [origin(1) origin(1) + T_stor_b{i}(1,1)], 'YData', [origin(2) origin(2) + T_stor_b{i}(2,1)],...
        'ZData', [origin(3) origin(3) + T_stor_b{i}(3,1)], 'Color','r','LineWidth',3);
    % y-axis
    line('XData', [origin(1) origin(1) + T_stor_b{i}(1,2)], 'YData', [origin(2) origin(2) + T_stor_b{i}(2,2)],...
        'ZData', [origin(3) origin(3) + T_stor_b{i}(3,2)], 'Color','g','LineWidth',3);
    % z-axis
    line('XData', [origin(1) origin(1) + T_stor_b{i}(1,3)], 'YData', [origin(2) origin(2) + T_stor_b{i}(2,3)],...
        'ZData', [origin(3) origin(3) + T_stor_b{i}(3,3)], 'Color','b','LineWidth',3);
    % frame label
    text(origin(1) - 0.2 * T_stor_b{i}(1,1), origin(2) - 0.2 * T_stor_b{i}(2,2), origin(3) - 0.2 * T_stor_b{i}(3,3), strcat('$\lbrace b \rbrace$', ', $\theta = $', string(theta(i))));
    % axes labels
    text(origin(1) + T_stor_b{i}(1,1), origin(2) + T_stor_b{i}(2,1), origin(3) + T_stor_b{i}(3,1), '$x$');
    text(origin(1) + T_stor_b{i}(1,2), origin(2) + T_stor_b{i}(2,2), origin(3) + T_stor_b{i}(3,2), '$y$');
    text(origin(1) + T_stor_b{i}(1,3), origin(2) + T_stor_b{i}(2,3), origin(3) + T_stor_b{i}(3,3), '$z$');
end

% plotting screw axis
origin = q';
% length of frame vectors
delta = 1;
C = 1;
% x-axis
line('XData', [origin(1) origin(1) + C * abs(s_hat(1))], 'YData', [origin(2) origin(2)],...
    'ZData', [origin(3) origin(3)], 'Color','k','LineWidth',3);
line('XData', [origin(1) origin(1) - C * abs(s_hat(1))], 'YData', [origin(2) origin(2)],...
    'ZData', [origin(3) origin(3)], 'Color','k','LineWidth',3);
% y-axis
line('XData', [origin(1) origin(1)], 'YData', [origin(2) origin(2) + C * abs(s_hat(2))],...
    'ZData', [origin(3) origin(3)], 'Color','k','LineWidth',3);
line('XData', [origin(1) origin(1)], 'YData', [origin(2) origin(2) - C * abs(s_hat(2))],...
    'ZData', [origin(3) origin(3)], 'Color','k','LineWidth',3);
% z-axis
line('XData', [origin(1) origin(1)], 'YData', [origin(2) origin(2)],...
    'ZData', [origin(3) origin(3) + C * 8 * abs(s_hat(3))], 'Color','k','LineWidth',3);
line('XData', [origin(1) origin(1)], 'YData', [origin(2) origin(2)],...
    'ZData', [origin(3) origin(3) - C * abs(s_hat(3))], 'Color','k','LineWidth',3);
text(origin(1) - 0.2, origin(2) - 0.2, origin(3) - 0.2, '$\hat{s}_{sb}$');