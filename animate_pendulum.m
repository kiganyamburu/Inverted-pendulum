function animate_pendulum(time, theta_history, alpha_history, r, Lp)
% ANIMATE_PENDULUM - Animates the inverted pendulum motion
%
% Inputs:
%   time           - Time vector
%   theta_history  - Arm angle history (rad)
%   alpha_history  - Pendulum angle history (rad)
%   r              - Arm length (m)
%   Lp             - Pendulum length (m)

% Animation parameters
skip_frames = 20;  % Skip frames for faster animation
animation_speed = 1.0;  % Speed multiplier

% Create figure
fig = figure('Name', 'Pendulum Animation', 'Position', [100, 100, 800, 800]);
axis equal;
axis([-0.3 0.3 -0.3 0.3]);
grid on;
hold on;
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Inverted Pendulum Swing-Up Animation');

% Draw base
plot(0, 0, 'ko', 'MarkerSize', 15, 'MarkerFaceColor', 'k');

% Initialize arm and pendulum
h_arm = plot([0 0], [0 0], 'b-', 'LineWidth', 4);
h_pendulum = plot([0 0], [0 0], 'r-', 'LineWidth', 3);
h_pivot = plot(0, 0, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
h_mass = plot(0, 0, 'ro', 'MarkerSize', 20, 'MarkerFaceColor', 'r');
h_time = text(-0.25, 0.25, '', 'FontSize', 12);

% Animation loop
for i = 1:skip_frames:length(time)
    % Calculate arm endpoint position
    x_arm = r * sin(theta_history(i));
    y_arm = r * cos(theta_history(i));
    
    % Calculate pendulum endpoint position
    x_pend = x_arm + Lp * sin(alpha_history(i) + theta_history(i));
    y_pend = y_arm + Lp * cos(alpha_history(i) + theta_history(i));
    
    % Update graphics
    set(h_arm, 'XData', [0 x_arm], 'YData', [0 y_arm]);
    set(h_pendulum, 'XData', [x_arm x_pend], 'YData', [y_arm y_pend]);
    set(h_pivot, 'XData', x_arm, 'YData', y_arm);
    set(h_mass, 'XData', x_pend, 'YData', y_pend);
    set(h_time, 'String', sprintf('Time: %.2f s', time(i)));
    
    % Control animation speed
    if i > 1
        dt = (time(i) - time(i-skip_frames)) / animation_speed;
        pause(dt);
    end
    
    drawnow;
end

disp('Animation complete!');
end
