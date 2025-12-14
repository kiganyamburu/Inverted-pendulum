%% Inverted Pendulum Swing-Up and LQR Control Simulation
% This script simulates the complete swing-up and stabilization control

clear all;
close all;
clc;

% Run the parameter initialization script
run('thamsosswingup20112025.m');

%% Package parameters for control function
params.mp = mp;
params.l = l;
params.g = g;
params.Jp_cm = Jp_cm;
params.K = K;
params.mu = mu;
params.E_ref = E_ref;
params.alpha_threshold = alpha_threshold;
params.Vm_max = Vm_max;
params.Lp = Lp;
params.r = r;
params.mr = mr;
params.Jr = Jr;
params.Jp = Jp;
params.Kt = Kt;
params.km = km;
params.Rm = Rm;
params.br = br;
params.bp = bp;

%% Simulation Parameters
t_sim = 30;              % Simulation time (seconds)
dt = 0.001;              % Time step (seconds)
time = 0:dt:t_sim;       % Time vector
n_steps = length(time);

%% Initial Conditions
% Start with pendulum hanging down
theta = thetaini;        % Arm angle
alpha = alphalini + pi;  % Pendulum angle (pi = downward)
theta_dot = thetadotini;
alpha_dot = 0.5;         % Initial velocity to kick-start swing-up

% Storage arrays
theta_history = zeros(1, n_steps);
alpha_history = zeros(1, n_steps);
theta_dot_history = zeros(1, n_steps);
alpha_dot_history = zeros(1, n_steps);
Vm_history = zeros(1, n_steps);
energy_history = zeros(1, n_steps);
mode_history = zeros(1, n_steps);  % 0 = swing-up, 1 = LQR

%% Simulation Loop
disp('Starting simulation...');
for i = 1:n_steps
    % Store current state
    theta_history(i) = theta;
    alpha_history(i) = alpha;
    theta_dot_history(i) = theta_dot;
    alpha_dot_history(i) = alpha_dot;
    
    % Calculate energy
    E_kinetic = 0.5 * Jp_cm * alpha_dot^2;
    E_potential = mp * g * l * (1 + cos(alpha));
    energy_history(i) = E_kinetic + E_potential;
    
    % Determine control mode
    if abs(alpha) < alpha_threshold && abs(alpha_dot) < 5
        mode_history(i) = 1;  % LQR mode
    else
        mode_history(i) = 0;  % Swing-up mode
    end
    
    % Calculate control input
    Vm = swingup_control(theta, alpha, theta_dot, alpha_dot, params);
    Vm_history(i) = Vm;
    
    % Calculate nonlinear dynamics (using the full nonlinear equations)
    % Current (A)
    Im = (Vm - km*theta_dot) / Rm;
    
    % Torque
    tau = Kt * Im;
    
    % Nonlinear equations of motion (from Quanser QUBE-Servo 2 Workbook)
    % Equation 1: (Jr + Jp*sin²α)θ̈ + mp*l*r*cos(α)α̈ + 2*Jp*sin(α)*cos(α)θ̇α̇ - mp*l*r*sin(α)α̇² = τ - br*θ̇
    % Equation 2: Jp*α̈ + mp*l*r*cos(α)θ̈ - Jp*sin(α)*cos(α)θ̇² + mp*g*l*sin(α) = -bp*α̇
    
    % Mass matrix M
    M11 = Jr + Jp*sin(alpha)^2;
    M12 = mp*l*r*cos(alpha);
    M21 = mp*l*r*cos(alpha);
    M22 = Jp;
    
    % Force vector F
    F1 = tau - br*theta_dot - 2*Jp*sin(alpha)*cos(alpha)*theta_dot*alpha_dot + mp*l*r*sin(alpha)*alpha_dot^2;
    F2 = -bp*alpha_dot + Jp*sin(alpha)*cos(alpha)*theta_dot^2 - mp*g*l*sin(alpha);
    
    % Solve M * [θ̈; α̈] = F
    M = [M11, M12; M21, M22];
    F = [F1; F2];
    accel = M \ F;
    
    theta_ddot = accel(1);
    alpha_ddot = accel(2);
    
    % Integrate (Euler method)
    theta_dot = theta_dot + theta_ddot * dt;
    alpha_dot = alpha_dot + alpha_ddot * dt;
    theta = theta + theta_dot * dt;
    alpha = alpha + alpha_dot * dt;
    
    % Wrap alpha to [-pi, pi]
    alpha = mod(alpha + pi, 2*pi) - pi;
end

disp('Simulation complete!');
disp(' ');

%% Plot Results
figure('Name', 'Inverted Pendulum Swing-Up Control', 'Position', [100, 100, 1200, 800]);

% Plot 1: Angles
subplot(4,1,1);
plot(time, theta_history*180/pi, 'b', 'LineWidth', 1.5);
hold on;
plot(time, alpha_history*180/pi, 'r', 'LineWidth', 1.5);
ylabel('Angle (deg)');
xlabel('Time (s)');
legend('Arm Angle (\theta)', 'Pendulum Angle (\alpha)', 'Location', 'best');
title('System Angles');
grid on;

% Plot 2: Angular Velocities
subplot(4,1,2);
plot(time, theta_dot_history*180/pi, 'b', 'LineWidth', 1.5);
hold on;
plot(time, alpha_dot_history*180/pi, 'r', 'LineWidth', 1.5);
ylabel('Angular Velocity (deg/s)');
xlabel('Time (s)');
legend('Arm Velocity', 'Pendulum Velocity', 'Location', 'best');
title('Angular Velocities');
grid on;

% Plot 3: Control Input
subplot(4,1,3);
plot(time, Vm_history, 'g', 'LineWidth', 1.5);
hold on;
plot(time, Vm_max*ones(size(time)), 'r--', 'LineWidth', 1);
plot(time, -Vm_max*ones(size(time)), 'r--', 'LineWidth', 1);
ylabel('Voltage (V)');
xlabel('Time (s)');
legend('Motor Voltage (V_m)', 'Limits', 'Location', 'best');
title('Control Input');
grid on;
ylim([-Vm_max-1, Vm_max+1]);

% Plot 4: Energy and Mode
subplot(4,1,4);
yyaxis left;
plot(time, energy_history, 'b', 'LineWidth', 1.5);
hold on;
plot(time, E_ref*ones(size(time)), 'r--', 'LineWidth', 1.5);
ylabel('Energy (J)');
xlabel('Time (s)');
legend('Total Energy', 'Reference Energy', 'Location', 'northwest');

yyaxis right;
plot(time, mode_history, 'm', 'LineWidth', 1.5);
ylabel('Control Mode');
ylim([-0.1, 1.1]);
yticks([0, 1]);
yticklabels({'Swing-Up', 'LQR'});

title('Energy and Control Mode');
grid on;

%% Performance Metrics
final_alpha_error = abs(alpha_history(end)) * 180/pi;
disp('Performance Metrics:');
disp(['Final pendulum angle error: ', num2str(final_alpha_error), ' degrees']);
disp(['Maximum control voltage: ', num2str(max(abs(Vm_history))), ' V']);
disp(['Average control effort: ', num2str(mean(abs(Vm_history))), ' V']);

% Find time to reach upright position (within 10 degrees)
upright_reached = find(abs(alpha_history) < 10*pi/180, 1);
if ~isempty(upright_reached)
    time_to_upright = time(upright_reached);
    disp(['Time to reach upright (±10°): ', num2str(time_to_upright), ' seconds']);
else
    disp('Pendulum did not reach upright position within simulation time');
end

disp(' ');
disp('Simulation plots generated successfully!');
