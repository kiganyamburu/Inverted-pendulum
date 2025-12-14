%% Quick Test Script - Verify Controller Implementation
% This script performs basic checks on the control system

clear all;
close all;
clc;

disp('============================================');
disp('INVERTED PENDULUM SWING-UP CONTROL');
disp('System Verification');
disp('============================================');
disp(' ');

% Load parameters
run('thamsosswingup20112025.m');

%% Test 1: Energy Calculation
disp('Test 1: Energy Calculation');
disp('----------------------------');
alpha_test = 0;  % Upright
alpha_dot_test = 0;
E_kinetic = 0.5 * Jp_cm * alpha_dot_test^2;
E_potential = mp * g * l * (cos(alpha_test) - 1);
E_total = E_kinetic + E_potential;
disp(['Energy at upright (α=0): ', num2str(E_total), ' J (should be ≈ ', num2str(E_ref), ' J)']);

alpha_test = pi;  % Hanging down
E_potential = mp * g * l * (cos(alpha_test) - 1);
E_total = E_potential;
disp(['Energy at downward (α=π): ', num2str(E_total), ' J (should be ≈ 0 J)']);
disp(' ');

%% Test 2: LQR Controller Stability
disp('Test 2: LQR Controller Stability');
disp('----------------------------------');
eig_A = eig(A);
eig_closedloop = eig(A - B*K);
disp('Open-loop eigenvalues:');
disp(eig_A);
disp('Closed-loop eigenvalues (with LQR):');
disp(eig_closedloop);
if all(real(eig_closedloop) < 0)
    disp('✓ Closed-loop system is STABLE');
else
    disp('✗ WARNING: Closed-loop system may be UNSTABLE');
end
disp(' ');

%% Test 3: Swing-Up Control Law
disp('Test 3: Swing-Up Control Law');
disp('------------------------------');
params.mp = mp;
params.l = l;
params.g = g;
params.Jp_cm = Jp_cm;
params.K = K;
params.mu = mu;
params.E_ref = E_ref;
params.alpha_threshold = alpha_threshold;
params.Vm_max = Vm_max;

% Test case 1: Pendulum hanging down, swinging right
theta = 0;
alpha = pi;
theta_dot = 0;
alpha_dot = 1;
Vm = swingup_control(theta, alpha, theta_dot, alpha_dot, params);
disp(['Hanging down, α̇=+1: Vm = ', num2str(Vm), ' V']);

% Test case 2: Pendulum at upright
theta = 0;
alpha = 0.01;  % Nearly upright
theta_dot = 0;
alpha_dot = 0;
Vm = swingup_control(theta, alpha, theta_dot, alpha_dot, params);
disp(['Nearly upright, small error: Vm = ', num2str(Vm), ' V']);
disp(' ');

%% Test 4: Control Mode Switching
disp('Test 4: Control Mode Switching');
disp('--------------------------------');
disp(['LQR activation threshold: ±', num2str(alpha_threshold*180/pi), ' degrees']);
disp(['Velocity threshold: 5 rad/s (', num2str(5*180/pi), ' deg/s)']);
disp(' ');

%% Summary
disp('============================================');
disp('VERIFICATION COMPLETE');
disp('============================================');
disp(' ');
disp('Next steps:');
disp('1. Run "simulate_swingup" in MATLAB to see full simulation');
disp('2. Adjust parameters in thamsosswingup20112025.m if needed');
disp('3. Use with Simulink model: swingup20112025moi2018a.slx');
disp(' ');
disp('Files created:');
disp('  - thamsosswingup20112025.m      (parameters & LQR design)');
disp('  - swingup_control.m             (control function)');
disp('  - simulate_swingup.m            (simulation script)');
disp('  - animate_pendulum.m            (animation function)');
disp('  - README.md                     (documentation)');
