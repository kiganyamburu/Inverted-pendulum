% Quick debug test
clear; clc;

% Load parameters
run('thamsosswingup20112025.m');

% Test at hanging down position
alpha = pi;  % Downward
alpha_dot = 0.1;
theta = 0;
theta_dot = 0;

E_kinetic = 0.5 * Jp_cm * alpha_dot^2;
E_potential = mp * g * l * (1 + cos(alpha));
E_total = E_kinetic + E_potential;

fprintf('Alpha = %.2f rad (%.1f deg)\n', alpha, alpha*180/pi);
fprintf('cos(alpha) = %.4f\n', cos(alpha));
fprintf('E_kinetic = %.6f J\n', E_kinetic);
fprintf('E_potential = %.6f J (formula: mp*g*l*(1+cos(alpha)))\n', E_potential);
fprintf('E_total = %.6f J\n', E_total);
fprintf('E_ref = %.6f J\n', E_ref);
fprintf('E_error = %.6f J\n', E_ref - E_total);
fprintf('\n');

% Test at upright
alpha_up = 0;
E_pot_up = mp * g * l * (1 + cos(alpha_up));
fprintf('At upright (alpha=0): E_pot = %.6f J\n', E_pot_up);
fprintf('Expected E_ref should be: %.6f J\n', E_pot_up);
fprintf('\n');

% Test control
Vm = swingup_control(theta, alpha, theta_dot, alpha_dot, struct('mp',mp,'l',l,'g',g,'Jp_cm',Jp_cm,'K',K,'mu',mu,'E_ref',E_ref,'alpha_threshold',alpha_threshold,'Vm_max',Vm_max));
fprintf('Control voltage: %.4f V\n', Vm);
