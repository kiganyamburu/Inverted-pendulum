function Vm = swingup_control(theta, alpha, theta_dot, alpha_dot, params)
% SWINGUP_CONTROL - Energy-based swing-up control with LQR stabilization
%
% Inputs:
%   theta       - Arm angle (rad)
%   alpha       - Pendulum angle from upright (rad), 0 = upright, pi = downward
%   theta_dot   - Arm angular velocity (rad/s)
%   alpha_dot   - Pendulum angular velocity (rad/s)
%   params      - Structure containing system parameters
%
% Output:
%   Vm          - Motor voltage command (V)

% Extract parameters
mp = params.mp;
l = params.l;
g = params.g;
Jp_cm = params.Jp_cm;
K = params.K;
mu = params.mu;
E_ref = params.E_ref;
alpha_threshold = params.alpha_threshold;
Vm_max = params.Vm_max;

% Calculate current pendulum energy
% E = Kinetic Energy + Potential Energy
E_kinetic = 0.5 * Jp_cm * alpha_dot^2;
E_potential = mp * g * l * (1 + cos(alpha));  % E=0 at downward (alpha=pi), E=2mgl at upright (alpha=0)
E_total = E_kinetic + E_potential;

% Energy error
E_error = E_ref - E_total;

% Check if pendulum is near upright position for LQR control
if abs(alpha) < alpha_threshold && abs(alpha_dot) < 5
    % Use LQR control for stabilization
    % State vector: x = [theta; alpha; theta_dot; alpha_dot]
    x = [theta; alpha; theta_dot; alpha_dot];
    Vm = -K * x;  % LQR control law
    
    % Indicate LQR mode (for debugging/visualization)
    % disp('Mode: LQR Stabilization');
else
    % Use energy-based swing-up control
    % Control law: Vm = -mu * E_error * sign(alpha_dot * cos(alpha))
    % Negative sign because positive voltage should add energy when pumping up
    control_sign = sign(alpha_dot * cos(alpha));
    if control_sign == 0
        control_sign = 1;  % Default to positive if at zero
    end
    Vm = -mu * E_error * control_sign;
    
    % Indicate swing-up mode (for debugging/visualization)
    % disp('Mode: Swing-Up');
end

% Saturate control signal to motor voltage limits
Vm = max(-Vm_max, min(Vm_max, Vm));

end
