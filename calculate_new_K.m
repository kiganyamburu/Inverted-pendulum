% Calculate LQR gains for new Q matrix
clear; clc;

% Load parameters
Lp  = 0.129;
l  = Lp/2;
r   = 0.085;
mp  = 0.024;
mr  = 0.095;
Jr  = (mr*r^2)/3;
Jp  = mp*(Lp^2)/3;
Kt  = 0.042;
km  = 0.042;
Rm  = 8.4;
br  = 0.0015;
bp  = 0.005;
g=9.81;

A =[
0                                       0                                                       1                                     0
0                                       0                                                       0                                     1
0 (g*l^2*mp^2*r)/(- l^2*mp^2*r^2 + Jp*Jr)     -(Jp*(Rm*br + km*Kt))/(Rm*(- l^2*mp^2*r^2 + Jp*Jr)) -(bp*l*mp*r)/(- l^2*mp^2*r^2 + Jp*Jr)
0    (Jr*g*l*mp)/(- l^2*mp^2*r^2 + Jp*Jr) -(l*mp*r*(Rm*br + km*Kt))/(Rm*(- l^2*mp^2*r^2 + Jp*Jr))     -(Jr*bp)/(- l^2*mp^2*r^2 + Jp*Jr)];

B =[ 
                                        0
                                        0
    (Jp*Kt)/(Rm*(- l^2*mp^2*r^2 + Jp*Jr))
(Kt*l*mp*r)/(Rm*(- l^2*mp^2*r^2 + Jp*Jr))];

Q = diag([10 50 1 5]);
R = 1;

% Iterative solution to Riccati equation (without toolbox)
S = Q;  % Initial guess
max_iter = 1000;
tol = 1e-8;

for iter = 1:max_iter
    K_temp = (R \ B') * S;
    % Riccati equation: A'*S + S*A - S*B*inv(R)*B'*S + Q = 0
    % Iterate: S_new = A'*S + S*A + Q - S*B*inv(R)*B'*S + S
    S_new = S + 0.01 * (A'*S + S*A - S*B*(R\B')*S + Q);
    
    if norm(S_new - S, 'fro') < tol
        fprintf('Converged after %d iterations\n', iter);
        break;
    end
    S = S_new;
end

K = (R \ B') * S;

fprintf('\nNew LQR gains with Q = diag([10 50 1 5]):\n');
fprintf('K = [%.4f, %.4f, %.4f, %.4f]\n', K(1), K(2), K(3), K(4));
fprintf('\nClosed-loop eigenvalues:\n');
eigs = eig(A - B*K);
disp(eigs);
if all(real(eigs) < 0)
    fprintf('\nSystem is STABLE\n');
else
    fprintf('\nWARNING: System is UNSTABLE\n');
end
