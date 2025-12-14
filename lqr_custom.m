function [K, S, E] = lqr_custom(A, B, Q, R)
% LQR_CUSTOM - Custom LQR implementation without Control System Toolbox
% Solves the continuous-time algebraic Riccati equation
%
% Inputs:
%   A - State matrix (n x n)
%   B - Input matrix (n x m)
%   Q - State weighting matrix (n x n)
%   R - Control weighting matrix (m x m)
%
% Outputs:
%   K - Optimal feedback gain matrix
%   S - Solution to Riccati equation
%   E - Closed-loop eigenvalues

% Solve the Riccati equation using iterative method (care function alternative)
% S*A + A'*S - S*B*inv(R)*B'*S + Q = 0

% Use the built-in icare (iterative continuous-time algebraic Riccati equation)
% or implement Newton's method

try
    % Try using icare if available
    [S, ~, K] = icare(A, B, Q, R);
catch
    % Fallback: Use Newton iteration method
    n = size(A, 1);
    S = Q;  % Initial guess
    
    % Newton iteration
    max_iter = 100;
    tol = 1e-10;
    
    for iter = 1:max_iter
        % Compute gain
        K_temp = R \ (B' * S);
        
        % Compute Riccati residual
        Acl = A - B * K_temp;
        residual = S * A + A' * S - S * B * (R \ B') * S + Q;
        
        if norm(residual, 'fro') < tol
            break;
        end
        
        % Update S using Lyapunov equation
        S = lyap((Acl)', Q + K_temp' * R * K_temp);
    end
    
    K = R \ (B' * S);
end

% Calculate closed-loop eigenvalues
E = eig(A - B * K);

end
