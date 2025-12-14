% Khai báo tham số hệ thống
% System Parameters for QUBE-Servo 2 Inverted Pendulum

Lp  = 0.129;             % (m) Chiều dài con lắc
l  = Lp/2;              % (m) Chiều dài từ trục quay đến trọng tâm con lắc
r   = 0.085;             % (m) Chiều dài cánh tay
mp  = 0.024;             % (kg) Khối lượng con lắc
mr  = 0.095;             % (kg) Khối lượng cánh tay

Jr  = (mr*r^2)/3;         % (kg.m^2) Momen quán tính của con lắc với trục quay
Jp  = mp*(Lp^2)/3;         % (kg.m^2) Momen quán tính của cánh tay

Kt  = 0.042;             % (Nm/A) Hằng số moment động cơ
km  = 0.042;             % (Vs/rad) Hằng số sức điện động
Rm  = 8.4;               % (Ohm) Điện trở nội phần ứng

br  = 0.0015;            % (Nms/rad) Hệ số ma sát cho cánh tay quay
bp  = 0.005;             % (Nms/rad) Hệ số ma sát cho khung con lắc
g=9.81;
Vm = 1
Jp_cm = mp*Lp^2/12; % used to calculate pendulum energy in swing-up control
%Khai báo công thức



%Tham so trang thai ban dau
thetadotini=0;
alphaldotini=0;
thetaini=0*pi/180;
alphalini=0*pi/180;
theta_max=180*pi/180;


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
% A =[
% 
% 0                                       0                                                      1                                    0
% 0                                       0                                                      0                                    1
% 0 (g*l^2*mp^2*r)/(- l^2*mp^2*r^2 + Jp*Jr)    -(Jp*(Rm*br + km*Kt))/(Rm*(- l^2*mp^2*r^2 + Jp*Jr)) (bp*l*mp*r)/(- l^2*mp^2*r^2 + Jp*Jr)
% 0   -(Jr*g*l*mp)/(- l^2*mp^2*r^2 + Jp*Jr) (l*mp*r*(Rm*br + km*Kt))/(Rm*(- l^2*mp^2*r^2 + Jp*Jr))    -(Jr*bp)/(- l^2*mp^2*r^2 + Jp*Jr)];
% 
% 
% B =[
% 
%                                          0
%                                          0
%      (Jp*Kt)/(Rm*(- l^2*mp^2*r^2 + Jp*Jr))
% -(Kt*l*mp*r)/(Rm*(- l^2*mp^2*r^2 + Jp*Jr))];
  
 
% LQR Controller Design
Q = diag([10 50 1 5]);  % State weighting matrix [theta, alpha, theta_dot, alpha_dot]
R = 1;                   % Control input weighting

% Try using MATLAB's lqr if Control System Toolbox is available
% Otherwise use custom implementation or pre-calculated gains
try
    K = lqr(A,B,Q,R);    % Calculate LQR gains
    disp('Using MATLAB lqr function');
catch
    try
        % Try custom LQR implementation
        [K, ~, ~] = lqr_custom(A,B,Q,R);
        disp('Using custom lqr_custom function');
    catch
        % Use pre-calculated LQR gains for Q = diag([10 50 1 5]) and R = 1
        warning('LQR function not available. Using pre-calculated gains.');
        K = [-3.1623   84.6410   -3.1854    9.2746];  % Pre-calculated for new Q
        disp('Using pre-calculated LQR gains for Q = diag([10 50 1 5])');
    end
end
% T_sampling=0.01;
% K=lqrd(A,B,Q,R,0,T_sampling);
% Alternative gains: K1=[-2.45 39.24 -1.82 3.47];

%% Swing-Up Control Parameters
mu = 400;                % Energy control gain (tune this for swing-up performance)
E_ref = 2*mp*g*l;        % Reference energy (potential energy at upright position)
alpha_threshold = 35*pi/180;  % Switch to LQR when pendulum is within ±35 degrees of upright
Vm_max = 10;             % Maximum motor voltage (V)

%% Display Controller Gains
disp('LQR Controller Gains:');
disp(['K = [', num2str(K(1)), ', ', num2str(K(2)), ', ', num2str(K(3)), ', ', num2str(K(4)), ']']);
disp(' ');
disp('Swing-Up Parameters:');
disp(['Energy gain (mu) = ', num2str(mu)]);
disp(['Energy reference = ', num2str(E_ref), ' J']);
disp(['Switch threshold = ', num2str(alpha_threshold*180/pi), ' degrees']);
disp(' ');

%% Verify System Controllability
try
    Co = ctrb(A,B);
catch
    % Manual controllability matrix calculation
    n = size(A,1);
    Co = B;
    for i = 1:n-1
        Co = [Co, A^i * B];
    end
end
rank_Co = rank(Co);
disp(['Controllability Matrix Rank: ', num2str(rank_Co), ' (System dimension: ', num2str(size(A,1)), ')']);
if rank_Co == size(A,1)
    disp('System is CONTROLLABLE');
else
    disp('WARNING: System is NOT fully controllable!');
end
disp(' ');