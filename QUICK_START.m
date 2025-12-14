% ===================================================================
% QUICK START GUIDE - Inverted Pendulum Swing-Up Control
% ===================================================================
%
% This guide will get you running the swing-up control quickly!
%
% ===================================================================
% STEP 1: Verify Implementation
% ===================================================================
%
% Run this command to check if everything is working:
%
%   >> test_control
%
% This will verify:
%   ✓ Energy calculations are correct
%   ✓ LQR controller is stable
%   ✓ Swing-up control law is functioning
%   ✓ All files are present
%
% ===================================================================
% STEP 2: Run Full Simulation
% ===================================================================
%
% Run this command to see the complete swing-up simulation:
%
%   >> simulate_swingup
%
% This will:
%   ✓ Simulate 20 seconds of pendulum motion
%   ✓ Start with pendulum hanging down
%   ✓ Swing up and stabilize at upright position
%   ✓ Display 4 plots showing system behavior
%   ✓ Print performance metrics
%
% Expected output:
%   - Time to upright: 2-4 seconds
%   - Final angle error: < 1 degree
%   - Maximum voltage: 8-10V
%
% ===================================================================
% STEP 3: Add Animation (Optional)
% ===================================================================
%
% To see the pendulum motion animated, add this line at the end of
% simulate_swingup.m (before the last "end"):
%
%   animate_pendulum(time, theta_history, alpha_history, r, Lp);
%
% Then run:
%
%   >> simulate_swingup
%
% ===================================================================
% STEP 4: Tune Parameters (If Needed)
% ===================================================================
%
% Edit thamsosswingup20112025.m to adjust:
%
% For LQR control:
%   Q = diag([5 1 1 1]);  % Increase for tighter control
%   R = 1;                 % Increase to reduce control effort
%
% For swing-up:
%   mu = 0.05;            % Increase for faster swing-up
%   alpha_threshold = 20*pi/180;  % Decrease for earlier LQR activation
%   Vm_max = 10;          % Set to your motor's voltage limit
%
% After editing, save and run simulate_swingup again.
%
% ===================================================================
% STEP 5: Use with Simulink
% ===================================================================
%
% To use with your Simulink model (swingup20112025moi2018a.slx):
%
% 1. Load parameters:
%      >> run('thamsosswingup20112025.m')
%
% 2. In Simulink, add a MATLAB Function block
%
% 3. Use this code in the MATLAB Function block:
%
%      function Vm = control_fcn(theta, alpha, theta_dot, alpha_dot)
%          % Load parameters (make them persistent or pass as arguments)
%          persistent params
%          if isempty(params)
%              % Initialize params structure
%              % (copy from simulate_swingup.m)
%          end
%          Vm = swingup_control(theta, alpha, theta_dot, alpha_dot, params);
%      end
%
% 4. Connect states to the function inputs
%
% 5. Run Simulink simulation
%
% ===================================================================
% FILES OVERVIEW
% ===================================================================
%
% Main files:
%   thamsosswingup20112025.m  - Parameters and LQR design
%   swingup_control.m         - Control function (swing-up + LQR)
%   simulate_swingup.m        - Full simulation script
%   animate_pendulum.m        - Animation function
%
% Documentation:
%   README.md                 - Complete documentation
%   IMPLEMENTATION_SUMMARY.md - What was implemented
%   QUICK_START.m             - This file
%
% Testing:
%   test_control.m            - Verification script
%
% Simulink:
%   swingup20112025moi2018a.slx - Your Simulink model
%
% ===================================================================
% TROUBLESHOOTING
% ===================================================================
%
% Problem: Pendulum doesn't swing up
% Solution: Increase mu to 0.1 or higher
%
% Problem: Unstable at upright position
% Solution: Increase Q(2) to 5 or 10 in LQR design
%
% Problem: Too much control effort
% Solution: Increase R to 2 or 5 in LQR design
%
% Problem: Slow swing-up
% Solution: Increase mu and/or Vm_max
%
% ===================================================================
% SUPPORT
% ===================================================================
%
% For detailed explanations, see:
%   - README.md for theory and usage
%   - IMPLEMENTATION_SUMMARY.md for technical details
%
% ===================================================================
% READY TO START?
% ===================================================================
%
% Just run:
%
%   >> test_control
%
% And then:
%
%   >> simulate_swingup
%
% Enjoy your swing-up control! 🎯
%
% ===================================================================
