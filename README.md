# Inverted Pendulum Swing-Up Control

## Overview

This project implements a complete control system for an inverted pendulum on a rotary arm (QUBE-Servo 2) using:

1. **Energy-based swing-up control** to bring the pendulum from hanging down to near-upright
2. **LQR (Linear Quadratic Regulator)** control to stabilize the pendulum at the upright position

## Files Description

### 1. `thamsosswingup20112025.m`

- Main parameter file containing all system parameters
- Defines physical constants (lengths, masses, inertias, motor parameters)
- Calculates state-space matrices A and B (linearized model)
- Designs LQR controller with specified Q and R matrices
- Displays controller gains and system properties

### 2. `swingup_control.m`

- Core control function implementing the hybrid controller
- **Swing-up mode**: Uses energy-based control when pendulum is far from upright
  - Control law: `Vm = μ × E_error × sign(α_dot × cos(α))`
  - Pumps energy into the system until it reaches reference energy
- **LQR mode**: Switches to LQR when pendulum is within threshold angle
  - Control law: `Vm = -K × x` where x is the state vector
  - Stabilizes the pendulum at upright position
- Includes voltage saturation for realistic motor limits

### 3. `simulate_swingup.m`

- Complete simulation of the swing-up and stabilization process
- Uses nonlinear dynamics for accurate system behavior
- Initial condition: Pendulum hanging down (α = π)
- Generates comprehensive plots:
  - System angles (arm and pendulum)
  - Angular velocities
  - Control input (motor voltage)
  - Energy and control mode switching
- Displays performance metrics

### 4. `animate_pendulum.m`

- Visualization function for animating the pendulum motion
- Shows real-time movement of arm and pendulum
- Can be called from simulation script to see the swing-up in action

## How to Use

### Run Simulation

```matlab
% In MATLAB command window:
simulate_swingup
```

This will:

1. Load all system parameters
2. Run the simulation for 20 seconds
3. Display performance metrics
4. Generate plots showing system behavior

### Add Animation (Optional)

To see the pendulum motion animated, add this line at the end of `simulate_swingup.m`:

```matlab
animate_pendulum(time, theta_history, alpha_history, r, Lp);
```

### Tune Controller Parameters

#### LQR Tuning (in `thamsosswingup20112025.m`)

```matlab
Q = diag([5 1 1 1]);  % State weights: [θ, α, θ̇, α̇]
R = 1;                 % Control input weight
```

- Increase Q(2) to make pendulum angle tracking more aggressive
- Increase Q(3,4) to reduce velocities
- Increase R to reduce control effort

#### Swing-Up Tuning (in `thamsosswingup20112025.m`)

```matlab
mu = 0.05;                        % Energy control gain
alpha_threshold = 20*pi/180;      % LQR switch threshold (degrees)
Vm_max = 10;                      % Maximum motor voltage
```

- Increase `mu` for faster swing-up (but may cause instability)
- Decrease `alpha_threshold` for tighter switching condition
- Adjust `Vm_max` based on your motor specifications

## Control Strategy

### 1. Swing-Up Phase

**When**: |α| > threshold OR |α̇| > 5 rad/s

The controller calculates the total mechanical energy:

```
E = E_kinetic + E_potential
  = 0.5 × J_p × α̇² + m_p × g × l × (cos(α) - 1)
```

Energy error:

```
E_error = E_ref - E_total
```

Control law:

```
V_m = μ × E_error × sign(α̇ × cos(α))
```

This pumps energy into the system, causing the pendulum to swing up.

### 2. Stabilization Phase

**When**: |α| < threshold AND |α̇| < 5 rad/s

Uses LQR control:

```
V_m = -K × [θ; α; θ̇; α̇]
```

The LQR gains K are pre-calculated to minimize the cost function:

```
J = ∫(x'Qx + u'Ru)dt
```

## System Model

### State Vector

```
x = [θ, α, θ̇, α̇]'
```

where:

- θ = arm angle (rad)
- α = pendulum angle from upright (rad), 0 = upright
- θ̇ = arm angular velocity (rad/s)
- α̇ = pendulum angular velocity (rad/s)

### Linearized Model (around upright position)

```
ẋ = Ax + Bu
```

### Nonlinear Model (used in simulation)

Full nonlinear equations derived from Lagrangian mechanics, accounting for:

- Coupled dynamics between arm and pendulum
- Gravitational torques
- Friction in both joints
- Motor dynamics

## Expected Results

With default parameters, the system should:

1. **Swing-up**: Take 2-4 seconds to bring pendulum near upright
2. **Stabilization**: Maintain pendulum within ±5° of upright
3. **Control effort**: Peak voltage around 8-10V during swing-up
4. **Final error**: < 1° steady-state error

## Troubleshooting

### Pendulum doesn't swing up

- Increase `mu` (energy gain)
- Check motor voltage limits (`Vm_max`)
- Verify initial conditions (should start at α = π)

### Unstable after reaching upright

- Reduce Q weights or increase R in LQR design
- Increase `alpha_threshold` for earlier switching to LQR
- Check system controllability (rank of controllability matrix)

### Too much overshoot

- Decrease `mu` for gentler swing-up
- Increase Q(3,4) to penalize high velocities
- Adjust switching threshold

## Theory References

The implementation is based on:

1. **Energy-based swing-up**: Åström & Furuta (2000)
2. **LQR control**: Standard optimal control theory
3. **System model**: Quanser QUBE-Servo 2 documentation

## Author

Implementation for inverted pendulum control project
Date: January 20, 2025
