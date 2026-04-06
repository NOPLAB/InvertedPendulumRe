# MPC Controller Design for Inverted Pendulum

## Overview

Linear MPC with condensed QP formulation and ADMM solver for the cart-type inverted pendulum. Designed for STM32F303K8 (Cortex-M4F) deployment via Rust firmware.

## System Model

- State: x = [x, x_dot, theta, theta_dot] (4 states)
- Input: u = F (force, scalar)
- Discrete-time ZOH linearization at Ts = 10ms (100Hz control)
- Existing 5kHz current loop maintained as inner loop

## Optimization Problem

```
min  sum_{k=0}^{N-1} [ x_k' Q x_k + u_k' R u_k ] + x_N' P x_N
s.t. x_{k+1} = Ad x_k + Bd u_k
     u_min <= u_k <= u_max
```

### Parameters

- N = 20 (prediction horizon, 200ms)
- Q = diag([100, 0, 50, 100])
- R = 10
- P = solution of discrete algebraic Riccati equation (terminal cost)
- u_max = 10N (box constraint on input)

### Condensed Form

Eliminate state variables, reduce to:
```
min  (1/2) U' H U + f' U
s.t. u_min <= u_k <= u_max  (box constraints)
```
- U = [u_0, ..., u_{N-1}] (N x 1)
- H (N x N) and related matrices precomputed offline

## ADMM Solver

Update rules:
```
U  <- (H + rho*I)^{-1} (-f + rho*(Z - W))
Z  <- clamp(U + W, u_min, u_max)
W  <- W + U - Z
```

- (H + rho*I)^{-1} precomputed offline (constant matrix)
- Online: matrix-vector multiply + clamp + vector add
- Max iterations: 50 (fixed, deterministic worst-case)
- rho = 1.0

### Memory (N=20)

- H_inv: 20x20 = 1.6KB
- Work vectors (U, Z, W, f): ~320B
- Total: ~2KB (STM32F303K8 has 64KB RAM)

## MATLAB Implementation

### New Files

- `control/design_mpc.m` — MPC parameter design, offline precomputation
- `sim/mpc_controller.m` — ADMM solver + MPC control step

### Modified Files

- `sim/FirmwareExperiment.m` — Add mode='mpc', 100Hz balance loop branch
- `main.m` — Add 'mpc' to comparison modes

### Simulation Flow

```
FirmwareExperiment (5kHz physics)
  +-- Current loop (5kHz): unchanged
  +-- Balance loop: MPC at 100Hz (others at 1kHz)
       +-- mpc_controller: get state -> compute f -> ADMM -> output u_0
```
