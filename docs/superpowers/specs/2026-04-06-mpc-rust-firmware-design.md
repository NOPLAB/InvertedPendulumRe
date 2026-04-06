# MPC Rust Firmware Implementation Design

## Overview

Port the MATLAB MPC controller (ADMM-based condensed QP) to the STM32F303K8 Rust firmware, integrating as Mode 4 in the existing controller dispatcher.

## Architecture

- Observer runs at 1kHz (existing matrices, unchanged)
- MPC called every 10th balance loop iteration (100Hz)
- Between MPC calls, force target held constant (ZOH)
- ADMM solver: 50 fixed iterations, box constraints only

## Execution Flow

```
5kHz current loop (every tick)
  +-- 1kHz balance loop (every 5th tick) -- observer updates here
       +-- 100Hz MPC (every 10th balance tick)
            +-- ADMM 50 iterations -> force command
       +-- otherwise: hold previous force command
```

## Files

### New Files

- `src/controller/mpc.rs` — MpcController struct + ADMM solver
- `src/controller/mpc_constants.rs` — QP matrices as const arrays (generated from MATLAB)
- `MATLAB/control/export_mpc_rust.m` — MATLAB script to export matrices to Rust

### Modified Files

- `src/controller/mod.rs` — Add Mode4/MPC to ControlMode enum and dispatcher
- `src/main.rs` — Add MPC decimation counter (every 10th balance tick)
- `src/constants.rs` — Add MPC_DECIMATION = 10, MPC_MAX_FORCE = 10.0

## Memory Budget

| Item | Size | Location |
|------|------|----------|
| H_inv_rho (20x20 f32) | 1,600B | Flash (const) |
| F (20x4 f32) | 320B | Flash (const) |
| U, Z, W vectors (20x3 f32) | 240B | RAM (struct) |
| Total | ~2.2KB | Flash: 1.9KB, RAM: 240B |

## MPC Parameters

- Ts = 10ms (100Hz)
- N = 20 (prediction horizon, 200ms)
- Q = diag([100, 0, 500, 100])
- R = 1
- u_max = 10.0 N (box constraint)
- ADMM rho = 1.0, max_iter = 50

## Controller Interface

```rust
pub struct MpcController {
    u: [f32; N],      // primal variable
    z: [f32; N],      // auxiliary variable
    w: [f32; N],      // dual variable
    force: f32,       // last computed force (held between updates)
    first_call: bool,
}

impl MpcController {
    pub fn new() -> Self;
    pub fn compute_force(&mut self, state: &[f32; 4]) -> f32;
}
```

## Integration with Existing Code

- ControlMode enum: add `Mode4` variant
- ControlSystem: add `mpc: MpcController` field
- update_balance(): add Mode4 case using observer state, calling mpc.compute_force()
- main.rs control_task: add mpc_counter for 100Hz decimation; only call update_balance MPC path every 10th balance tick
- Current loop: MPC uses force->current->voltage path (same as LQR)
