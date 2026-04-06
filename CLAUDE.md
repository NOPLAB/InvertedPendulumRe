# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Cart-type inverted pendulum control system with two implementations:
- **`MATLAB/`** — MATLAB/Simulink simulation (LQR and PID controllers, nonlinear plant model)
- **`inverted-pendulum-stm32/`** — Embedded Rust firmware targeting STM32F303K8 (Cortex-M4F) using Embassy async framework

## MATLAB

### Running

```bash
# From MATLAB/ directory in MATLAB:
main   % Runs full pipeline: linearize → design controllers → simulate → compare → animate
```

`main.m` adds all subdirectory paths automatically. Individual modules can be run standalone after calling `params()`.

### Architecture

State vector: `[x, x_dot, theta, theta_dot]` where theta=0 is upright.

- `plant/params.m` — Physical parameters (masses, lengths, friction)
- `plant/plant_model.m` — Nonlinear equations of motion (used in RK4 integration)
- `plant/linearize_system.m` — State-space linearization around upright equilibrium
- `control/design_lqr.m` — LQR controller design (Q/R weight tuning)
- `control/design_pid.m` — Cascaded PID gains (inner: angle, outer: position)
- `sim/simulate_nonlinear.m` — RK4 simulation with dt=0.001s
- `simulink/build_simulink_model.m` — Programmatically generates `InvertedPendulum_Simulink.slx`

### Simulink

The Simulink model is generated programmatically via `build_simulink_model.m`, not hand-built. It includes a Manual Switch to toggle between LQR and PID at runtime. Workspace variables `K_lqr`, `x0`, and PID gains must be set before running.

## Embedded Rust (inverted-pendulum-stm32/)

### Build and Flash

```bash
cd inverted-pendulum-stm32
cargo build              # debug build (default feature includes defmt logging)
cargo build --release    # release build (LTO, opt-level 3, panic=abort)
cargo run                # build + flash via probe-rs (STM32F303K8Tx)
```

Requires nightly Rust (`build-std` is enabled) and `probe-rs` installed. Target is `thumbv7em-none-eabi`.

### Architecture

LQR state feedback → current PID → PWM duty cycle.

- `main.rs` — Embassy entry point, HAL initialization, button UI, task spawning
- `config/` — Constants and conversion functions
  - `hardware.rs` — Motor/mechanical/ADC/encoder/PWM physical parameters
  - `control.rs` — Control loop timing, limits, utility functions (clamp, lpf_alpha)
  - `ui.rs` — Button/LED timing constants
- `driver/` — HAL peripheral drivers
  - `adc.rs` — ADC reading (pendulum angle + motor currents, lock-free atomic)
  - `encoder.rs` — X4 quadrature encoder via EXTI interrupts
  - `motor.rs` — PWM H-bridge drive (left/right independent, TIM1/TIM2/TIM3)
- `controller/` — Control algorithms (HAL-independent)
  - `mod.rs` — ControlSystem: unified preprocessing + current control pipeline
  - `lqr.rs` / `lqr_constants.rs` — LQR state feedback
  - `pid_balance.rs` / `pid_balance_constants.rs` — Cascaded PID (angle + position)
  - `mrac.rs` / `mrac_constants.rs` — Model Reference Adaptive Control
  - `mpc.rs` / `mpc_constants.rs` — MPC with ADMM QP solver
  - `observer.rs` / `observer_constants.rs` — Discrete Luenberger state observer
  - `pid.rs` — Generic PID with anti-windup
  - `filter.rs` — First-order low-pass filter
- `tasks/` — Embassy async tasks
  - `control.rs` — Main control loop task (balance @ 1kHz + current @ 5kHz)
  - `encoder.rs` — Encoder interrupt tasks (left/right)

### Key Dependencies

- `embassy-stm32` (STM32F303K8 HAL + time driver)
- `embassy-executor` (async task executor for Cortex-M)
- `defmt` + `defmt-rtt` (structured logging over RTT, enabled by default `debug` feature)
- `micromath` (no_std floating-point math)

### Features

- `default = ["debug"]` — includes defmt logging, RTT, panic-probe
- Release builds should use `--release` (disables debug info, enables fat LTO)

## Conventions

- Commit messages use Conventional Commits format (`feat:`, `fix:`, `chore:`, etc.)
- Source comments are in Japanese where applicable
