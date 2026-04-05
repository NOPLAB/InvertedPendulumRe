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

- `main.rs` — Embassy async runtime, task spawning, button control (start/stop)
- `constants.rs` — Motor/mechanical/ADC/control parameters
- `controller.rs` — LQR state feedback + current PID (full control pipeline)
- `motor.rs` — PWM H-bridge drive (left/right independent, TIM1/TIM2/TIM3)
- `encoder.rs` — X4 quadrature encoder via EXTI interrupts
- `adc.rs` — ADC reading (pendulum angle + motor currents, lock-free atomic)
- `filter.rs` — First-order low-pass filter
- `pid.rs` — PID controller with anti-windup

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
