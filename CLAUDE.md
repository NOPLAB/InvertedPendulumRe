# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Cart-type inverted pendulum control system. MATLAB designs and simulates controllers, then MATLAB Coder generates C code that is compiled into the Rust firmware via FFI.

- **`MATLAB/`** — Plant modeling, controller design (LQR/PID/MPC/MRAC/Observer), simulation, and C code generation
- **`inverted-pendulum-controller/`** — Rust `no_std` crate wrapping MATLAB Coder-generated C code via `cc` build script + FFI
- **`inverted-pendulum-stm32/`** — Embedded Rust firmware for STM32F303K8 (Cortex-M4F, Embassy async)
- **`inverted-pendulum-esp32/`** — ESP32-C3 firmware (BLE/WiFi gateway, esp-hal + Embassy)
- **`inverted-pendulum-protocol/`** — Shared `no_std` crate: UART framing protocol (COBS + CRC8) between STM32 ↔ ESP32

## Build Commands

### MATLAB

```matlab
main        % Full pipeline: linearize → design → simulate → compare → animate
build       % Code generation only: designs controllers → generates C to inverted-pendulum-controller/codegen/
```

`main.m` and `build.m` add subdirectory paths automatically. Individual modules work standalone after calling `params()`.

### Rust — STM32 firmware

```bash
cd inverted-pendulum-stm32
cargo build              # debug (defmt logging enabled by default)
cargo build --release    # release (fat LTO, opt-level 3, panic=abort)
cargo run                # build + flash via probe-rs
cargo fmt && cargo clippy --all-features   # lint
```

Requires nightly Rust (`build-std = ["core"]`) and `probe-rs`. Target: `thumbv7em-none-eabi`.

**First build**: if `inverted-pendulum-controller/codegen/` is empty, `build.rs` invokes `matlab -batch "cd MATLAB; build"` automatically.

### Rust — ESP32 firmware

```bash
cd inverted-pendulum-esp32
cargo build              # target: riscv32imc-unknown-none-elf
cargo run                # flash via espflash --monitor
```

Requires stable Rust. Needs `espflash` installed.

### Rust — Protocol crate (has tests)

```bash
cd inverted-pendulum-protocol
cargo test               # runs COBS, CRC8, serialization roundtrip tests
```

## Architecture

### MATLAB → C → Rust pipeline

1. `MATLAB/build.m` designs all controllers and calls `codegen_controller()` to generate C source via MATLAB Coder
2. Generated C lands in `inverted-pendulum-controller/codegen/`
3. `inverted-pendulum-controller/build.rs` compiles all `.c` files (excluding `interface/`) via `cc` crate
4. `inverted-pendulum-controller/src/lib.rs` exposes safe Rust wrappers: `lqr_compute()`, `pid_compute()`, `mpc_compute()`, `mrac_compute()`, `observer_update()`
5. `inverted-pendulum-stm32` depends on `inverted-pendulum-controller` for control algorithms

### State vector

`[x, x_dot, theta, theta_dot]` where theta=0 is upright.

### STM32 firmware

Control pipeline: state estimation (observer) → controller (LQR/PID/MPC/MRAC) → current PID → PWM duty cycle.

- `src/config/` — Constants: hardware parameters, control loop timing, limits
- `src/driver/` — HAL peripheral drivers (ADC, encoder via EXTI, PWM H-bridge motor)
- `src/controller/` — Control algorithms (HAL-independent); `mod.rs` is the unified pipeline
- `src/tasks/` — Embassy async tasks (control loop @ 1kHz, current loop @ 5kHz, encoder interrupts)

### Communication

STM32 ↔ ESP32 communicate over UART using `inverted-pendulum-protocol`: COBS-framed, CRC8-verified, little-endian serialization. ESP32 bridges to BLE/WiFi.

### Simulink

The Simulink model is generated programmatically via `simulink/build_simulink_model.m`, not hand-built. Workspace variables `K_lqr`, `x0`, and PID gains must be set before running.

## Conventions

- Commit messages: Conventional Commits with scopes — `feat(stm32):`, `fix(matlab):`, `docs:`, etc.
- Source comments are in Japanese
- MATLAB: one function per file, filename matches function name, `snake_case` functions
- Rust: standard `rustfmt`, focused modules; preserve existing Japanese comments when editing
