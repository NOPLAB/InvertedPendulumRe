# Repository Guidelines

## Project Structure & Module Organization
This repository has two active codebases. `MATLAB/` contains controller design, plant models, firmware-aligned simulation, and Simulink assets. Key folders are `plant/`, `control/`, `sim/`, and `simulink/`. `inverted-pendulum-stm32/` contains the embedded Rust firmware for STM32F303K8; main application code lives under `src/`, with controllers in `src/controller/`. Design notes and implementation plans live in `docs/`. Avoid editing `MATLAB/resources/project/` unless you are intentionally changing MATLAB project metadata.

## Build, Test, and Development Commands
Use the command set that matches the subsystem you touch.

- `cd inverted-pendulum-stm32 && cargo build` builds the debug firmware for `thumbv7em-none-eabi`.
- `cd inverted-pendulum-stm32 && cargo build --release` builds the optimized firmware used for deployment.
- `cd inverted-pendulum-stm32 && cargo run` flashes and runs through `probe-rs` (`STM32F303K8Tx` runner is configured in `.cargo/config.toml`).
- `cd inverted-pendulum-stm32 && cargo fmt && cargo clippy --all-features` formats and lint-checks Rust changes.
- In MATLAB, run `main` from `MATLAB/` to execute the full design and simulation pipeline.
- Use `run_simulink_model` or `build_simulink_model` only when working on Simulink assets.

## Coding Style & Naming Conventions
Rust follows standard `rustfmt` style: 4-space indentation, `snake_case` modules/functions, `CamelCase` types, and focused modules such as `mpc.rs` or `observer.rs`. MATLAB files should keep one top-level function per file, with the filename matching the function name, `snake_case` function names, and aligned struct field assignments as seen in `plant/params.m`. Keep comments short and technical; preserve existing Japanese comments when editing nearby logic.

## Testing Guidelines
There is no established automated test suite yet. Validate changes with `cargo build`, `cargo clippy --all-features`, and the relevant MATLAB simulation entry point, usually `main` or `simulate_firmware`. If you add testable host-side Rust logic, prefer local `#[cfg(test)]` unit tests near the module they cover and name them after behavior, for example `computes_mpc_force_bounds`.

## Commit & Pull Request Guidelines
Recent history uses Conventional Commits with optional scopes, for example `feat(stm32): ...`, `feat(matlab): ...`, `docs: ...`, and `refactor: ...`. Keep commits small and imperative. PRs should state which subsystem changed, list the verification steps you ran, link the relevant issue or design note, and include plots or hardware logs when behavior changes are visible in simulation or on-device.
