# STM32 Firmware Directory Restructure

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reorganize `inverted-pendulum-stm32/src/` from a flat layout into `config/`, `driver/`, `controller/`, `tasks/` modules for clear separation of concerns.

**Architecture:** Split the flat `src/*.rs` files into four module groups: `config/` (constants & conversions), `driver/` (HAL peripherals), `controller/` (control algorithms + generic building blocks), `tasks/` (Embassy async tasks). The `main.rs` becomes a thin entry point that only initializes hardware and spawns tasks.

**Tech Stack:** Rust (no_std, Embassy async, STM32F303K8)

---

## File Structure

### Current → Target mapping

```
src/
├── main.rs                              → main.rs (slim: init + button loop only)
├── constants.rs                         → DELETE (split into config/)
├── adc.rs                               → driver/adc.rs
├── encoder.rs                           → driver/encoder.rs
├── motor.rs                             → driver/motor.rs
├── filter.rs                            → controller/filter.rs
├── pid.rs                               → controller/pid.rs
├── fmt.rs                               → fmt.rs (unchanged)
└── controller/
    ├── mod.rs                           → controller/mod.rs (update imports)
    ├── lqr.rs                           → (update imports)
    ├── lqr_constants.rs                 → (unchanged)
    ├── pid_balance.rs                   → (update imports)
    ├── pid_balance_constants.rs         → (unchanged)
    ├── mrac.rs                          → (update imports + extract constants)
    ├── mrac_constants.rs                → REWRITE (current content is stale)
    ├── mpc.rs                           → (unchanged)
    ├── mpc_constants.rs                 → (unchanged)
    ├── observer.rs                      → (update imports)
    └── observer_constants.rs            → (unchanged)

NEW directories/files:
├── config/
│   ├── mod.rs                           re-exports all sub-modules
│   ├── hardware.rs                      motor, mechanical, ADC, encoder, PWM constants + conversion fns
│   ├── control.rs                       timing, limits, clamp, lpf_alpha
│   └── ui.rs                            button/LED/mode constants
├── driver/
│   ├── mod.rs                           re-exports
│   ├── adc.rs                           from src/adc.rs
│   ├── encoder.rs                       from src/encoder.rs
│   └── motor.rs                         from src/motor.rs
└── tasks/
    ├── mod.rs                           re-exports
    ├── control.rs                       control_task (from main.rs)
    └── encoder.rs                       encoder_r_task, encoder_l_task (from main.rs)
```

### Notes

- `main.rs` keeps: HAL init, peripheral setup, static atomics (`QEI_R`, `QEI_L`, `RUNNING`, `CONTROL_MODE`), `blink_mode`, button loop, task spawning. The button loop is the natural main loop for Embassy.
- `mrac_constants.rs` currently exists on disk with stale 5-state constants. It must be rewritten with the current 4-state constants from `mrac.rs` and re-added to `mod.rs`.
- `config/mod.rs` uses `pub use` re-exports so callers can write `use crate::config::MOTOR_KT` instead of `use crate::config::hardware::MOTOR_KT`.
- `no_std` environment — no test framework, verification is `cargo build`.

---

### Task 1: Create `config/` module (split `constants.rs`)

**Files:**
- Create: `src/config/mod.rs`
- Create: `src/config/hardware.rs`
- Create: `src/config/control.rs`
- Create: `src/config/ui.rs`

- [ ] **Step 1: Create `src/config/hardware.rs`**

```rust
// Motor constants
pub const MOTOR_KT: f32 = 0.0186; // Torque constant [Nm/A]
#[allow(dead_code)]
pub const MOTOR_KE: f32 = 0.0186; // Back EMF constant [V*s/rad]
#[allow(dead_code)]
pub const MOTOR_RA: f32 = 32.4; // Armature resistance [Ohm]
pub const MOTOR_LA: f32 = 2.955e-3; // Armature inductance [H]

// Mechanical constants
pub const WHEEL_RADIUS: f32 = 0.0255; // [m]
pub const GEAR_RATIO: f32 = 6.67;

// ADC constants
pub const ADC_RESOLUTION: u32 = 4096; // 12-bit
pub const ADC_VREF: f32 = 3.3;

pub const POT_ELECTRICAL_ANGLE: f32 = 333.3; // [deg]
pub const POT_SUPPLY_VOLTAGE: f32 = 5.0;
pub const VOLTAGE_DIVIDER_RATIO: f32 = ADC_VREF / POT_SUPPLY_VOLTAGE;
pub const ADC_TO_RAD: f32 =
    (POT_ELECTRICAL_ANGLE * (2.0 * core::f32::consts::PI / 360.0)) * VOLTAGE_DIVIDER_RATIO;

// Current sensing
pub const AMPLIFICATION_FACTOR: f32 = 150.0;
pub const SHUNT_RESISTOR: f32 = 0.010; // [Ohm]

// Encoder
pub const ENCODER_PULSES_PER_REV: i32 = 12 * 4; // 12 pulses * X4
#[allow(dead_code)]
pub const PULSE_TO_RAD: f32 = (2.0 * core::f32::consts::PI) / (12.0 * 4.0);
pub const PULSE_TO_POSITION: f32 =
    (2.0 * core::f32::consts::PI * WHEEL_RADIUS) / ((ENCODER_PULSES_PER_REV as f32) * GEAR_RATIO);

// Motor PWM
pub const MOTOR_PWM_FREQUENCY: u32 = 50_000; // [Hz]

// Utility functions
pub fn adc_to_radians(ad_value: u16, zero_offset: u16) -> f32 {
    let normalized_ad = (ad_value as f32) / (ADC_RESOLUTION as f32);
    let normalized_offset = (zero_offset as f32) / (ADC_RESOLUTION as f32);
    -(normalized_ad - normalized_offset) * ADC_TO_RAD
}

pub fn adc_to_current(adc_value: u16) -> f32 {
    let normalized = (adc_value as f32) / (ADC_RESOLUTION as f32);
    normalized * ADC_VREF / AMPLIFICATION_FACTOR / SHUNT_RESISTOR
}

pub fn adc_to_current_with_offset(adc_value: u16, offset: u16) -> f32 {
    let compensated = adc_value.saturating_sub(offset);
    adc_to_current(compensated)
}

pub fn pulses_to_position(pulses: i32) -> f32 {
    (pulses as f32) * PULSE_TO_POSITION
}
```

- [ ] **Step 2: Create `src/config/control.rs`**

```rust
use super::hardware::MOTOR_KT;
use super::hardware::WHEEL_RADIUS;
use super::hardware::GEAR_RATIO;

// Control
pub const CURRENT_CONTROL_FREQUENCY: u32 = 5_000; // [Hz] 電流制御ループ
pub const BALANCE_CONTROL_FREQUENCY: u32 = 1_000; // [Hz] 振り子制御ループ
pub const BALANCE_DECIMATION: u32 = CURRENT_CONTROL_FREQUENCY / BALANCE_CONTROL_FREQUENCY;
pub const CURRENT_DT: f32 = 1.0 / CURRENT_CONTROL_FREQUENCY as f32;
pub const BALANCE_DT: f32 = 1.0 / BALANCE_CONTROL_FREQUENCY as f32;
pub const MAX_FORCE: f32 = 10.0;
pub const MAX_VOLTAGE: f32 = 12.0;

pub const FORCE_TO_CURRENT: f32 = WHEEL_RADIUS / (GEAR_RATIO * MOTOR_KT * 2.0);

// Control modes
pub const NUM_MODES: u8 = 5;

// Utility functions
pub fn clamp(value: f32, min: f32, max: f32) -> f32 {
    if value > max {
        max
    } else if value < min {
        min
    } else {
        value
    }
}

pub fn lpf_alpha(cutoff_freq: f32, sample_time: f32) -> f32 {
    let tau = 1.0 / (2.0 * core::f32::consts::PI * cutoff_freq);
    sample_time / (tau + sample_time)
}
```

- [ ] **Step 3: Create `src/config/ui.rs`**

```rust
// Button timing
pub const LONG_PRESS_MS: u64 = 800;
pub const DEBOUNCE_MS: u64 = 50;

// LED blink timing
pub const BLINK_ON_MS: u64 = 150;
pub const BLINK_OFF_MS: u64 = 150;
```

- [ ] **Step 4: Create `src/config/mod.rs`**

```rust
pub mod control;
pub mod hardware;
pub mod ui;

pub use control::*;
pub use hardware::*;
pub use ui::*;
```

- [ ] **Step 5: Update `main.rs` module declaration**

Replace `mod constants;` with `mod config;`, update the use statement from `use constants::*;` to `use config::*;`.

- [ ] **Step 6: Update `controller/mod.rs` imports**

Replace:
```rust
use crate::constants::{clamp, BALANCE_DT, CURRENT_DT, FORCE_TO_CURRENT, MAX_FORCE, MAX_VOLTAGE};
```
With:
```rust
use crate::config::{clamp, BALANCE_DT, CURRENT_DT, FORCE_TO_CURRENT, MAX_FORCE, MAX_VOLTAGE};
```

- [ ] **Step 7: Update `controller/pid_balance.rs` imports**

Replace:
```rust
use crate::constants::{BALANCE_DT, MAX_FORCE};
```
With:
```rust
use crate::config::{BALANCE_DT, MAX_FORCE};
```

- [ ] **Step 8: Update `controller/mrac.rs` imports**

Replace:
```rust
use crate::constants::{clamp, BALANCE_DT};
```
With:
```rust
use crate::config::{clamp, BALANCE_DT};
```

- [ ] **Step 9: Update `controller/observer.rs` imports**

Replace:
```rust
use crate::constants::FORCE_TO_CURRENT;
```
With:
```rust
use crate::config::FORCE_TO_CURRENT;
```

- [ ] **Step 10: Delete `src/constants.rs`**

- [ ] **Step 11: Build verification**

Run: `cargo build`
Expected: success

- [ ] **Step 12: Commit**

```bash
git add -A
git commit -m "refactor(stm32): split constants.rs into config/ module (hardware, control, ui)"
```

---

### Task 2: Create `driver/` module (move HAL peripherals)

**Files:**
- Create: `src/driver/mod.rs`
- Create: `src/driver/adc.rs` (from `src/adc.rs`)
- Create: `src/driver/encoder.rs` (from `src/encoder.rs`)
- Create: `src/driver/motor.rs` (from `src/motor.rs`)
- Delete: `src/adc.rs`, `src/encoder.rs`, `src/motor.rs`

- [ ] **Step 1: Create `src/driver/mod.rs`**

```rust
pub mod adc;
pub mod encoder;
pub mod motor;
```

- [ ] **Step 2: Move `src/adc.rs` → `src/driver/adc.rs`**

Copy the file, then update its import from:
```rust
use crate::constants::*;
```
To:
```rust
use crate::config::*;
```

- [ ] **Step 3: Move `src/encoder.rs` → `src/driver/encoder.rs`**

Copy the file verbatim (no import changes needed — it only uses `embassy_stm32` and `embassy_futures`).

- [ ] **Step 4: Move `src/motor.rs` → `src/driver/motor.rs`**

Copy the file verbatim (no import changes needed — it only uses `embassy_stm32`).

- [ ] **Step 5: Update `main.rs` module declarations**

Replace:
```rust
mod adc;
mod encoder;
mod motor;
```
With:
```rust
mod driver;
```

Update use statements:
- `use adc::{...}` → `use driver::adc::{...}`
- `use encoder::Qei;` → `use driver::encoder::Qei;`
- `use motor::Motors;` → `use driver::motor::Motors;`

- [ ] **Step 6: Delete old files**

Delete `src/adc.rs`, `src/encoder.rs`, `src/motor.rs`.

- [ ] **Step 7: Build verification**

Run: `cargo build`
Expected: success

- [ ] **Step 8: Commit**

```bash
git add -A
git commit -m "refactor(stm32): move adc/encoder/motor into driver/ module"
```

---

### Task 3: Move `pid.rs` and `filter.rs` into `controller/`

**Files:**
- Create: `src/controller/pid.rs` (from `src/pid.rs`)
- Create: `src/controller/filter.rs` (from `src/filter.rs`)
- Modify: `src/controller/mod.rs` — add `mod pid; mod filter;`, update imports
- Delete: `src/pid.rs`, `src/filter.rs`

- [ ] **Step 1: Move `src/pid.rs` → `src/controller/pid.rs`**

Copy the file, update its import from:
```rust
use crate::constants::clamp;
```
To:
```rust
use crate::config::clamp;
```

- [ ] **Step 2: Move `src/filter.rs` → `src/controller/filter.rs`**

Copy the file, update its import from:
```rust
use crate::constants::lpf_alpha;
```
To:
```rust
use crate::config::lpf_alpha;
```

- [ ] **Step 3: Update `controller/mod.rs`**

Add module declarations (private — only used within controller):
```rust
mod filter;
mod pid;
```

Update imports from:
```rust
use crate::filter::LowPassFilter;
use crate::pid::Pid;
```
To:
```rust
use filter::LowPassFilter;
use pid::Pid;
```

- [ ] **Step 4: Remove old module declarations from `main.rs`**

Remove these lines from `main.rs`:
```rust
mod filter;
mod pid;
```

- [ ] **Step 5: Delete old files**

Delete `src/pid.rs`, `src/filter.rs`.

- [ ] **Step 6: Build verification**

Run: `cargo build`
Expected: success

- [ ] **Step 7: Commit**

```bash
git add -A
git commit -m "refactor(stm32): move pid/filter into controller/ module"
```

---

### Task 4: Rewrite `mrac_constants.rs` with current constants

**Files:**
- Rewrite: `src/controller/mrac_constants.rs`
- Modify: `src/controller/mod.rs` — re-add `pub mod mrac_constants;`
- Modify: `src/controller/mrac.rs` — extract constants, add `use super::mrac_constants::*;`

The current `mrac_constants.rs` on disk contains stale 5-state constants. The current `mrac.rs` has been rewritten to a 4-state force-output design with new inline constants (lines 109-130). Extract those to `mrac_constants.rs`.

- [ ] **Step 1: Rewrite `src/controller/mrac_constants.rs`**

```rust
// Designed in MATLAB/control/design_mrac.m
pub const MRAC_KX: [f32; 4] = [-3.162_277_7, -3.504_865_6, -24.942_75, -4.399_181_4];

pub const MRAC_AM: [[f32; 4]; 4] = [
    [0.0, 1.0, 0.0, 0.0],
    [5.475_094_3, 6.068_243_5, 42.799_32, 7.631_178],
    [0.0, 0.0, 0.0, 1.0],
    [-27.375_473, -30.341_217, -164.94661, -40.003_26],
];

pub const MRAC_PB: [f32; 4] = [-1.8973666, -2.0386274, -8.406_606, -1.395_032_3];
pub const MRAC_GAMMA_DIAG: [f32; 5] = [0.035, 0.006, 0.12, 0.012, 0.08];
pub const MRAC_SIGMA: f32 = 2.0;
pub const MRAC_NORMALIZATION_EPS: f32 = 0.1;
pub const MRAC_ADAPTATION_DELAY_SEC: f32 = 0.75;
pub const MRAC_ERROR_DEADZONE: f32 = 0.002;
pub const MRAC_MAX_ADAPTIVE_GAIN: f32 = 1.5;
pub const MRAC_MAX_ADAPTIVE_FORCE: f32 = 1.5;
pub const MRAC_ENABLE_ANGLE_LIMIT: f32 = 0.174_532_92;
pub const MRAC_ENABLE_ANGULAR_VELOCITY_LIMIT: f32 = 3.5;
pub const MRAC_ENABLE_POSITION_LIMIT: f32 = 0.35;
pub const MRAC_ENABLE_VELOCITY_LIMIT: f32 = 1.2;
```

- [ ] **Step 2: Update `controller/mrac.rs`**

Add import and remove inline constants. Replace the constants block (lines 109-130) with:
```rust
use super::mrac_constants::*;
```
Place this import at the top of the file alongside the existing imports.

- [ ] **Step 3: Re-add `mrac_constants` to `controller/mod.rs`**

Add `pub mod mrac_constants;` to the module declarations.

- [ ] **Step 4: Build verification**

Run: `cargo build`
Expected: success

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "refactor(stm32): extract mrac constants to mrac_constants.rs (4-state design)"
```

---

### Task 5: Create `tasks/` module (extract from `main.rs`)

**Files:**
- Create: `src/tasks/mod.rs`
- Create: `src/tasks/control.rs`
- Create: `src/tasks/encoder.rs`
- Modify: `src/main.rs` — remove extracted functions, add `mod tasks;`

- [ ] **Step 1: Create `src/tasks/encoder.rs`**

```rust
use core::sync::atomic::Ordering;

use crate::driver::encoder::Qei;

#[embassy_executor::task]
pub async fn encoder_r_task(mut qei: Qei<'static>) {
    loop {
        let result = qei.step().await;
        qei.reset();
        let prev = crate::QEI_R.load(Ordering::Relaxed);
        crate::QEI_R.store(prev + result.pulses, Ordering::Relaxed);
    }
}

#[embassy_executor::task]
pub async fn encoder_l_task(mut qei: Qei<'static>) {
    loop {
        let result = qei.step().await;
        qei.reset();
        let prev = crate::QEI_L.load(Ordering::Relaxed);
        crate::QEI_L.store(prev + result.pulses, Ordering::Relaxed);
    }
}
```

- [ ] **Step 2: Create `src/tasks/control.rs`**

```rust
use core::sync::atomic::Ordering;

use crate::config::*;
use crate::controller::{ControlMode, ControlSystem, State};
use crate::driver::adc::{calibrate_theta, get_currents, get_theta};
use crate::driver::motor::Motors;
use embassy_time::{Duration, Ticker};

const DEBUG_DECIMATION: u32 = 500; // 5kHz / 500 = 10Hz 表示

#[embassy_executor::task]
pub async fn control_task(mut motors: Motors) {
    let mut ticker = Ticker::every(Duration::from_hz(CURRENT_CONTROL_FREQUENCY as u64));
    let mut control_system = ControlSystem::new();
    let mut prev_qei_r: i32 = 0;
    let mut prev_qei_l: i32 = 0;
    let mut qei_r_offset: i32 = 0;
    let mut qei_l_offset: i32 = 0;
    let mut was_running = false;
    let mut balance_counter: u32 = 0;
    let mut current_mode = crate::CONTROL_MODE.load(Ordering::Relaxed);
    let mut debug_counter: u32 = 0;

    loop {
        // モード変更検出
        let mode = crate::CONTROL_MODE.load(Ordering::Relaxed);
        if mode != current_mode {
            control_system.set_mode(ControlMode::from_u8(mode));
            current_mode = mode;
        }

        let running = crate::RUNNING.load(Ordering::Relaxed);

        if running {
            let qei_r = crate::QEI_R.load(Ordering::Relaxed);
            let qei_l = crate::QEI_L.load(Ordering::Relaxed);

            // 起動直後: エンコーダオフセットを記録し、速度スパイクを防止
            if !was_running {
                qei_r_offset = qei_r;
                qei_l_offset = qei_l;
                prev_qei_r = qei_r;
                prev_qei_l = qei_l;
                balance_counter = 0;
                control_system.reset();
            }

            let theta = get_theta();
            let (current_r, current_l) = get_currents();

            // 振り子制御 (1kHz): BALANCE_DECIMATION 回に1回
            if balance_counter == 0 {
                let position_r = pulses_to_position(qei_r - qei_r_offset);
                let position_l = -pulses_to_position(qei_l - qei_l_offset);
                let velocity_r = pulses_to_position(qei_r - prev_qei_r) / BALANCE_DT;
                let velocity_l = -pulses_to_position(qei_l - prev_qei_l) / BALANCE_DT;
                prev_qei_r = qei_r;
                prev_qei_l = qei_l;

                let state = State {
                    theta,
                    position_r,
                    position_l,
                    velocity_r,
                    velocity_l,
                    current_r,
                    current_l,
                    vin: 12.0,
                };
                control_system.update_balance(&state);

                // デバッグモード: センサー値を10Hzで表示
                if current_mode == ControlMode::Debug as u8 {
                    debug_counter += 1;
                    if debug_counter >= DEBUG_DECIMATION / BALANCE_DECIMATION {
                        debug_counter = 0;
                        let pos = (position_r + position_l) / 2.0;
                        let vel = (velocity_r + velocity_l) / 2.0;
                        info!(
                            "th={} pos={} vel={} qR={} qL={} iR={} iL={}",
                            theta,
                            pos,
                            vel,
                            qei_r - qei_r_offset,
                            qei_l - qei_l_offset,
                            current_r,
                            current_l
                        );
                    }
                }
            }
            balance_counter = (balance_counter + 1) % BALANCE_DECIMATION;

            // 電流制御 (10kHz): 毎回実行
            let state = State {
                theta,
                position_r: 0.0,
                position_l: 0.0,
                velocity_r: 0.0,
                velocity_l: 0.0,
                current_r,
                current_l,
                vin: 12.0,
            };
            let output = control_system.update_current(&state);
            motors.set_both(output.left, output.right);
        } else {
            motors.stop();
        }

        was_running = running;
        ticker.next().await;
    }
}
```

- [ ] **Step 3: Create `src/tasks/mod.rs`**

```rust
pub mod control;
pub mod encoder;
```

- [ ] **Step 4: Update `main.rs`**

This is the largest edit. The new `main.rs` should be:

```rust
#![no_std]
#![no_main]

mod config;
mod controller;
mod driver;
mod fmt;
mod tasks;

use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU8, Ordering};

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use config::*;
use driver::adc::{adc_task, calibrate_current, calibrate_theta, AdcSensors};
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_stm32::{
    adc::Adc,
    bind_interrupts,
    exti::ExtiInput,
    gpio::{Level, Output, Pull, Speed},
    peripherals,
    rcc::AdcClockSource,
    time::Hertz,
    timer::simple_pwm::{PwmPin, SimplePwm},
};
use embassy_time::Timer;
use driver::encoder::Qei;
use driver::motor::Motors;
use tasks::control::control_task;
use tasks::encoder::{encoder_l_task, encoder_r_task};

bind_interrupts!(struct Irqs {
    ADC1_2 => embassy_stm32::adc::InterruptHandler<peripherals::ADC1>,
    embassy_stm32::adc::InterruptHandler<peripherals::ADC2>;
    EXTI4 => embassy_stm32::exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI4>;
    EXTI9_5 => embassy_stm32::exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI9_5>;
});

pub(crate) static QEI_R: AtomicI32 = AtomicI32::new(0);
pub(crate) static QEI_L: AtomicI32 = AtomicI32::new(0);
pub(crate) static RUNNING: AtomicBool = AtomicBool::new(false);
pub(crate) static CONTROL_MODE: AtomicU8 = AtomicU8::new(2); // デフォルト: LQR (2)

/// LED点滅でモード番号を表示
async fn blink_mode(led: &mut Output<'_>, count: u8) {
    for _ in 0..count {
        led.set_high();
        Timer::after_millis(BLINK_ON_MS).await;
        led.set_low();
        Timer::after_millis(BLINK_OFF_MS).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Clock: 64 MHz
    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc;
        config.rcc.sys = rcc::Sysclk::PLL1_P;
        config.rcc.pll = Some(rcc::Pll {
            src: rcc::PllSource::HSI,
            mul: rcc::PllMul::MUL16,
            prediv: rcc::PllPreDiv::DIV2,
        });
        config.rcc.adc = AdcClockSource::Pll(rcc::AdcPllPrescaler::DIV1);
        config.rcc.ahb_pre = rcc::AHBPrescaler::DIV1;
        config.rcc.apb1_pre = rcc::APBPrescaler::DIV2;
        config.rcc.apb2_pre = rcc::APBPrescaler::DIV1;
        config.rcc.mux.tim1sw = rcc::mux::Timsw::PCLK2_TIM;
        config.rcc.mux.tim2sw = rcc::mux::Tim2sw::PCLK1_TIM;
    }
    let p = embassy_stm32::init(config);

    let mut led = Output::new(p.PB6, Level::Low, Speed::Low);
    let mut button = ExtiInput::new(p.PB5, p.EXTI5, Pull::Up, Irqs);

    // ADC
    let adc1 = Adc::new(p.ADC1, Irqs);
    let adc2 = Adc::new(p.ADC2, Irqs);

    spawner.spawn(adc_task(AdcSensors::new(adc1, adc2), p.PB0, p.PA5, p.PA7).unwrap());
    Timer::after_millis(20).await;
    calibrate_current();

    // Encoders
    let qei_r = Qei::new(
        ExtiInput::new(p.PA6, p.EXTI6, Pull::None, Irqs),
        ExtiInput::new(p.PA4, p.EXTI4, Pull::None, Irqs),
        1000,
    );
    let qei_l = Qei::new(
        ExtiInput::new(p.PA8, p.EXTI8, Pull::Up, Irqs),
        ExtiInput::new(p.PA9, p.EXTI9, Pull::Up, Irqs),
        1000,
    );
    spawner.spawn(encoder_r_task(qei_r).unwrap());
    spawner.spawn(encoder_l_task(qei_l).unwrap());

    // Motors
    let motor_r = SimplePwm::new(
        p.TIM1,
        None,
        None,
        Some(PwmPin::new(
            p.PA10,
            embassy_stm32::gpio::OutputType::PushPull,
        )),
        Some(PwmPin::new(
            p.PA11,
            embassy_stm32::gpio::OutputType::PushPull,
        )),
        Hertz(MOTOR_PWM_FREQUENCY),
        embassy_stm32::timer::low_level::CountingMode::EdgeAlignedUp,
    );
    let motor_l_fwd = SimplePwm::new(
        p.TIM2,
        None,
        Some(PwmPin::new(
            p.PA1,
            embassy_stm32::gpio::OutputType::PushPull,
        )),
        None,
        None,
        Hertz(MOTOR_PWM_FREQUENCY),
        embassy_stm32::timer::low_level::CountingMode::EdgeAlignedUp,
    );
    let motor_l_rev = SimplePwm::new(
        p.TIM3,
        None,
        None,
        None,
        Some(PwmPin::new(
            p.PB7,
            embassy_stm32::gpio::OutputType::PushPull,
        )),
        Hertz(MOTOR_PWM_FREQUENCY),
        embassy_stm32::timer::low_level::CountingMode::EdgeAlignedUp,
    );
    let motors = Motors::new(motor_r, motor_l_fwd, motor_l_rev);

    spawner.spawn(control_task(motors).unwrap());

    // 起動時に現在モードを点滅表示
    let mode = CONTROL_MODE.load(Ordering::Relaxed);
    blink_mode(&mut led, mode + 1).await;

    // ボタン: 短押し=モード切替、長押し=起動/停止
    loop {
        button.wait_for_falling_edge().await;
        Timer::after_millis(DEBOUNCE_MS).await;

        // 長押し判定: rising edge vs タイムアウト
        let press = select(
            button.wait_for_rising_edge(),
            Timer::after_millis(LONG_PRESS_MS),
        )
        .await;

        match press {
            Either::First(_) => {
                // 短押し: 停止中のみモード切替
                if !RUNNING.load(Ordering::Relaxed) {
                    let current = CONTROL_MODE.load(Ordering::Relaxed);
                    let next = (current + 1) % NUM_MODES;
                    CONTROL_MODE.store(next, Ordering::Relaxed);
                    info!("Mode: {}", next);
                    // LED N回点滅 (モード番号 = next + 1)
                    blink_mode(&mut led, next + 1).await;
                }
            }
            Either::Second(_) => {
                // 長押し: 起動/停止トグル
                let was_running = RUNNING.load(Ordering::Relaxed);
                if was_running {
                    RUNNING.store(false, Ordering::Relaxed);
                    led.set_low();
                    info!("Control stopped");
                } else {
                    calibrate_theta();
                    RUNNING.store(true, Ordering::Relaxed);
                    led.set_high();
                    info!("Control started");
                }
                // ボタンリリースを待って再トリガー防止
                button.wait_for_rising_edge().await;
                Timer::after_millis(DEBOUNCE_MS).await;
            }
        }
    }
}
```

- [ ] **Step 5: Build verification**

Run: `cargo build`
Expected: success

- [ ] **Step 6: Commit**

```bash
git add -A
git commit -m "refactor(stm32): extract tasks/ module, slim down main.rs"
```

---

### Task 6: Update CLAUDE.md architecture section

**Files:**
- Modify: `CLAUDE.md`

- [ ] **Step 1: Update the Embedded Rust architecture section**

Replace the current architecture description under `## Embedded Rust (inverted-pendulum-stm32/)` → `### Architecture` with:

```markdown
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
```

- [ ] **Step 2: Commit**

```bash
git add CLAUDE.md
git commit -m "docs: update architecture section for new directory structure"
```
