#![no_std]
#![no_main]

mod adc;
mod constants;
mod controller;
mod encoder;
mod filter;
mod fmt;
mod motor;
mod pid;

use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU8, Ordering};

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use adc::{adc_task, calibrate_current, calibrate_theta, get_currents, get_theta, AdcSensors};
use constants::*;
use controller::{ControlMode, ControlSystem, State};
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
use embassy_time::{Duration, Ticker, Timer};
use encoder::Qei;
use motor::Motors;

bind_interrupts!(struct Irqs {
    ADC1_2 => embassy_stm32::adc::InterruptHandler<peripherals::ADC1>,
    embassy_stm32::adc::InterruptHandler<peripherals::ADC2>;
    EXTI4 => embassy_stm32::exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI4>;
    EXTI9_5 => embassy_stm32::exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI9_5>;
});

static QEI_R: AtomicI32 = AtomicI32::new(0);
static QEI_L: AtomicI32 = AtomicI32::new(0);
static RUNNING: AtomicBool = AtomicBool::new(false);
static CONTROL_MODE: AtomicU8 = AtomicU8::new(2); // デフォルト: LQR (2)

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

#[embassy_executor::task]
async fn encoder_r_task(mut qei: Qei<'static>) {
    loop {
        let result = qei.step().await;
        qei.reset();
        let prev = QEI_R.load(Ordering::Relaxed);
        QEI_R.store(prev + result.pulses, Ordering::Relaxed);
    }
}

#[embassy_executor::task]
async fn encoder_l_task(mut qei: Qei<'static>) {
    loop {
        let result = qei.step().await;
        qei.reset();
        let prev = QEI_L.load(Ordering::Relaxed);
        QEI_L.store(prev + result.pulses, Ordering::Relaxed);
    }
}

#[embassy_executor::task]
async fn control_task(mut motors: Motors) {
    let mut ticker = Ticker::every(Duration::from_hz(CURRENT_CONTROL_FREQUENCY as u64));
    let mut control_system = ControlSystem::new();
    let mut prev_qei_r: i32 = 0;
    let mut prev_qei_l: i32 = 0;
    let mut qei_r_offset: i32 = 0;
    let mut qei_l_offset: i32 = 0;
    let mut was_running = false;
    let mut balance_counter: u32 = 0;
    let mut current_mode = CONTROL_MODE.load(Ordering::Relaxed);
    let mut debug_counter: u32 = 0;
    const DEBUG_DECIMATION: u32 = 500; // 5kHz / 500 = 10Hz 表示

    loop {
        // モード変更検出
        let mode = CONTROL_MODE.load(Ordering::Relaxed);
        if mode != current_mode {
            control_system.set_mode(ControlMode::from_u8(mode));
            current_mode = mode;
        }

        let running = RUNNING.load(Ordering::Relaxed);

        if running {
            let qei_r = QEI_R.load(Ordering::Relaxed);
            let qei_l = QEI_L.load(Ordering::Relaxed);

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
                            theta, pos, vel,
                            qei_r - qei_r_offset, qei_l - qei_l_offset,
                            current_r, current_l
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
