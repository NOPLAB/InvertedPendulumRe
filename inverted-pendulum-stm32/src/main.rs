#![no_std]
#![no_main]

mod config;
mod controller;
mod driver;
mod fmt;
mod tasks;

use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, AtomicU8, Ordering};

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use driver::adc::{adc_task, calibrate_current, calibrate_theta, AdcSensors};
use config::*;
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
use tasks::balance::balance_task;
use tasks::encoder::{encoder_l_task, encoder_r_task};

bind_interrupts!(struct Irqs {
    ADC1_2 => embassy_stm32::adc::InterruptHandler<peripherals::ADC1>,
    embassy_stm32::adc::InterruptHandler<peripherals::ADC2>;
    EXTI4 => embassy_stm32::exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI4>;
    EXTI9_5 => embassy_stm32::exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI9_5>;
    USART2 => embassy_stm32::usart::InterruptHandler<peripherals::USART2>;
    DMA1_CHANNEL6 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH6>;
    DMA1_CHANNEL7 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH7>;
});

pub(crate) static QEI_R: AtomicI32 = AtomicI32::new(0);
pub(crate) static QEI_L: AtomicI32 = AtomicI32::new(0);
pub(crate) static RUNNING: AtomicBool = AtomicBool::new(false);
pub(crate) static CONTROL_MODE: AtomicU8 = AtomicU8::new(2); // デフォルト: LQR (2)
pub(crate) static TARGET_CURRENT: AtomicU32 = AtomicU32::new(0); // f32 bits: バランス→電流タスク間

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

    // MUX (TC4051BP) チャンネル選択: ch4 = VIN (C=1, B=0, A=0)
    let _mux_a = Output::new(p.PB4, Level::Low, Speed::Low);
    let _mux_b = Output::new(p.PB3, Level::Low, Speed::Low);
    let _mux_c = Output::new(p.PA12, Level::High, Speed::Low);

    // ADC
    let adc1 = Adc::new(p.ADC1, Irqs);
    let adc2 = Adc::new(p.ADC2, Irqs);

    spawner.spawn(adc_task(AdcSensors::new(adc1, adc2), p.PB0, p.PA0, p.PA5, p.PA7).unwrap());
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

    // UART: PA2 (TX) / PA3 (RX) — ESP32と通信
    let mut uart_config = embassy_stm32::usart::Config::default();
    uart_config.baudrate = 115200;
    let uart = embassy_stm32::usart::Uart::new(
        p.USART2, p.PA3, p.PA2, p.DMA1_CH7, p.DMA1_CH6, Irqs, uart_config,
    )
    .unwrap();
    let (uart_tx, uart_rx) = uart.split();

    spawner.spawn(tasks::uart::uart_tx_task(uart_tx).unwrap());
    spawner.spawn(tasks::uart::uart_rx_task(uart_rx).unwrap());

    tasks::current::init(p.TIM6, motors);
    spawner.spawn(balance_task().unwrap());

    // 起動時に現在モードを点滅表示
    let mode = CONTROL_MODE.load(Ordering::Relaxed);
    blink_mode(&mut led, mode + 1).await;

    // ボタン: 短押し=モード切替、長押し=起動/停止
    // UARTコマンド: ESP32からの制御指令
    loop {
        let event = select(
            button.wait_for_falling_edge(),
            tasks::uart::RX_COMMAND_SIGNAL.wait(),
        )
        .await;

        match event {
            Either::First(_) => {
                // ボタン押下
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
            Either::Second(cmd) => {
                // UART受信コマンド処理
                match cmd.command_type {
                    inverted_pendulum_protocol::CommandType::Start => {
                        calibrate_theta();
                        RUNNING.store(true, Ordering::Relaxed);
                        led.set_high();
                        info!("Control started (UART)");
                    }
                    inverted_pendulum_protocol::CommandType::Stop => {
                        RUNNING.store(false, Ordering::Relaxed);
                        led.set_low();
                        info!("Control stopped (UART)");
                    }
                    inverted_pendulum_protocol::CommandType::ModeChange => {
                        if !RUNNING.load(Ordering::Relaxed) {
                            let mode = cmd.payload[0] % 5;
                            CONTROL_MODE.store(mode, Ordering::Relaxed);
                            info!("Mode: {} (UART)", mode);
                            blink_mode(&mut led, mode + 1).await;
                        }
                    }
                    inverted_pendulum_protocol::CommandType::ParamSet => {}
                }
            }
        }
    }
}
