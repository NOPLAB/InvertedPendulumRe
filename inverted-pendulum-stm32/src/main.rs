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

use core::sync::atomic::{AtomicBool, AtomicI32, Ordering};

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use adc::{adc_task, calibrate_theta, get_currents, get_theta, AdcSensors};
use constants::*;
use controller::{Controller, State};
use embassy_executor::Spawner;
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
});

static QEI_R: AtomicI32 = AtomicI32::new(0);
static QEI_L: AtomicI32 = AtomicI32::new(0);
static RUNNING: AtomicBool = AtomicBool::new(false);

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
    let mut button = ExtiInput::new(p.PB5, p.EXTI5, Pull::Up);

    // ADC
    let mut adc1 = Adc::new(p.ADC1, Irqs);
    adc1.set_sample_time(embassy_stm32::adc::SampleTime::CYCLES601_5);
    let mut adc2 = Adc::new(p.ADC2, Irqs);
    adc2.set_sample_time(embassy_stm32::adc::SampleTime::CYCLES601_5);

    spawner
        .spawn(adc_task(
            AdcSensors::new(adc1, adc2),
            p.PB0,
            p.PA5,
            p.PA7,
        ))
        .unwrap();

    // Encoders
    let qei_r = Qei::new(
        ExtiInput::new(p.PA6, p.EXTI6, Pull::None),
        ExtiInput::new(p.PA4, p.EXTI4, Pull::None),
        1000,
    );
    let qei_l = Qei::new(
        ExtiInput::new(p.PA8, p.EXTI8, Pull::Up),
        ExtiInput::new(p.PA9, p.EXTI9, Pull::Up),
        1000,
    );
    spawner.spawn(encoder_r_task(qei_r)).unwrap();
    spawner.spawn(encoder_l_task(qei_l)).unwrap();

    // Motors
    let motor_r = SimplePwm::new(
        p.TIM1,
        None,
        None,
        Some(PwmPin::new_ch3(p.PA10, embassy_stm32::gpio::OutputType::PushPull)),
        Some(PwmPin::new_ch4(p.PA11, embassy_stm32::gpio::OutputType::PushPull)),
        Hertz(MOTOR_PWM_FREQUENCY),
        embassy_stm32::timer::low_level::CountingMode::EdgeAlignedUp,
    );
    let motor_l_fwd = SimplePwm::new(
        p.TIM2,
        None,
        Some(PwmPin::new_ch2(p.PA1, embassy_stm32::gpio::OutputType::PushPull)),
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
        Some(PwmPin::new_ch4(p.PB7, embassy_stm32::gpio::OutputType::PushPull)),
        Hertz(MOTOR_PWM_FREQUENCY),
        embassy_stm32::timer::low_level::CountingMode::EdgeAlignedUp,
    );
    let motors = Motors::new(motor_r, motor_l_fwd, motor_l_rev);

    spawner.spawn(control_task(motors)).unwrap();

    // Button: toggle control on/off
    loop {
        button.wait_for_falling_edge().await;
        Timer::after_millis(200).await;

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
    let mut ticker = Ticker::every(Duration::from_hz(CONTROL_FREQUENCY as u64));
    let mut controller = Controller::new();
    let mut prev_qei_r: i32 = 0;
    let mut prev_qei_l: i32 = 0;

    loop {
        if RUNNING.load(Ordering::Relaxed) {
            let theta = get_theta();
            let (current_r, current_l) = get_currents();
            let qei_r = QEI_R.load(Ordering::Relaxed);
            let qei_l = QEI_L.load(Ordering::Relaxed);

            let position_r = pulses_to_position(qei_r);
            let position_l = -pulses_to_position(qei_l);
            let velocity_r = pulses_to_position(qei_r - prev_qei_r) / DT;
            let velocity_l = -pulses_to_position(qei_l - prev_qei_l) / DT;
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
                vin: 7.2, // TODO: read from ADC multiplexer
            };

            let output = controller.update(&state);
            motors.set_both(output.left, output.right);
        } else {
            motors.stop();
            controller.reset();
            prev_qei_r = QEI_R.load(Ordering::Relaxed);
            prev_qei_l = QEI_L.load(Ordering::Relaxed);
        }

        ticker.next().await;
    }
}
