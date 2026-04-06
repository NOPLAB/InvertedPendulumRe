#![no_std]
#![no_main]

use embassy_time::{Duration, Timer};
use esp_hal::gpio::{Level, Output, OutputConfig};
use log::info;

extern crate esp_backtrace as _;
extern crate esp_println as _;

#[esp_rtos::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    info!("inverted-pendulum-esp32 started");

    // LED_1 (GPIO3) — 動作確認用に点滅
    let mut led1 = Output::new(peripherals.GPIO3, Level::Low, OutputConfig::default());

    loop {
        led1.toggle();
        Timer::after(Duration::from_millis(500)).await;
    }
}
