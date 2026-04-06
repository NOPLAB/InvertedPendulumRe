#![no_std]
#![no_main]

mod led;
mod uart;

use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::uart::{Config as UartConfig, Uart};
use log::info;

extern crate esp_backtrace as _;
extern crate esp_println as _;

#[esp_rtos::main]
async fn main(spawner: embassy_executor::Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    info!("inverted-pendulum-esp32 started");

    // UART: GPIO20 (RX) / GPIO21 (TX) — STM32と通信
    let uart_config = UartConfig::default().with_baudrate(115200);
    let uart = Uart::new(peripherals.UART0, uart_config)
        .unwrap()
        .with_rx(peripherals.GPIO20)
        .with_tx(peripherals.GPIO21)
        .into_async();
    let (uart_rx, uart_tx) = uart.split();

    // LED
    let led1 = Output::new(peripherals.GPIO3, Level::Low, OutputConfig::default());
    let led2 = Output::new(peripherals.GPIO6, Level::Low, OutputConfig::default());

    // タスク起動
    spawner.must_spawn(uart::uart_rx_task(uart_rx));
    spawner.must_spawn(uart::uart_tx_task(uart_tx));
    spawner.must_spawn(led::led_task(led1, led2));

    // Bridge loop: センサデータの配信 (BLE/WiFi実装後に完成)
    loop {
        let data = uart::SENSOR_SIGNAL.wait().await;
        // BLE/WiFi配信は後のタスクで追加
        let _ = data;
    }
}
