#![no_std]
#![no_main]

mod ble;
mod led;
mod uart;
mod wifi;

use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::uart::{Config as UartConfig, Uart};
use esp_radio::ble::controller::BleConnector;
use log::info;
use static_cell::StaticCell;
use trouble_host::prelude::ExternalController;

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

    // Radio初期化 (BLE + WiFi共有)
    static RADIO: StaticCell<esp_radio::Controller<'static>> = StaticCell::new();
    let radio = RADIO.init(esp_radio::init().unwrap());

    // BLE: BleConnector → ExternalController
    let connector = BleConnector::new(radio, peripherals.BT, Default::default()).unwrap();
    let ble_controller: ExternalController<_, 20> = ExternalController::new(connector);

    // WiFi: WifiController + Interfaces
    let (wifi_controller, _wifi_interfaces) =
        esp_radio::wifi::new(radio, peripherals.WIFI, esp_radio::wifi::Config::default()).unwrap();

    // タスク起動
    spawner.must_spawn(uart::uart_rx_task(uart_rx));
    spawner.must_spawn(uart::uart_tx_task(uart_tx));
    spawner.must_spawn(led::led_task(led1, led2));
    spawner.must_spawn(ble::ble_task(ble_controller));
    spawner.must_spawn(wifi::wifi_task(wifi_controller));

    // Bridge loop: センサデータをBLE・WiFiへ配信
    loop {
        let data = uart::SENSOR_SIGNAL.wait().await;
        ble::BLE_SENSOR_CHANNEL.send(data).await;
        wifi::WIFI_SENSOR_CHANNEL.try_send(data).ok();
    }
}
