use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use esp_hal::gpio::Output;
use log::info;

#[derive(Clone, Copy, PartialEq)]
pub enum BleStatus {
    Disconnected,
    Advertising,
    Connected,
}

#[derive(Clone, Copy, PartialEq)]
pub enum WifiStatus {
    Disconnected,
    Connecting,
    StaConnected,
    ApMode,
}

pub static BLE_STATUS: Signal<CriticalSectionRawMutex, BleStatus> = Signal::new();
pub static WIFI_STATUS: Signal<CriticalSectionRawMutex, WifiStatus> = Signal::new();

#[embassy_executor::task]
pub async fn led_task(mut led1: Output<'static>, mut led2: Output<'static>) {
    info!("led_task started");
    let mut ble_status = BleStatus::Disconnected;
    let mut wifi_status = WifiStatus::Disconnected;
    let mut tick: u32 = 0;

    loop {
        if BLE_STATUS.signaled() {
            ble_status = BLE_STATUS.wait().await;
        }
        if WIFI_STATUS.signaled() {
            wifi_status = WIFI_STATUS.wait().await;
        }

        match ble_status {
            BleStatus::Disconnected => led1.set_low(),
            BleStatus::Connected => led1.set_high(),
            BleStatus::Advertising => {
                if tick % 4 < 2 {
                    led1.set_high();
                } else {
                    led1.set_low();
                }
            }
        }

        match wifi_status {
            WifiStatus::Disconnected => led2.set_low(),
            WifiStatus::StaConnected => led2.set_high(),
            WifiStatus::Connecting => {
                if tick % 4 < 2 {
                    led2.set_high();
                } else {
                    led2.set_low();
                }
            }
            WifiStatus::ApMode => {
                if tick % 2 == 0 {
                    led2.set_high();
                } else {
                    led2.set_low();
                }
            }
        }

        tick = tick.wrapping_add(1);
        Timer::after(Duration::from_millis(50)).await;
    }
}
