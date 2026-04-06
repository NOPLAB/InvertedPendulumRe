use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use inverted_pendulum_protocol::{Command, SensorData, MAX_COMMAND_FRAME_SIZE, MAX_SENSOR_FRAME_SIZE};
use log::{info, warn};

/// 最新センサデータ (uart_rx → bridge)
pub static SENSOR_SIGNAL: Signal<CriticalSectionRawMutex, SensorData> = Signal::new();

/// コマンドキュー (ble/wifi → uart_tx)
pub static COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, Command, 4> = Channel::new();

/// UART受信タスク: COBSフレームを読み取り、SensorDataをデコード
#[embassy_executor::task]
pub async fn uart_rx_task(mut rx: esp_hal::uart::UartRx<'static, esp_hal::Async>) {
    info!("uart_rx_task started");
    let mut buf = [0u8; 1];
    let mut frame_buf = [0u8; MAX_SENSOR_FRAME_SIZE];
    let mut frame_pos: usize = 0;

    loop {
        match rx.read_async(&mut buf).await {
            Ok(n) if n > 0 => {
                let byte = buf[0];
                if byte == 0x00 {
                    if frame_pos > 0 {
                        match SensorData::from_frame(&frame_buf[..frame_pos]) {
                            Ok(data) => {
                                SENSOR_SIGNAL.signal(data);
                            }
                            Err(_e) => {
                                warn!("sensor frame decode failed");
                            }
                        }
                        frame_pos = 0;
                    }
                } else if frame_pos < frame_buf.len() {
                    frame_buf[frame_pos] = byte;
                    frame_pos += 1;
                } else {
                    // フレームバッファオーバーフロー、リセット
                    frame_pos = 0;
                }
            }
            Ok(_) => {}
            Err(_e) => {
                warn!("uart rx error");
            }
        }
    }
}

/// UART送信タスク: コマンドをCOBSフレームとしてシリアル送信
#[embassy_executor::task]
pub async fn uart_tx_task(mut tx: esp_hal::uart::UartTx<'static, esp_hal::Async>) {
    info!("uart_tx_task started");
    let mut frame_buf = [0u8; MAX_COMMAND_FRAME_SIZE];

    loop {
        let cmd = COMMAND_CHANNEL.receive().await;
        match cmd.to_frame(&mut frame_buf) {
            Ok(frame_len) => {
                let mut written = 0;
                while written < frame_len {
                    match tx.write_async(&frame_buf[written..frame_len]).await {
                        Ok(n) => written += n,
                        Err(_e) => {
                            warn!("uart tx error");
                            break;
                        }
                    }
                }
            }
            Err(_e) => {
                warn!("command frame encode failed");
            }
        }
    }
}
