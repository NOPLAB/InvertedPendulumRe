use embassy_stm32::mode::Async;
use embassy_stm32::usart::{UartRx, UartTx};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use inverted_pendulum_protocol::{
    Command, SensorData, MAX_COMMAND_FRAME_SIZE, MAX_SENSOR_FRAME_SIZE,
};

/// 送信用センサデータ (control_task → uart_tx)
pub static TX_SENSOR_SIGNAL: Signal<CriticalSectionRawMutex, SensorData> = Signal::new();

/// 受信コマンド (uart_rx → main)
pub static RX_COMMAND_SIGNAL: Signal<CriticalSectionRawMutex, Command> = Signal::new();

#[embassy_executor::task]
pub async fn uart_tx_task(mut tx: UartTx<'static, Async>) {
    info!("uart_tx_task started");
    let mut frame_buf = [0u8; MAX_SENSOR_FRAME_SIZE];
    loop {
        let data = TX_SENSOR_SIGNAL.wait().await;
        match data.to_frame(&mut frame_buf) {
            Ok(frame_len) => {
                if let Err(_e) = tx.write(&frame_buf[..frame_len]).await {
                    warn!("uart tx error");
                }
            }
            Err(_) => {
                warn!("sensor frame encode error");
            }
        }
    }
}

#[embassy_executor::task]
pub async fn uart_rx_task(mut rx: UartRx<'static, Async>) {
    info!("uart_rx_task started");
    let mut buf = [0u8; 1];
    let mut frame_buf = [0u8; MAX_COMMAND_FRAME_SIZE];
    let mut frame_pos: usize = 0;
    loop {
        match rx.read(&mut buf).await {
            Ok(_) => {
                let byte = buf[0];
                if byte == 0x00 {
                    if frame_pos > 0 {
                        match Command::from_frame(&frame_buf[..frame_pos]) {
                            Ok(cmd) => RX_COMMAND_SIGNAL.signal(cmd),
                            Err(_) => warn!("command decode failed"),
                        }
                        frame_pos = 0;
                    }
                } else if frame_pos < frame_buf.len() {
                    frame_buf[frame_pos] = byte;
                    frame_pos += 1;
                } else {
                    frame_pos = 0;
                }
            }
            Err(_) => warn!("uart rx error"),
        }
    }
}
