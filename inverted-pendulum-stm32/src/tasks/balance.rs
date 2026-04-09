use core::sync::atomic::Ordering;

use crate::config::*;
use crate::controller::{BalanceController, BalanceState, ControlMode};
use crate::driver::adc::get_theta;
use embassy_time::{Duration, Ticker};

const DEBUG_INTERVAL: u32 = BALANCE_CONTROL_FREQUENCY / 10; // 1kHz / 10Hz = 100

#[embassy_executor::task]
pub async fn balance_task() {
    let mut ticker = Ticker::every(Duration::from_hz(BALANCE_CONTROL_FREQUENCY as u64));
    let mut controller = BalanceController::new();
    let mut prev_qei_r: i32 = 0;
    let mut prev_qei_l: i32 = 0;
    let mut qei_r_offset: i32 = 0;
    let mut qei_l_offset: i32 = 0;
    let mut was_running = false;
    let mut current_mode = crate::CONTROL_MODE.load(Ordering::Relaxed);
    let mut debug_counter: u32 = 0;
    let mut uart_counter: u32 = 0;

    loop {
        // モード変更検出
        let mode = crate::CONTROL_MODE.load(Ordering::Relaxed);
        if mode != current_mode {
            controller.set_mode(ControlMode::from_u8(mode));
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
                controller.reset();
            }

            let theta = get_theta();

            let position_r = pulses_to_position(qei_r - qei_r_offset);
            let position_l = -pulses_to_position(qei_l - qei_l_offset);
            let velocity_r = pulses_to_position(qei_r - prev_qei_r) / BALANCE_DT;
            let velocity_l = -pulses_to_position(qei_l - prev_qei_l) / BALANCE_DT;
            prev_qei_r = qei_r;
            prev_qei_l = qei_l;

            let state = BalanceState {
                theta,
                position_r,
                position_l,
            };
            let target_current = controller.update(&state);

            // AtomicU32 経由で電流制御タスクに渡す
            crate::TARGET_CURRENT.store(target_current.to_bits(), Ordering::Relaxed);

            // UART送信 (10Hz = バランスループ100回に1回)
            uart_counter += 1;
            if uart_counter >= (BALANCE_CONTROL_FREQUENCY / 10) {
                uart_counter = 0;
                let pos = (position_r + position_l) / 2.0;
                let vel = (velocity_r + velocity_l) / 2.0;
                let sensor_data = inverted_pendulum_protocol::SensorData {
                    theta,
                    position: pos,
                    velocity: vel,
                    theta_dot: 0.0,
                    current_l: 0.0,
                    current_r: 0.0,
                    control_mode: crate::CONTROL_MODE
                        .load(core::sync::atomic::Ordering::Relaxed),
                    running: 1,
                };
                crate::tasks::uart::TX_SENSOR_SIGNAL.signal(sensor_data);
            }

            // デバッグモード: センサー値を10Hzで表示
            if current_mode == ControlMode::Debug as u8 {
                debug_counter += 1;
                if debug_counter >= DEBUG_INTERVAL {
                    debug_counter = 0;
                    let pos = (position_r + position_l) / 2.0;
                    let vel = (velocity_r + velocity_l) / 2.0;
                    info!(
                        "th={} pos={} vel={} qR={} qL={}",
                        theta,
                        pos,
                        vel,
                        qei_r - qei_r_offset,
                        qei_l - qei_l_offset,
                    );
                }
            }
        } else {
            // 停止時: target_current をゼロに
            crate::TARGET_CURRENT.store(0f32.to_bits(), Ordering::Relaxed);
        }

        was_running = running;
        ticker.next().await;
    }
}
