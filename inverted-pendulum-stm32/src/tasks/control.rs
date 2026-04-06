use core::sync::atomic::Ordering;

use crate::config::*;
use crate::controller::{ControlMode, ControlSystem, State};
use crate::driver::adc::{get_currents, get_theta};
use crate::driver::motor::Motors;
use embassy_time::{Duration, Ticker};

const DEBUG_DECIMATION: u32 = 500; // 5kHz / 500 = 10Hz 表示

#[embassy_executor::task]
pub async fn control_task(mut motors: Motors) {
    let mut ticker = Ticker::every(Duration::from_hz(CURRENT_CONTROL_FREQUENCY as u64));
    let mut control_system = ControlSystem::new();
    let mut prev_qei_r: i32 = 0;
    let mut prev_qei_l: i32 = 0;
    let mut qei_r_offset: i32 = 0;
    let mut qei_l_offset: i32 = 0;
    let mut was_running = false;
    let mut balance_counter: u32 = 0;
    let mut current_mode = crate::CONTROL_MODE.load(Ordering::Relaxed);
    let mut debug_counter: u32 = 0;

    loop {
        // モード変更検出
        let mode = crate::CONTROL_MODE.load(Ordering::Relaxed);
        if mode != current_mode {
            control_system.set_mode(ControlMode::from_u8(mode));
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
                            theta,
                            pos,
                            vel,
                            qei_r - qei_r_offset,
                            qei_l - qei_l_offset,
                            current_r,
                            current_l
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
