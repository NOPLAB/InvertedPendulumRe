use super::hardware::MOTOR_KT;
use super::hardware::WHEEL_RADIUS;
use super::hardware::GEAR_RATIO;

// Control
pub const CURRENT_CONTROL_FREQUENCY: u32 = 5_000; // [Hz] 電流制御ループ
pub const BALANCE_CONTROL_FREQUENCY: u32 = 1_000; // [Hz] 振り子制御ループ
pub const BALANCE_DECIMATION: u32 = CURRENT_CONTROL_FREQUENCY / BALANCE_CONTROL_FREQUENCY;
pub const CURRENT_DT: f32 = 1.0 / CURRENT_CONTROL_FREQUENCY as f32;
pub const BALANCE_DT: f32 = 1.0 / BALANCE_CONTROL_FREQUENCY as f32;
pub const MAX_FORCE: f32 = 10.0;
pub const MAX_VOLTAGE: f32 = 12.0;

pub const FORCE_TO_CURRENT: f32 = WHEEL_RADIUS / (GEAR_RATIO * MOTOR_KT * 2.0);

// Control modes
pub const NUM_MODES: u8 = 5;

// Utility functions
pub fn clamp(value: f32, min: f32, max: f32) -> f32 {
    if value > max {
        max
    } else if value < min {
        min
    } else {
        value
    }
}

pub fn lpf_alpha(cutoff_freq: f32, sample_time: f32) -> f32 {
    let tau = 1.0 / (2.0 * core::f32::consts::PI * cutoff_freq);
    sample_time / (tau + sample_time)
}
