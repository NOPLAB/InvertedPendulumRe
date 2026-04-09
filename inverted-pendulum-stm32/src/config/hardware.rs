// Motor constants
pub const MOTOR_KT: f32 = 0.0186; // Torque constant [Nm/A]
#[allow(dead_code)]
pub const MOTOR_KE: f32 = 0.0186; // Back EMF constant [V*s/rad]
#[allow(dead_code)]
pub const MOTOR_RA: f32 = 32.4; // Armature resistance [Ohm]
#[allow(dead_code)]
pub const MOTOR_LA: f32 = 2.955e-3; // Armature inductance [H]

// Mechanical constants
pub const WHEEL_RADIUS: f32 = 0.0255; // [m]
pub const GEAR_RATIO: f32 = 6.67;

// ADC constants
pub const ADC_RESOLUTION: u32 = 4096; // 12-bit
pub const ADC_VREF: f32 = 3.3;

pub const POT_ELECTRICAL_ANGLE: f32 = 333.3; // [deg]
pub const POT_SUPPLY_VOLTAGE: f32 = 5.0;
pub const VOLTAGE_DIVIDER_RATIO: f32 = ADC_VREF / POT_SUPPLY_VOLTAGE;
pub const ADC_TO_RAD: f32 =
    (POT_ELECTRICAL_ANGLE * (2.0 * core::f32::consts::PI / 360.0)) * VOLTAGE_DIVIDER_RATIO;

// Current sensing
pub const AMPLIFICATION_FACTOR: f32 = 150.0;
pub const SHUNT_RESISTOR: f32 = 0.010; // [Ohm]

// Encoder
pub const ENCODER_PULSES_PER_REV: i32 = 12 * 4; // 12 pulses * X4
#[allow(dead_code)]
pub const PULSE_TO_RAD: f32 = (2.0 * core::f32::consts::PI) / (12.0 * 4.0);
pub const PULSE_TO_POSITION: f32 =
    (2.0 * core::f32::consts::PI * WHEEL_RADIUS) / ((ENCODER_PULSES_PER_REV as f32) * GEAR_RATIO);

// Motor PWM
pub const MOTOR_PWM_FREQUENCY: u32 = 50_000; // [Hz]

// 電源電圧分圧比 (2.4kΩ + 750Ω 分圧回路: R10, R13)
pub const VIN_DIVIDER_GAIN: f32 = (2400.0 + 750.0) / 750.0; // ADC電圧 → 実電圧

// Utility functions
pub fn adc_to_radians(ad_value: u16, zero_offset: u16) -> f32 {
    let normalized_ad = (ad_value as f32) / (ADC_RESOLUTION as f32);
    let normalized_offset = (zero_offset as f32) / (ADC_RESOLUTION as f32);
    -(normalized_ad - normalized_offset) * ADC_TO_RAD
}

pub fn adc_to_current(adc_value: u16) -> f32 {
    let normalized = (adc_value as f32) / (ADC_RESOLUTION as f32);
    normalized * ADC_VREF / AMPLIFICATION_FACTOR / SHUNT_RESISTOR
}

pub fn adc_to_current_with_offset(adc_value: u16, offset: u16) -> f32 {
    let compensated = adc_value.saturating_sub(offset);
    adc_to_current(compensated)
}

pub fn adc_to_vin(adc_value: u16) -> f32 {
    let normalized = (adc_value as f32) / (ADC_RESOLUTION as f32);
    normalized * ADC_VREF * VIN_DIVIDER_GAIN
}

pub fn pulses_to_position(pulses: i32) -> f32 {
    (pulses as f32) * PULSE_TO_POSITION
}
