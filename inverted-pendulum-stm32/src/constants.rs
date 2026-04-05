// Motor constants
pub const MOTOR_KT: f32 = 0.0186; // Torque constant [Nm/A]
pub const MOTOR_KE: f32 = 0.0186; // Back EMF constant [V*s/rad]
pub const MOTOR_RA: f32 = 32.4; // Armature resistance [Ohm]

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
    (POT_ELECTRICAL_ANGLE * (2.0 * core::f32::consts::PI / 360.0)) / VOLTAGE_DIVIDER_RATIO;

// Current sensing
pub const AMPLIFICATION_FACTOR: f32 = 150.0;
pub const SHUNT_RESISTOR: f32 = 0.010; // [Ohm]

// Encoder
pub const ENCODER_PULSES_PER_REV: i32 = 12 * 4; // 12 pulses * X4
pub const PULSE_TO_RAD: f32 = (2.0 * core::f32::consts::PI) / (12.0 * 4.0);
pub const PULSE_TO_POSITION: f32 =
    (2.0 * core::f32::consts::PI * WHEEL_RADIUS) / ((ENCODER_PULSES_PER_REV as f32) * GEAR_RATIO);

// Control
pub const CONTROL_FREQUENCY: u32 = 2000; // [Hz]
pub const DT: f32 = 1.0 / CONTROL_FREQUENCY as f32;
pub const MAX_FORCE: f32 = 10.0;
pub const MAX_VOLTAGE: f32 = 12.0;

// Motor PWM
pub const MOTOR_PWM_FREQUENCY: u32 = 50_000; // [Hz]

// Filter
pub const THETA_FILTER_CUTOFF: f32 = 500.0; // [Hz]
pub const CURRENT_FILTER_CUTOFF: f32 = 500.0; // [Hz]

// Current PID
pub const CURRENT_PID_KP: f32 = 0.928;
pub const CURRENT_PID_KI: f32 = 10178.8;
pub const CURRENT_PID_KD: f32 = 0.0;

// LQR gains: [position, velocity, angle, angular_velocity]
pub const K_POSITION: f32 = -7.0711;
pub const K_VELOCITY: f32 = -7.9196;
pub const K_ANGLE: f32 = -30.1895;
pub const K_ANGULAR_VELOCITY: f32 = -3.7929;

pub const FORCE_TO_CURRENT: f32 = WHEEL_RADIUS / (GEAR_RATIO * MOTOR_KT * 2.0);

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
    let compensated = if adc_value >= offset {
        adc_value - offset
    } else {
        0
    };
    adc_to_current(compensated)
}

pub fn pulses_to_position(pulses: i32) -> f32 {
    (pulses as f32) * PULSE_TO_POSITION
}

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
