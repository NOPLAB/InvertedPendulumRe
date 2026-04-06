use crate::constants::{clamp, BALANCE_DT, GEAR_RATIO, MOTOR_KE, MOTOR_LA, MOTOR_RA, WHEEL_RADIUS};

use super::ProcessedState;

/// MATLAB で設計した 5 状態の direct MRAC.
/// 状態: [position, velocity, theta, theta_dot, armature_current]
/// 入力: motor voltage
pub struct MracController {
    reference_state: [f32; 5],
    adaptive_gains: [f32; 5],
    current_state: f32,
    initialized: bool,
    prev_voltage: f32,
    startup_timer: f32,
}

impl MracController {
    pub fn new() -> Self {
        Self {
            reference_state: [0.0; 5],
            adaptive_gains: [0.0; 5],
            current_state: 0.0,
            initialized: false,
            prev_voltage: 0.0,
            startup_timer: 0.0,
        }
    }

    pub fn compute_voltage(&mut self, state: &ProcessedState, vin: f32) -> f32 {
        self.current_state =
            step_current_estimate(self.current_state, self.prev_voltage, state.velocity);

        let x = [
            state.position,
            state.velocity,
            soft_zone(state.theta, MRAC_THETA_SOFT_ZONE, MRAC_INNER_ANGLE_GAIN),
            soft_zone(
                state.theta_dot,
                MRAC_THETA_DOT_SOFT_ZONE,
                MRAC_INNER_ANGULAR_VELOCITY_GAIN,
            ),
            soft_zone(
                self.current_state,
                MRAC_CURRENT_SOFT_ZONE,
                MRAC_INNER_CURRENT_GAIN,
            ),
        ];

        if !self.initialized {
            self.reference_state = x;
            self.initialized = true;
            self.startup_timer = 0.0;
        } else {
            let mut x_m_dot = [0.0; 5];
            mat_vec_mul(&MRAC_AM, &self.reference_state, &mut x_m_dot);
            for i in 0..5 {
                self.reference_state[i] += BALANCE_DT * x_m_dot[i];
            }
            self.startup_timer += BALANCE_DT;
        }

        let mut error = [0.0; 5];
        for i in 0..5 {
            error[i] = x[i] - self.reference_state[i];
        }

        let s = dot(&error, &MRAC_PB);
        let x_norm = dot(&x, &x);
        let adaptation_enabled = state.theta.abs() < MRAC_ENABLE_ANGLE_LIMIT
            && state.theta_dot.abs() < MRAC_ENABLE_ANGULAR_VELOCITY_LIMIT
            && self.current_state.abs() < MRAC_ENABLE_CURRENT_LIMIT
            && self.startup_timer >= MRAC_ADAPTATION_DELAY_SEC
            && s.abs() > MRAC_ERROR_DEADZONE;

        if adaptation_enabled {
            let scale = s / (MRAC_NORMALIZATION_EPS + x_norm);
            for i in 0..5 {
                let update =
                    MRAC_GAMMA_DIAG[i] * x[i] * scale - MRAC_SIGMA * self.adaptive_gains[i];
                self.adaptive_gains[i] += BALANCE_DT * update;
                self.adaptive_gains[i] = clamp(
                    self.adaptive_gains[i],
                    -MRAC_MAX_ADAPTIVE_GAIN,
                    MRAC_MAX_ADAPTIVE_GAIN,
                );
            }
        } else {
            for gain in &mut self.adaptive_gains {
                *gain += BALANCE_DT * (-MRAC_SIGMA * *gain);
            }
        }

        let raw_voltage = -dot(&MRAC_KX, &x) - dot(&self.adaptive_gains, &x);
        let max_delta = MRAC_MAX_VOLTAGE_SLEW_RATE * BALANCE_DT;
        let slewed_voltage =
            self.prev_voltage + clamp(raw_voltage - self.prev_voltage, -max_delta, max_delta);
        let available_voltage = if vin > 1.0 {
            vin.min(MRAC_MAX_VOLTAGE)
        } else {
            7.2_f32.min(MRAC_MAX_VOLTAGE)
        };
        let compensated_voltage = apply_low_speed_drive_compensation(
            slewed_voltage,
            state.theta,
            state.theta_dot,
            state.velocity,
        );
        let voltage = clamp(compensated_voltage, -available_voltage, available_voltage);
        self.prev_voltage = voltage;
        voltage
    }

    pub fn reset(&mut self) {
        self.reference_state = [0.0; 5];
        self.adaptive_gains = [0.0; 5];
        self.current_state = 0.0;
        self.initialized = false;
        self.prev_voltage = 0.0;
        self.startup_timer = 0.0;
    }
}

fn dot(a: &[f32; 5], b: &[f32; 5]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3] + a[4] * b[4]
}

fn mat_vec_mul(matrix: &[[f32; 5]; 5], vector: &[f32; 5], out: &mut [f32; 5]) {
    for row in 0..5 {
        out[row] = matrix[row][0] * vector[0]
            + matrix[row][1] * vector[1]
            + matrix[row][2] * vector[2]
            + matrix[row][3] * vector[3]
            + matrix[row][4] * vector[4];
    }
}

fn step_current_estimate(current: f32, voltage: f32, velocity: f32) -> f32 {
    let motor_speed = GEAR_RATIO * velocity / WHEEL_RADIUS;
    let numerator = current + (BALANCE_DT / MOTOR_LA) * (voltage - MOTOR_KE * motor_speed);
    let denominator = 1.0 + (BALANCE_DT * MOTOR_RA / MOTOR_LA);
    numerator / denominator
}

fn soft_zone(value: f32, zone: f32, inner_gain: f32) -> f32 {
    let abs_value = value.abs();
    if abs_value <= zone {
        inner_gain * value
    } else {
        value.signum() * (inner_gain * zone + (abs_value - zone))
    }
}

fn apply_low_speed_drive_compensation(
    voltage: f32,
    theta: f32,
    theta_dot: f32,
    velocity: f32,
) -> f32 {
    let abs_voltage = voltage.abs();
    if abs_voltage < MRAC_MIN_ACTIVE_VOLTAGE {
        0.0
    } else if theta.abs() < MRAC_STICTION_ENABLE_ANGLE_LIMIT
        && theta_dot.abs() < MRAC_STICTION_ENABLE_ANGULAR_VELOCITY_LIMIT
        && velocity.abs() < MRAC_LOW_SPEED_VELOCITY_THRESHOLD
        && abs_voltage < MRAC_MIN_DRIVE_VOLTAGE
    {
        voltage.signum() * MRAC_MIN_DRIVE_VOLTAGE
    } else {
        voltage
    }
}

// Designed in MATLAB/control/design_mrac.m
const MRAC_KX: [f32; 5] = [
    -5.16397779,
    -14.2638837,
    -80.7713301,
    -10.474196,
    0.0624770634,
];

const MRAC_AM: [[f32; 5]; 5] = [
    [0.0, 1.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, -0.385979743, 0.0145371597, 16.8096489],
    [0.0, 0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 50.9798987, -1.92005654, -84.0482443],
    [1747.53902, 3184.25288, 27333.7834, 3544.56718, -10985.6098],
];

#[allow(dead_code)]
const MRAC_P: [[f32; 5]; 5] = [
    [24.1029124, 13.2871724, 28.19529, 3.33081182, -0.0051500996],
    [
        13.2871724,
        10.852679,
        24.2230811,
        2.74708325,
        -0.00440830955,
    ],
    [28.19529, 24.2230811, 78.1225272, 6.63480748, -0.0136787575],
    [
        3.33081182,
        2.74708325,
        6.63480748,
        0.829363463,
        -0.00213913777,
    ],
    [
        -0.0051500996,
        -0.00440830955,
        -0.0136787575,
        -0.00213913777,
        1.09860663e-05,
    ],
];

const MRAC_PB: [f32; 5] = [
    -1.74284251,
    -1.49181372,
    -4.62902114,
    -0.723904491,
    0.00371778895,
];
const MRAC_GAMMA_DIAG: [f32; 5] = [0.024, 0.008, 0.12, 0.016, 0.0016];
const MRAC_SIGMA: f32 = 5.0;
const MRAC_NORMALIZATION_EPS: f32 = 0.35;
const MRAC_MAX_VOLTAGE: f32 = 9.0;
const MRAC_MAX_VOLTAGE_SLEW_RATE: f32 = 150.0;
const MRAC_ADAPTATION_DELAY_SEC: f32 = 0.5;
const MRAC_ERROR_DEADZONE: f32 = 0.03;
const MRAC_MIN_ACTIVE_VOLTAGE: f32 = 0.1;
const MRAC_MIN_DRIVE_VOLTAGE: f32 = 0.5;
const MRAC_LOW_SPEED_VELOCITY_THRESHOLD: f32 = 0.02;
const MRAC_STICTION_ENABLE_ANGLE_LIMIT: f32 = 0.0436332313;
const MRAC_STICTION_ENABLE_ANGULAR_VELOCITY_LIMIT: f32 = 1.2;
const MRAC_THETA_SOFT_ZONE: f32 = 0.020943951;
const MRAC_THETA_DOT_SOFT_ZONE: f32 = 0.20943951;
const MRAC_INNER_ANGLE_GAIN: f32 = 1.0;
const MRAC_INNER_ANGULAR_VELOCITY_GAIN: f32 = 0.4;
const MRAC_CURRENT_SOFT_ZONE: f32 = 0.2;
const MRAC_INNER_CURRENT_GAIN: f32 = 0.35;
const MRAC_MAX_ADAPTIVE_GAIN: f32 = 2.0;
const MRAC_ENABLE_ANGLE_LIMIT: f32 = 0.261799388;
const MRAC_ENABLE_ANGULAR_VELOCITY_LIMIT: f32 = 6.0;
const MRAC_ENABLE_CURRENT_LIMIT: f32 = 1.2;
