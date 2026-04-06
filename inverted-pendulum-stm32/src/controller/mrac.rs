use crate::constants::{clamp, BALANCE_DT};

use super::ProcessedState;

/// MATLAB で設計した 4 状態の LQR アンカー型 MRAC.
/// 状態: [position, velocity, theta, theta_dot]
/// 特徴量: [state; 1]
pub struct MracController {
    reference_state: [f32; 4],
    adaptive_gains: [f32; 5],
    initialized: bool,
    startup_timer: f32,
}

impl MracController {
    pub fn new() -> Self {
        Self {
            reference_state: [0.0; 4],
            adaptive_gains: [0.0; 5],
            initialized: false,
            startup_timer: 0.0,
        }
    }

    pub fn compute_force(&mut self, state: &ProcessedState) -> f32 {
        let x = [state.position, state.velocity, state.theta, state.theta_dot];
        let phi = [x[0], x[1], x[2], x[3], 1.0];

        if !self.initialized {
            self.reference_state = x;
            self.initialized = true;
            self.startup_timer = 0.0;
        } else {
            let mut x_m_dot = [0.0; 4];
            mat_vec_mul(&MRAC_AM, &self.reference_state, &mut x_m_dot);
            for (reference_state, x_m_dot_i) in self.reference_state.iter_mut().zip(x_m_dot.iter())
            {
                *reference_state += BALANCE_DT * *x_m_dot_i;
            }
            self.startup_timer += BALANCE_DT;
        }

        let mut error = [0.0; 4];
        for i in 0..4 {
            error[i] = x[i] - self.reference_state[i];
        }

        let s = dot4(&error, &MRAC_PB);
        let phi_norm = dot5(&phi, &phi);
        let adaptation_enabled = state.theta.abs() < MRAC_ENABLE_ANGLE_LIMIT
            && state.theta_dot.abs() < MRAC_ENABLE_ANGULAR_VELOCITY_LIMIT
            && state.position.abs() < MRAC_ENABLE_POSITION_LIMIT
            && state.velocity.abs() < MRAC_ENABLE_VELOCITY_LIMIT
            && self.startup_timer >= MRAC_ADAPTATION_DELAY_SEC
            && s.abs() > MRAC_ERROR_DEADZONE;

        if adaptation_enabled {
            let scale = s / (MRAC_NORMALIZATION_EPS + phi_norm);
            for i in 0..5 {
                let update =
                    MRAC_GAMMA_DIAG[i] * phi[i] * scale - MRAC_SIGMA * self.adaptive_gains[i];
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

        let nominal_force = -dot4(&MRAC_KX, &x);
        let adaptive_force = clamp(
            dot5(&self.adaptive_gains, &phi),
            -MRAC_MAX_ADAPTIVE_FORCE,
            MRAC_MAX_ADAPTIVE_FORCE,
        );
        nominal_force - adaptive_force
    }

    pub fn reset(&mut self) {
        self.reference_state = [0.0; 4];
        self.adaptive_gains = [0.0; 5];
        self.initialized = false;
        self.startup_timer = 0.0;
    }
}

fn dot4(a: &[f32; 4], b: &[f32; 4]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]
}

fn dot5(a: &[f32; 5], b: &[f32; 5]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3] + a[4] * b[4]
}

fn mat_vec_mul(matrix: &[[f32; 4]; 4], vector: &[f32; 4], out: &mut [f32; 4]) {
    for row in 0..4 {
        out[row] = matrix[row][0] * vector[0]
            + matrix[row][1] * vector[1]
            + matrix[row][2] * vector[2]
            + matrix[row][3] * vector[3];
    }
}

// Designed in MATLAB/control/design_mrac.m
const MRAC_KX: [f32; 4] = [-3.162_277_7, -3.504_865_6, -24.942_75, -4.399_181_4];

const MRAC_AM: [[f32; 4]; 4] = [
    [0.0, 1.0, 0.0, 0.0],
    [5.475_094_3, 6.068_243_5, 42.799_32, 7.631_178],
    [0.0, 0.0, 0.0, 1.0],
    [-27.375_473, -30.341_217, -164.94661, -40.003_26],
];

const MRAC_PB: [f32; 4] = [-1.8973666, -2.0386274, -8.406_606, -1.395_032_3];
const MRAC_GAMMA_DIAG: [f32; 5] = [0.035, 0.006, 0.12, 0.012, 0.08];
const MRAC_SIGMA: f32 = 2.0;
const MRAC_NORMALIZATION_EPS: f32 = 0.1;
const MRAC_ADAPTATION_DELAY_SEC: f32 = 0.75;
const MRAC_ERROR_DEADZONE: f32 = 0.002;
const MRAC_MAX_ADAPTIVE_GAIN: f32 = 1.5;
const MRAC_MAX_ADAPTIVE_FORCE: f32 = 1.5;
const MRAC_ENABLE_ANGLE_LIMIT: f32 = 0.174_532_92;
const MRAC_ENABLE_ANGULAR_VELOCITY_LIMIT: f32 = 3.5;
const MRAC_ENABLE_POSITION_LIMIT: f32 = 0.35;
const MRAC_ENABLE_VELOCITY_LIMIT: f32 = 1.2;
