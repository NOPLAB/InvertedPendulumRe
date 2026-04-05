use crate::constants::*;

use super::ProcessedState;

/// LQR状態フィードバック制御器（力の計算のみ）
pub struct LqrController {
    k_pos: f32,
    k_vel: f32,
    k_ang: f32,
    k_ang_vel: f32,
}

impl LqrController {
    pub fn new() -> Self {
        Self {
            k_pos: K_POSITION,
            k_vel: K_VELOCITY,
            k_ang: K_ANGLE,
            k_ang_vel: K_ANGULAR_VELOCITY,
        }
    }

    /// u = -K * x
    pub fn compute_force(&self, state: &ProcessedState) -> f32 {
        -(self.k_pos * state.position
            + self.k_vel * state.velocity
            + self.k_ang * state.theta
            + self.k_ang_vel * state.theta_dot)
    }

    pub fn reset(&mut self) {
        // LQRは状態を持たないのでリセット不要
    }
}
