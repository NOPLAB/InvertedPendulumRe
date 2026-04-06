use super::ProcessedState;

// Designed in MATLAB/control/design_lqr.m
// Q = diag([100, 0, 50, 100]), R = 10
const K_POSITION: f32 = -3.1623;
const K_VELOCITY: f32 = -3.5049;
const K_ANGLE: f32 = -24.9428;
const K_ANGULAR_VELOCITY: f32 = -4.3992;

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
