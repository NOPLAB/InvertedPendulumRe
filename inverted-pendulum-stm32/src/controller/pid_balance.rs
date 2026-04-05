use crate::constants::*;
use crate::pid::Pid;

use super::ProcessedState;

/// カスケードPIDバランス制御器（角度PID + 位置PID）
pub struct PidBalanceController {
    angle_pid: Pid,
    position_pid: Pid,
}

impl PidBalanceController {
    pub fn new() -> Self {
        Self {
            angle_pid: Pid::new(
                BALANCE_ANGLE_KP,
                BALANCE_ANGLE_KI,
                BALANCE_ANGLE_KD,
                BALANCE_DT,
                -MAX_FORCE,
                MAX_FORCE,
            ),
            position_pid: Pid::new(
                BALANCE_POS_KP,
                BALANCE_POS_KI,
                BALANCE_POS_KD,
                BALANCE_DT,
                -MAX_FORCE,
                MAX_FORCE,
            ),
        }
    }

    /// 角度PIDと位置PIDの出力を合算して力を計算
    pub fn compute_force(&mut self, state: &ProcessedState) -> f32 {
        // LQRと同じ符号規則: theta>0 → force>0, position>0 → force>0
        let u_theta = self.angle_pid.update(state.theta, 0.0);
        let u_x = self.position_pid.update(state.position, 0.0);
        u_theta + u_x
    }

    pub fn reset(&mut self) {
        self.angle_pid.reset();
        self.position_pid.reset();
    }
}
