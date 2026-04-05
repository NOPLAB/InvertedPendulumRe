pub mod lqr;
pub mod pid_balance;

use crate::constants::*;
use crate::filter::LowPassFilter;
use crate::pid::Pid;

use lqr::LqrController;
use pid_balance::PidBalanceController;

/// センサー状態（制御ループへの入力）
pub struct State {
    pub theta: f32,      // 振り子角度 [rad]
    pub position_r: f32, // 右車輪位置 [m]
    pub position_l: f32, // 左車輪位置 [m]
    pub velocity_r: f32, // 右車輪速度 [m/s]
    pub velocity_l: f32, // 左車輪速度 [m/s]
    pub current_r: f32,  // 右モーター電流 [A]
    pub current_l: f32,  // 左モーター電流 [A]
    pub vin: f32,        // バッテリー電圧 [V]
}

/// モーター出力デューティ比
pub struct MotorOutput {
    pub left: f32,
    pub right: f32,
}

/// 前処理済み状態（内部制御器への入力）
pub struct ProcessedState {
    pub position: f32,
    pub velocity: f32,
    pub theta: f32,
    pub theta_dot: f32,
}

/// 制御モード
#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
pub enum ControlMode {
    Pid = 0,
    Lqr = 1,
}

impl ControlMode {
    pub fn from_u8(val: u8) -> Self {
        match val {
            0 => ControlMode::Pid,
            _ => ControlMode::Lqr,
        }
    }
}

/// 統合制御システム
/// 共通の前処理・電流制御パイプラインと、モード切り替え可能な力計算を統合
pub struct ControlSystem {
    mode: ControlMode,
    lqr: LqrController,
    pid_balance: PidBalanceController,

    // theta前処理
    theta_filter: LowPassFilter,
    prev_theta: f32,
    theta_vel: f32,

    // 電流制御（共通パイプライン）
    pid_l: Pid,
    pid_r: Pid,
    current_filter_l: LowPassFilter,
    current_filter_r: LowPassFilter,
}

impl ControlSystem {
    pub fn new() -> Self {
        Self {
            mode: ControlMode::Lqr,
            lqr: LqrController::new(),
            pid_balance: PidBalanceController::new(),
            theta_filter: LowPassFilter::new(DT, THETA_FILTER_CUTOFF),
            prev_theta: 0.0,
            theta_vel: 0.0,
            pid_l: Pid::new(
                CURRENT_PID_KP,
                CURRENT_PID_KI,
                CURRENT_PID_KD,
                DT,
                -MAX_VOLTAGE,
                MAX_VOLTAGE,
            ),
            pid_r: Pid::new(
                CURRENT_PID_KP,
                CURRENT_PID_KI,
                CURRENT_PID_KD,
                DT,
                -MAX_VOLTAGE,
                MAX_VOLTAGE,
            ),
            current_filter_l: LowPassFilter::new(DT, CURRENT_FILTER_CUTOFF),
            current_filter_r: LowPassFilter::new(DT, CURRENT_FILTER_CUTOFF),
        }
    }

    pub fn set_mode(&mut self, mode: ControlMode) {
        if self.mode != mode {
            self.mode = mode;
            // 新モードの内部状態をリセット
            match mode {
                ControlMode::Lqr => self.lqr.reset(),
                ControlMode::Pid => self.pid_balance.reset(),
            }
        }
    }

    /// センサー状態からモーター出力を計算
    /// パイプライン: 前処理 → 力計算(LQR/PID) → 電流PID → デューティ比
    pub fn update(&mut self, state: &State) -> MotorOutput {
        // 前処理: 左右平均
        let position = (state.position_r + state.position_l) / 2.0;
        let velocity = (state.velocity_r + state.velocity_l) / 2.0;

        // thetaフィルタリング・角速度計算
        let theta = self.theta_filter.update(state.theta);
        self.theta_vel = (theta - self.prev_theta) / DT;
        self.prev_theta = theta;

        let processed = ProcessedState {
            position,
            velocity,
            theta,
            theta_dot: self.theta_vel,
        };

        // モード依存: 力の計算
        let force = match self.mode {
            ControlMode::Lqr => self.lqr.compute_force(&processed),
            ControlMode::Pid => self.pid_balance.compute_force(&processed),
        };
        let force = clamp(force, -MAX_FORCE, MAX_FORCE);

        // 共通パイプライン: 力 → 電流目標 → PID → 電圧 → デューティ比
        let target_current = force * FORCE_TO_CURRENT;

        let filtered_l = self.current_filter_l.update(state.current_l);
        let filtered_r = self.current_filter_r.update(state.current_r);

        let voltage_l = self.pid_l.update(target_current, filtered_l);
        let voltage_r = self.pid_r.update(target_current, filtered_r);

        let vin = if state.vin > 1.0 { state.vin } else { 7.2 };
        let duty_l = clamp(voltage_l / vin, -1.0, 1.0);
        let duty_r = clamp(voltage_r / vin, -1.0, 1.0);

        MotorOutput {
            left: duty_l,
            right: duty_r,
        }
    }

    pub fn reset(&mut self) {
        self.theta_filter.reset();
        self.prev_theta = 0.0;
        self.theta_vel = 0.0;
        self.pid_l.reset();
        self.pid_r.reset();
        self.current_filter_l.reset();
        self.current_filter_r.reset();
        self.lqr.reset();
        self.pid_balance.reset();
    }
}
